#include "base/tools/Tools2d.h"
#include "base/features/FeatureExtractorSift.h"
#include "base/features/FeatureMatcherFlann.h"
#include "base/features/FeaturePipelineCv.h"

#include "onnx/lightglue/core/FeatureExtractorLightglue.h"
#include "onnx/lightglue/core/FeatureMatcherLightglue.h"
#include "onnx/lightglue/core/FeaturePipelineLightGlue.h"

#include "sfm/data/ImagesData.h"
#include "sfm/data/Types.h"
#include "sfm/data/Graph.h"
#include "sfm/reconstruction/EpipolarGeometry.h"
#include "sfm/ba/MotionBA.h"
#include "sfm/data/Map.h"
#include "sfm/utils/Triangulator.h"
#include "sfm/utils/Saver.h"

#include "sfm/data_source/MonoCameraDataset.h"

#include "sfm/view/Render.h"
#include "sfm/view/FrameView.h"
#include "sfm/view/GraphView.h"
#include "sfm/view/CloudView.h"
#include "sfm/view/MapView.h"
#include "sfm/view/Gui.h"

#include <opencv4/opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <execution>

using namespace eacham;
using namespace eacham::dataset;

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cout << "usage: ./sfm path_to_images_folder" << std::endl;

        return -1;
    }

    const unsigned startFrameId = argc > 2 ? std::stoi(argv[2]) : 0;
    const unsigned maxFramesCount = argc > 3 ? std::stoi(argv[3]) : 999999;

    std::cout << "Read data" << std::endl;
    DatasetInputData dataInput { 
                         .calibPath = "", 
                         .images = argv[1], 
                         .startFrameId = startFrameId,
                         .gt = "" };

    eacham::dataset::SfmInputSource source {dataInput};

    std::map<unsigned, Frame> frames;
    unsigned frameId = 0;

    while (source.HasNext())
    {
        source.PrepareNext();

        const auto [imageData, gtPosData] = source.Get();
        const auto [gtPos] = gtPosData;
        auto [ts, image, name] = imageData;

        while (image.rows > 1500.0F)
            ResizeImage(image, 0.95);

        std::cout << "ts: " << ts << ", image: " << image.size() << ", gtPos:\n" << gtPos << std::endl;

        frames[frameId] = {
                .id = frameId,
                .image = image.clone(),
                .name = name
            };

        if (++frameId >= maxFramesCount)
        {
            break;
        }
    }

    // graph
    std::cout << "Graph" << std::endl;
    std::shared_ptr<graph_t> graph = std::make_shared<graph_t>();

    // extract features
    std::cout << "Extract Features" << std::endl;

    FeaturePipelineCv<FeatureExtractorSift, FeatureMatcherFlann> featurePipe;
    std::for_each(std::execution::par, frames.begin(), frames.end(), 
        [&graph, &featurePipe](auto& frame) {  
            const auto& [id, image] = frame;
            cv::Mat grayIm = image.image;

            const auto [features, descriptors] = featurePipe.Extract(grayIm);

            std::cout << id << ") " << features.size() << std::endl;

            if (features.size() > 100)
            {
                graph->Create(id, features, descriptors, image.image);
            }
        });

    std::cout << "Match Features" << std::endl;
    std::vector<std::pair<unsigned, unsigned>> checkPairs;
    
    for (auto& [id1, frame1] : frames)
    {
        for (auto& [id2, frame2] : frames)
        {
            if (id1 == id2)
            {
                continue;
            }

            const auto iter = std::find_if(checkPairs.begin(), checkPairs.end(), [&id1, &id2](const auto& pair) {
                    return ((pair.first == id1 && pair.second == id2) || (pair.second == id1 && pair.first == id2));
                });

            if (iter != checkPairs.end())
            {
                continue;
            }

            checkPairs.push_back({id1, id2});
        }
    }

    // map
    std::shared_ptr<Map> globalMap = std::make_shared<Map>();
    
    const double focal = std::max(frames[0].image.rows, frames[0].image.cols) * 1.2;
    const double cx = frames[0].image.cols / 2.0;
    const double cy = frames[0].image.rows / 2.0;
    cv::Mat K = (cv::Mat_<double>(3,3) << focal,  0,      cx, 
                                          0,      focal,  cy, 
                                          0,      0,      1);

    std::pair<unsigned, unsigned> bestPair;
    float bestQuality = 0;
    std::mutex bestMutex;
    
    std::for_each(std::execution::par, 
        checkPairs.cbegin(), 
        checkPairs.cend(), 
        [&graph, &featurePipe, &bestPair, &bestQuality, &bestMutex, &globalMap, &K](const auto& pair12) { 
            const auto [id1, id2] = pair12;

            auto node1 = graph->Get(id1);
            auto node2 = graph->Get(id2);

            const auto matches12 = featurePipe.Match(node1->GetDescriptors(), node2->GetDescriptors());
            const auto matches21 = featurePipe.Match(node2->GetDescriptors(), node1->GetDescriptors());

            std::vector<std::pair<unsigned, unsigned>> bestMatches12;
            std::vector<std::pair<unsigned, unsigned>> bestMatches21;

            for (const auto& [m1, m2] : matches12)
            {
                if (std::find_if(matches21.begin(), matches21.end(), [&m1, &m2](const auto& mm){
                        return mm.first == m2 && mm.second == m1;
                    }) != matches21.end())
                {
                    bestMatches12.push_back({m1, m2});
                    bestMatches21.push_back({m2, m1});
                }
            }

            // std::cout << "matches: [" << id1 << " - " << id2 
            //     << "]: '12': " << matches12.size()
            //     << "]: '21': " << matches21.size() 
            //     << "]: 'good pair': " << bestMatches12.size() 
            //     << std::endl;

            if (bestMatches12.size() > 30)
            {
                graph->Connect(id1, id2, bestMatches12);
                graph->Connect(id2, id1, bestMatches21);
                
                // TODO: BAD!!! Break after finding optimal correspondence 
                // (i.e. more than 200 matches)
                if (bestMatches12.size() > 100)
                {
                    std::lock_guard<std::mutex> lock {bestMutex};
                    RecoverPoseTwoView(id1, id2, graph, K, globalMap, 4.0);
                    RecoverPoseTwoView(id2, id1, graph, K, globalMap, 4.0);

                    auto& factor12 = node1->GetFactor(node2->GetId());
                    auto& factor21 = node2->GetFactor(node1->GetId());

                    if (factor21.quality > 100 && factor21.quality > 100)
                    {
                        if (factor12.quality > bestQuality)
                        {
                            bestQuality = factor12.quality;
                            bestPair = {id1, id2};
                        }
                        if (factor21.quality > bestQuality)
                        {
                            bestQuality = factor21.quality;
                            bestPair = {id2, id1};
                        }
                    }
                }
            }
        });

    // calc 3d points
    std::cout << "Two View" << std::endl;
    // render
    std::atomic<bool> waitForNextStep = true;
    std::atomic<bool> waitForBA = true;

    Render render;
    render.Add(std::make_unique<GraphView>(graph));
    render.Add(std::make_unique<MapView>(globalMap));
    render.SetOnStepClick([&](){ waitForNextStep = false; });
    render.SetOnBAClick([&](){ waitForBA = false; });
    render.Activate();


    unsigned prevId = bestPair.first;
    unsigned currentId = bestPair.second;
    // unsigned prevId = 9999;
    // unsigned currentId = 9999;
    // std::set<unsigned> excluded;
    // while (true)
    // {
    //     std::tie(prevId, currentId) = graph->GetBestPair(excluded);

    //     if (prevId < 999 && prevId < 999)
    //     {
    //         RecoverPoseTwoView(prevId, currentId, graph, K, globalMap);

    //         auto& factor = graph->Get(prevId)->GetFactor(currentId);

    //         if (factor.quality > 80)
    //         {
    //             break;
    //         }

    //         excluded.insert(prevId);
    //         excluded.insert(currentId);

    //         prevId = 999;
    //         currentId = 999;
    //     }
    //     else
    //     {
    //         throw std::runtime_error("No good initial pair");
    //     }
    // }

    // if (prevId > 900 || prevId > 900)
    // {
    //     throw std::runtime_error("No good initial pair after Loop");
    //     return;
    // }

    auto& factor = graph->Get(prevId)->GetFactor(currentId);
    std::cout << "initial pair: " << prevId << " - " << currentId << ", matches: " 
        << factor.matches.size() << ", q: " << factor.quality << std::endl;
    std::cout << "names: " << frames[prevId].name << " -> " << frames[currentId].name << std::endl;

    // DrawMatches("initial", prevId, currentId, graph, 0);

    std::set<unsigned> excluded;
    excluded.insert(prevId);
    excluded.insert(currentId);

    // // auto [prevId, currentId] = graph->GetBestPair();
    // RecoverPoseTwoView(prevId, currentId, graph, K, globalMap);

    // std::cout << "initial pair: " << prevId << " - " << currentId << ", quality: " 
    //     << factor.quality << std::endl;

    if (factor.quality < 100)
    {
        return 0;
    }

    graph->FixNode(prevId);

    graph->Get(prevId)->SetTransform(Eigen::Matrix4d::Identity());
    graph->Get(currentId)->SetTransform(factor.transform);

    graph->Get(prevId)->SetValid(true);
    graph->Get(currentId)->SetValid(true);

    for (const auto& m : factor.matches)
    {
        if (m.triangulatedPointId > 999999)
        {
            continue;
        }

        graph->Get(prevId)->SetPoint3d(m.id1, m.triangulatedPointId);
        graph->Get(currentId)->SetPoint3d(m.id2, m.triangulatedPointId);
        globalMap->AddObserver(prevId, m.id1, m.triangulatedPointId);
        globalMap->AddObserver(currentId, m.id2, m.triangulatedPointId);
    }
    
    std::tie(prevId, currentId) = graph->GetBestPairForValid(excluded);

    while (currentId < 1000)
    {
        // UI: while (waitForNextStep) {}
        waitForNextStep = true;

        if (RecoverPosePnP(prevId, currentId, graph, globalMap, K))
        {
            TriangulateFrame(currentId, graph, globalMap, K, 2);

            EstimateUsingBA(graph, globalMap, K);
            TriangulateFrame(currentId, graph, globalMap, K, 3);

            // TODO: recheck excluded frames
            excluded = {};
        }

        std::tie(prevId, currentId) = graph->GetBestPairForValid(excluded);

        if (prevId < 1000)
            excluded.insert(prevId);
        if (currentId < 1000)
            excluded.insert(currentId);
    }


    // // TODO: process excluded frames???
    // excluded = {};
    // std::tie(prevId, currentId) = graph->GetBestPairForValid(excluded);

    // while (currentId < 1000)
    // {
    //     if (RecoverPosePnP(prevId, currentId, graph, globalMap, K))
    //     {
    //         TriangulateFrame(currentId, graph, globalMap, K, 2);

    //         EstimateUsingBA(graph, globalMap, K);
    //         TriangulateFrame(currentId, graph, globalMap, K, 3);

    //         excluded = {};
    //     }

    //     std::tie(prevId, currentId) = graph->GetBestPairForValid(excluded);
    //     if (prevId < 1000)
    //         excluded.insert(prevId);
    //     if (currentId < 1000)
    //         excluded.insert(currentId);
    // }

    // // TODO: process robust global ba refinement

    std::cout << "excluded frames: " << excluded.size() << std::endl;

    // for (const auto& frameId : excluded)
    // {
    //     std::cout << "frameId: " << frameId << std::endl;
    // }

    // std::cout << "Graph stat:\n size: " << graph->Size() << std::endl;

    std::map<unsigned, Eigen::Matrix4d> framePositions; 

    unsigned invalidNodes = 0;
    for (const auto [id, node] : graph->GetNodes())
    {
        if (node->IsValid())
        {
            framePositions.insert({node->GetId(), node->GetTransform()});
        }
        else
        {
            std::cout << "Node: " << node->GetId() << " is invalid\n";
            ++invalidNodes;
        }
    }
    std::cout << "invalidNodes: " << invalidNodes << " out of " << graph->Size() << std::endl;
    std::cout << "framePositions: " << framePositions.size() << std::endl;

    // for (const auto& [id, transform] : framePositions)
    // {
    //     std::cout << "frameId: " << id << "\n" << transform << std::endl;
    // }
        
    std::cout << "Saving" << std::endl;
    SavePositions("positions.txt", framePositions);

    // TODO: create a lib, return framePositions to caller

    std::cout << "END..." << std::endl;

    return 0;
}