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
        auto [ts, image] = imageData;

        while (image.rows > 1500.0F)
            ResizeImage(image, 0.95);

        std::cout << "ts: " << ts << ", image: " << image.size() << ", gtPos:\n" << gtPos << std::endl;

        frames[frameId] = {
                .id = frameId,
                .image = image.clone()
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
    
    std::for_each(std::execution::par, 
        checkPairs.cbegin(), 
        checkPairs.cend(), 
        [&graph, &featurePipe](const auto& pair12) { 
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

            std::cout << "matches: [" << id1 << " - " << id2 
                << "]: '12': " << matches12.size()
                << "]: '21': " << matches21.size() 
                << "]: 'good pair': " << bestMatches12.size() 
                << std::endl;

            if (bestMatches12.size() > 30)
            {
                graph->Connect(id1, id2, bestMatches12);
                graph->Connect(id2, id1, bestMatches21);
            }
        });

    // map
    std::shared_ptr<Map> globalMap = std::make_shared<Map>();

    // calc 3d points
    std::cout << "Two View" << std::endl;
    
    const double focal = std::max(frames[0].image.rows, frames[0].image.cols) * 1.2;
    const double cx = frames[0].image.cols / 2.0;
    const double cy = frames[0].image.rows / 2.0;
    cv::Mat K = (cv::Mat_<double>(3,3) << focal,  0,      cx, 
                                          0,      focal,  cy, 
                                          0,      0,      1);
    // render
    std::atomic<bool> waitForNextStep = true;
    std::atomic<bool> waitForBA = true;

    Render render;
    render.Add(std::make_unique<GraphView>(graph));
    render.Add(std::make_unique<MapView>(globalMap));
    render.SetOnStepClick([&](){ waitForNextStep = false; });
    render.SetOnBAClick([&](){ waitForBA = false; });
    render.Activate();


    auto [prevId, currentId] = graph->GetBestPair();
    prevId = 0;
    currentId = 1;
    std::cout << "initial pair: " << prevId << " - " << currentId << std::endl;
    RecoverPoseTwoView(prevId, currentId, graph, K, globalMap);

    auto& factor = graph->Get(prevId)->GetFactor(currentId);
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
    
    std::tie(prevId, currentId) = graph->GetBestPairForValid();
    graph->FixNode(prevId);

    std::set<unsigned> excluded = {prevId, currentId};

    while (currentId < 1000)
    {
        // while (waitForNextStep) {}
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

        excluded.insert(prevId);
        excluded.insert(currentId);
    }


    // process excluded frames???
    excluded = {};
    std::tie(prevId, currentId) = graph->GetBestPairForValid(excluded);

    while (currentId < 1000)
    {
        if (RecoverPosePnP(prevId, currentId, graph, globalMap, K))
        {
            TriangulateFrame(currentId, graph, globalMap, K, 2);

            EstimateUsingBA(graph, globalMap, K);
            TriangulateFrame(currentId, graph, globalMap, K, 3);

            excluded = {};
        }

        std::tie(prevId, currentId) = graph->GetBestPairForValid(excluded);
        excluded.insert(prevId);
        excluded.insert(currentId);
    }

    // process robust global ba refinement
    //

    std::cout << "excluded frames: " << excluded.size() << std::endl;

    for (const auto& frameId : excluded)
    {
        std::cout << "frameId: " << frameId << std::endl;
    }

    std::cout << "Graph stat:\n size: " << graph->Size() << std::endl;

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
            std::cout << "Node: " << node->GetId() << " is invalid";
            ++invalidNodes;
        }
    }
    std::cout << "invalidNodes: " << invalidNodes << " out of " << graph->Size() << std::endl;
    std::cout << "framePositions: " << framePositions.size() << std::endl;

    for (const auto& [id, transform] : framePositions)
    {
        std::cout << "frameId: " << id << "\n" << transform << std::endl;
    }

    // return framePositions to caller

    std::cout << "END..." << std::endl;

    return 0;
}