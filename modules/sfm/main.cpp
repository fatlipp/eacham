#include "base/tools/Tools2d.h"
#include "base/features/FeatureExtractorSift.h"
#include "base/features/FeatureMatcherFlann.h"
#include "base/features/FeaturePipelineCv.h"

#include "sfm/config/ConfigParser.h"
#include "sfm/config/SfmConfig.h"
#include "sfm/reconstruction/ReconstructionManager.h"
#include "sfm/reconstruction/Triangulator.h"
#include "sfm/reconstruction/BundleAdjuster.h"

#include "sfm/data/Frame.h"
#include "sfm/data/Types.h"
#include "sfm/data/Graph.h"
#include "sfm/data/Map.h"
#include "sfm/utils/Saver.h"
#include "sfm/utils/Utils.h"

#include "sfm/data_source/MonoImageReader.h"
#include "sfm/data_source/SfmInputSource.h"

#include "sfm/view/Render.h"
#include "sfm/view/FrameView.h"
#include "sfm/view/GraphView.h"
#include "sfm/view/CloudView.h"
#include "sfm/view/MapView.h"
#include "sfm/view/Gui.h"

#include <opencv4/opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <execution>

#include <functional>

using namespace eacham;
using namespace eacham::dataset;

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cout << "usage: ./sfm path to a config file" << std::endl;

        return -1;
    }

    // const auto config = parser::Parse<SfmConfig>(argv[1]);
    auto parserFunc = std::bind(&parser::Parse<SfmConfig>, std::placeholders::_1);
    const auto config = utils::CallWithTimer(parserFunc, argv[1]);

    std::cout << "Read data from: " << config.imagesPath << std::endl;
    MonoImageReader imageReader { config.imagesPath };
    dataset::SfmInputSource<MonoImageReader> source { std::move(imageReader) };
    const auto frames = source.GetAll(config.maxDataSize);

    std::cout << "frames: " << frames.size() << std::endl;

    std::cout << "Graph" << std::endl;
    std::shared_ptr<graph_t> graph = std::make_shared<graph_t>();

    std::cout << "Extract Features" << std::endl;
    FeatureExtractorSift extractor{config.maxFeaturesCount};
    FeatureMatcherFlann matcher{config.inliersRatio};
    // FeaturePipelineCv<FeatureExtractorSift, FeatureMatcherFlann> featurePipe{config.maxFeaturesCount};
    std::for_each(std::execution::par_unseq, frames.begin(), frames.end(), 
        [&graph, &extractor, &config](auto& frame) {  
            const auto [features, descriptors] = extractor.Extract(frame.image);
            if (features.size() >= config.minFeaturesCount)
            {
                graph->Create(frame.id, features, descriptors, frame.image);
            }
        });

    std::cout << "Match Features" << std::endl;
    auto startTime = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < frames.size(); ++i)
    {
        std::cout << "frame: " << i << "/" << frames.size() << ": ";
        auto frameTimerStart = std::chrono::high_resolution_clock::now();
        auto node1 = graph->Get(frames[i].id);

        auto iterStart = frames.begin();
        std::advance(iterStart, i + 1);

        std::for_each(std::execution::par_unseq, 
                iterStart, frames.end(),
                [&graph, &matcher, &node1](const auto& frame)
            {
            auto node2 = graph->Get(frame.id);

            auto t12 = std::async(std::launch::async, &FeatureMatcherFlann::Match, &matcher, 
                node1->GetDescriptors(), node2->GetDescriptors());
            auto t21 = std::async(std::launch::async, &FeatureMatcherFlann::Match, &matcher, 
                node2->GetDescriptors(), node1->GetDescriptors());
            auto matches12 = t12.get();
            auto matches21 = t21.get();

            if (matches21.size() < 30 || matches21.size() < 30)
            {
                return;
            }

            std::set<int> bb1;
            std::set<int> bb2;
            match_t bestMatches12;
            match_t bestMatches21;

            for (const auto& [m1, m2] : matches12)
            {
                if (matches21.count(m2) > 0 && matches21[m2] == m1)
                {
                    bb1.insert(m1);
                    bb2.insert(m2);
                    // if (bestMatches21.count(m2) > 0) std::__throw_bad_exception();
                    bestMatches12.insert({m1, m2});
                    bestMatches21.insert({m2, m1});
                }
            }

            if (bestMatches12.size() > 30)
            {
                graph->Connect(node1, node2, bestMatches12);
                graph->Connect(node2, node1, bestMatches21);
            }
        });
        auto frameTimerEnd = std::chrono::high_resolution_clock::now();
        const auto frameDuration = std::chrono::duration_cast<std::chrono::milliseconds>(frameTimerEnd - frameTimerStart).count();
        std::cout << frameDuration << " ms\n";
    }
    const auto endTime = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();

    std::cout << "[Match] time: " << duration << "ms" << std::endl;

    // map
    cv::Mat K = utils::ImageToCameraParams(frames[0].image);
    std::shared_ptr<Map> globalMap = std::make_shared<Map>();

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

    // func();


    ReconstructionManager reconstructor(graph, globalMap, config.initialMaxReprError, config.initialMinTriAngle, config.minPnpInliers);
    std::cout << "Find Best Pair" << std::endl;
    std::pair<unsigned, unsigned> bestPair = utils::FindBestPair(graph, globalMap, reconstructor, K, config.initialMinInliers);
    unsigned prevId = bestPair.first;
    unsigned currentId = bestPair.second;

    std::cout << "prevId: " << prevId << ", currentId: " << currentId << std::endl;

    if (prevId > graph->Size() || currentId > graph->Size())
    {
        return -1;
    }

    std::set<unsigned> excluded;
    excluded.insert(prevId);
    excluded.insert(currentId);
    std::tie(prevId, currentId) = graph->GetBestPairForValid(excluded);

    if (prevId > graph->Size())
    {
        std::cout << "No good pair: " << prevId << std::endl;
        return -1;
    }

    const auto startTime2 = std::chrono::high_resolution_clock::now();

    while (true)
    {
        // while (waitForNextStep) {}
        // waitForNextStep = true;
        if (reconstructor.RecoverPosePnP(prevId, currentId, K))
        {
            TriangulateFrame(currentId, graph, globalMap, K, 2, config.maxReprError, config.minTriAngle);
            RefineBA(currentId, graph, globalMap, K, config.refineOpt);

            // ?? reset if frame a new frame has been added
            excluded = {};
        }

        std::tie(prevId, currentId) = graph->GetBestPairForValid(excluded);

        if (prevId > graph->Size() || currentId > graph->Size())
        {
            break;
        }

        excluded.insert(prevId);
        excluded.insert(currentId);
    }

    if (config.globalOpt.maxIter > 0)
    {
        std::cout << "Global BA" << std::endl;
        RefineBA(-1, graph, globalMap, K, config.globalOpt);
    }
    const auto endTime2 = std::chrono::high_resolution_clock::now();
    const auto durationEnd = std::chrono::duration_cast<std::chrono::milliseconds>(endTime2 - startTime2).count();

    std::cout << "[SfM] time: " << durationEnd << "ms" << std::endl;
    
    std::map<unsigned, std::pair<std::string, Eigen::Matrix4d>> framePositions; 

    unsigned invalidNodes = 0;
    for (const auto [id, node] : graph->GetNodes())
    {
        if (node->IsValid())
        {
            framePositions.insert({id, {frames[id].name, node->GetTransform()}});
        }
        else
        {
            std::cout << "Node: " << id << " is invalid\n";
            ++invalidNodes;
        }
    }
    std::cout << "invalidNodes: " << invalidNodes << " out of " << graph->Size() << std::endl;
    std::cout << "framePositions: " << framePositions.size() << std::endl;

    std::cout << "Saving" << std::endl;
    SavePositions(config.outputTransformPath, framePositions, 
        frames[0].image.cols, frames[0].image.rows, 
        K.at<double>(0, 2), 
        K.at<double>(1, 2), 
        K.at<double>(0, 0), 
        K.at<double>(1, 1));

    std::cout << "END..." << std::endl;

    return 0;
}