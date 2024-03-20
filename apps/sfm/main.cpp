#include "base/features/FeatureExtractorSift.h"
#include "base/features/FeatureMatcherFlann.h"
#include "base/features/FeaturePipelineCv.h"

#include "base/config/ConfigParser.h"
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

#include "view/Render.h"
#include "view/GraphView.h"
#include "view/MapView.h"

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

    auto graph = std::make_shared<graph_t>();
    auto globalMap = std::make_shared<Map>();
    
    // render
    std::atomic<bool> waitForNextStep = true;
    std::atomic<bool> waitForBA = true;
    std::atomic<bool> start = false;

    auto render = std::make_unique<Render>();
    render->Add(std::make_unique<GraphView>(graph));
    render->Add(std::make_unique<MapView>(globalMap));
    render->SetOnPlayClick([&start](){ start = true; });
    render->SetOnStepClick([&waitForNextStep](){ waitForNextStep = false; });
    render->SetOnBAClick([&waitForBA](){ waitForBA = false; });
    render->Activate();
    while (!start) {}

    std::cout << "Read data from: " << config.imagesPath << std::endl;
    MonoImageReader imageReader { config.imagesPath };
    dataset::SfmInputSource<MonoImageReader> source { std::move(imageReader) };

    const auto frames = source.GetAll(config.maxDataSize);
    std::cout << "frames: " << frames.size() << std::endl;

    std::cout << "Extract Features" << std::endl;
    FeatureExtractorSift extractor{config.maxFeaturesCount};
    FeatureMatcherFlann matcher{config.inliersRatio};
    
    std::for_each(std::execution::par, frames.begin(), frames.end(), 
        [&graph, &extractor, &config](auto& frame) {  
            const auto [features, descriptors] = extractor.Extract(frame.image);
            if (features.size() >= config.minFeaturesCount)
            {
                graph->Create(frame.id, features, descriptors, frame.image);
            }
        });

    std::cout << "Match Features" << std::endl;
    auto startTime = std::chrono::high_resolution_clock::now();
    /// 2
    std::vector<std::pair<unsigned, unsigned>> pairs;
    for (int i = 0; i < frames.size(); ++i)
    {
        for (int j = i + 1; j < frames.size(); ++j)
        {
            pairs.push_back({i, j});
            pairs.push_back({j, i});
        }
    }

    std::unordered_map<unsigned, 
        std::unordered_map<unsigned, unsigned>> checkedBuffer;
    std::mutex mutex;

    std::for_each(std::execution::par_unseq, 
            pairs.begin(), pairs.end(),
            [&graph, &matcher, &mutex, &checkedBuffer, &frames](const auto& pair) {
        const auto frame1 = frames[pair.first].id;
        const auto frame2 = frames[pair.second].id;

        auto node1 = graph->Get(frame1);
        auto node2 = graph->Get(frame2);

        auto t12 = std::async(std::launch::async, &FeatureMatcherFlann::Match, &matcher, 
            node1->GetDescriptors(), node2->GetDescriptors());
        auto matches12 = t12.get();

        if (matches12.size() < 30)
        {
            return;
        }

        const unsigned hash = std::min(frame1, frame2) + std::max(frame1, frame2) * 10000;

        {
            std::lock_guard<std::mutex> lock(mutex);

            if (checkedBuffer.count(hash) == 0)
            {
                checkedBuffer[hash] = std::move(matches12);
                return;
            }
        }

        auto& matches21 = checkedBuffer[hash];

        match_t bestMatches12;
        match_t bestMatches21;

        for (const auto& [m1, m2] : matches12)
        {
            if (matches21.count(m2) > 0 && matches21[m2] == m1)
            {
                bestMatches12[m1] = m2;
                bestMatches21[m2] = m1;
            }
        }

        if (bestMatches12.size() > 30)
        {
            graph->Connect(node1, node2, std::move(bestMatches12));
            graph->Connect(node2, node1, std::move(bestMatches21));
        }
    });

    /// 2 end
    const auto endTime = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
    std::cout << "[Match] time1: " << duration << "ms" << std::endl;

    auto K = utils::ImageToCameraParams(frames[0].image);

    // calc 3d points
    std::cout << "Two View" << std::endl;
    ReconstructionManager reconstructor{graph, 
    globalMap, config.initialMaxReprError, config.initialMinTriAngle, config.minPnpInliers};
    std::cout << "Find Best Pair" << std::endl;
    auto [prevId, currentId] = 
        utils::FindBestPair(graph, globalMap, reconstructor, K, config.initialMinInliers);

    std::cout << "prevId: " << prevId << ", currentId: " << currentId << std::endl;

    if (prevId > graph->Size() || currentId > graph->Size())
    {
        return -1;
    }
    auto node1 = graph->Get(prevId);
    auto node2 = graph->Get(currentId);

    std::cout << "1: " << frames[prevId].name << "" << std::endl;
    std::cout << "2: " << frames[currentId].name << "" << std::endl;

    // cv::imshow("1", node1->GetImage());
    // cv::imshow("2", node2->GetImage());
    // cv::waitKey(0);

    std::set<unsigned> excluded;
    excluded.insert(prevId);
    excluded.insert(currentId);

    unsigned bestCount3d = 0;
    std::tie(prevId, currentId, bestCount3d) = graph->GetBestPairForValid(excluded);

    if (prevId > graph->Size())
    {
        std::cout << "No good pair: " << prevId << std::endl;
        return -1;
    }

    const auto startTime2 = std::chrono::high_resolution_clock::now();

    while (true)
    {
        std::printf("RecoverPosePnP: %d - %d: %d\n", prevId, currentId, bestCount3d);
        std::cout << "names: " << frames[prevId].name << " <-> ";
        std::cout << "" << frames[currentId].name << ".\n";

        if (reconstructor.RecoverPosePnP(prevId, currentId, K))
        {
            TriangulateFrame(currentId, graph, globalMap, K, 2, 
                config.maxReprError, config.minTriAngle);

            std::cout << "RefineBA...\n";
            RefineBA(currentId, graph, globalMap, K, config.refineOpt);

            TriangulateFrame(currentId, graph, globalMap, K, 3,
                config.maxReprError, config.minTriAngle);

            // ?? reset if frame a new frame has been added
            excluded = {};
        }

        std::tie(prevId, currentId, bestCount3d) = graph->GetBestPairForValid(excluded);

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
            std::cout << "Node: " << id << " is invalid: ";
            std::cout << "id: " << frames[id].id << ", ";
            std::cout << "file: " << frames[id].name << ".\n";

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