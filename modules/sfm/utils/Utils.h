#pragma once

#include "sfm/data/Types.h"
#include "sfm/reconstruction/ReconstructionManager.h"

#include <opencv4/opencv2/core.hpp>

namespace eacham::utils
{

cv::Mat ImageToCameraParams(const cv::Mat& image)
{
    const double focal = std::max(image.rows, image.cols) * 1.2;
    const double cx = image.cols * 0.5;
    const double cy = image.rows * 0.5;
    const cv::Mat K = (cv::Mat_<double>(3,3) << focal,  0,      cx, 
                                                0,      focal,  cy, 
                                                0,      0,      1);
    return K;
}

std::pair<unsigned, unsigned> FindBestPair(std::shared_ptr<graph_t> graph, std::shared_ptr<Map> map,
    const ReconstructionManager& reconstructor, const cv::Mat& K, const unsigned minInitialInliers)
{
    for (const auto& [id1, node1] : graph->GetNodes())
    {
        for (const auto& [id2, factor] : node1->GetFactors())
        {
            // if (id2 < id1 + 2) continue;
            MatchTwoView rec1 = reconstructor.RecoverPoseTwoView(id1, id2, K);
            MatchTwoView rec2 = reconstructor.RecoverPoseTwoView(id2, id1, K);

            if (rec1.matches.size() > minInitialInliers && rec2.matches.size() > minInitialInliers)
            {
                graph->FixNode(id1);

                auto& factor2 = node1->GetFactor(id2);
                factor2.quality = rec1.matches.size();

                auto node2 = graph->Get(id2);
                node1->SetTransform(Eigen::Matrix4d::Identity());
                node1->SetValid(true);
                node2->SetTransform(rec1.transform);
                node2->SetValid(true);

                for (const auto& m : rec1.matches)
                {
                    const auto p2d1 = std::get<0>(m);
                    const auto p2d2 = std::get<1>(m);
                    const auto id3d = map->Add(std::get<2>(m), {0.3, 0.3, 0.3});

                    node1->SetPoint3d(p2d1, id3d);
                    node2->SetPoint3d(p2d2, id3d);
                    map->AddObserver(id1, p2d1, id3d);
                    map->AddObserver(id2, p2d2, id3d);
                }

                return {id1, id2};
            }
        }
    }

    return {std::numeric_limits<unsigned>::max(), 
            std::numeric_limits<unsigned>::max()};
}

}