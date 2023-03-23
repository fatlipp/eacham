#include "FrameCreator.h"

#include <opencv2/core.hpp>
#include <eigen3/Eigen/Core>

#include "tools/Tools3d.h"

namespace eacham
{

Frame FrameCreator::Create(const stereodata_t& data, const cv::Mat &camera)
{
    const auto [features1, descriptor1] = extractor->GetFeatures(std::get<1>(data));
    
    if (features1.size() < 11)
    {
        return {};
    }

    const auto [features2, descriptor2] = extractor->GetFeatures(std::get<2>(data)); 

    if (features2.size() < 11)
    {
        return {};
    }

    std::vector<std::vector<cv::DMatch>> matches;
    this->mather->knnMatch(descriptor1, descriptor2, matches, 2);

    if (matches.size() < 40)
    {
        return {};
    }

    Frame frame {std::get<0>(data), std::get<1>(data)};

    int pointId = 0;

    std::vector<cv::Point2f> ptsGood1;
    std::vector<cv::Point3f> pts3d1;

    for (const auto& m : matches)
    {
        if (m[0].distance < 0.7f * m[1].distance)
        {
            const int id1 = m[0].queryIdx;
            const int id2 = m[0].trainIdx;

            cv::Point3f pos3d = Get3dPointByStereoPair(features1[id1].pt, features2[id2].pt, camera);

            if (pos3d.z > 0.0f && pos3d.z < 60.0f)
            {
                frame.AddPoint(pointId, pos3d, features1[id1], descriptor1.row(id1));
                ptsGood1.push_back(features1[id1].pt);
                pts3d1.push_back(pos3d);

                ++pointId;
            }
        }
    }

    return frame;
}

}