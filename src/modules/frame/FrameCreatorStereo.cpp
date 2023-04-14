#include "FrameCreatorStereo.h"

#include <opencv2/core.hpp>
#include <eigen3/Eigen/Core>

#include "tools/Tools3d.h"

namespace eacham
{

Frame FrameCreatorStereo::Create(const stereodata_t& data)
{
    const auto [features1, descriptor1] = extractor->GetFeatures(std::get<1>(data));
    
    std::cout << "features1.size() = " << features1.size() << std::endl;

    if (features1.size() < 50)
    {
        return {};
    }


    const auto [features2, descriptor2] = extractor->GetFeatures(std::get<2>(data)); 

    std::cout << "features2.size() = " << features2.size() << std::endl;

    if (features2.size() < 50)
    {
        return {};
    }

    std::vector<std::vector<cv::DMatch>> matches;
    this->matcher->knnMatch(descriptor1, descriptor2, matches, 2);

    std::cout << "matches.size() = " << matches.size() << std::endl;

    if (matches.size() < 40)
    {
        return {};
    }

    Frame frame { std::get<0>(data) };

    int pointId = 0;

    for (const auto& m : matches)
    {
        if (m[0].distance < 0.7f * m[1].distance)
        {
            const int id1 = m[0].queryIdx;
            const int id2 = m[0].trainIdx;

            const cv::Point3f pos3d = tools::Get3dPointByStereoPair(features1[id1].pt, features2[id2].pt, this->cameraData);

            if (pos3d.z > 0.10f && pos3d.z < 70.0f)
            {
                frame.AddPoint(pointId++, pos3d, features1[id1], descriptor1.row(id1));
            }
        }
    }
    std::cout << "pointId = " << pointId << std::endl;


    cv::imshow("im1", std::get<1>(data));
    cv::waitKey(1);

    return frame;
}

}