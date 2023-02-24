#include "FrameCreator.h"

#include <opencv4/opencv2/core.hpp>
#include <eigen3/Eigen/Core>

#include "tools/Tools3d.h"

namespace odometry
{

Frame FrameCreator::Create(const stereodata_t& data, const cv::Mat &camera)
{
    const auto fd = extractor.GetFeatures(std::get<0>(data));
    const auto [features1, descriptor1] = fd;
    
    if (features1.size() < 500)
    {
        return {};
    }

    const auto [features2, descriptor2] = extractor.GetFeatures(std::get<1>(data)); 

    if (features2.size() < 500)
    {
        return {};
    }

    std::cout << "features1: " << features1.size() << ", descriptor1: cols: " << descriptor1.cols << ", rows: " << descriptor1.rows << std::endl;
    std::cout << "features2: " << features2.size() << ", descriptor2: cols: " << descriptor2.cols << ", rows: " << descriptor2.rows << std::endl;
    
    std::vector<std::vector<cv::DMatch>> matches;
    // this->mather->match(descriptor1, descriptor2, matches);
    this->mather->knnMatch(descriptor1, descriptor2, matches, 2);

    // cv::Mat img_match;
    // cv::drawMatches(std::get<0>(data), features1, std::get<1>(data), features2, matches, img_match);
    // cv::imshow("img_match", img_match);

    if (matches.size() < 100)
    {
        return {};
    }

    Frame frame {fd};

    int pointId = 0;

    std::vector<cv::Point3f> points3d;
    for (const auto& m : matches)
    {
        ///
        ///
        if(m[0].distance < 0.7f * m[1].distance)
        {
            const cv::Point3f pos3d = tools::Get3dPointByStereoPair(features1[m[0].queryIdx].pt, features2[m[0].trainIdx].pt, camera);

            if (pos3d.z > 0.0f && pos3d.z < 30.0F)
            {
                frame.AddPoint(pointId++, pos3d, features1[m[0].queryIdx], descriptor1.row(m[0].queryIdx));
            }
        }
    }

    return frame;
}

}