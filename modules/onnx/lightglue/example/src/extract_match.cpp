
#include "onnx/lightglue/core/FeatureExtractorLightglue.h"
#include "onnx/lightglue/core/FeatureMatcherLightglue.h"

#include "base/tools/Tools2d.h"

#include <iostream>
#include <string>

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

int main(int argc, const char* argv[]) 
{
    std::string imagepath1 = argv[1];
    std::string imagepath2 = argv[2];

    cv::Mat image1 = cv::imread(imagepath1);
    cv::Mat image2 = cv::imread(imagepath2);
    const auto scale1 = 512.0f / std::max(image1.cols, image1.rows);
    const auto scale2 = 512.0f / std::max(image2.cols, image2.rows);

    cv::Mat image1res ;
    cv::Mat image2res ;
    cv::resize(image1, image1res, cv::Size(), scale1, scale1);
    cv::resize(image2, image2res, cv::Size(), scale2, scale2);

    eacham::FeatureExtractorLightglue extractor;
    eacham::FeatureMatcherLightglue matcher;

    const auto [keypoints1, descriptors1] = extractor.Extract(image1res);
    const auto [keypoints2, descriptors2] = extractor.Extract(image2res);

    const auto matches = matcher.Match(
        {eacham::NormalizeKeypoints<eacham::point_t>(keypoints1, image1res.size()), descriptors1},
        {eacham::NormalizeKeypoints<eacham::point_t>(keypoints2, image2res.size()), descriptors2}
    );

    cv::Mat output;
    cv::hconcat(image1, image2, output);

    cv::RNG rng(12345);
    
    for (int i = 0; i < matches.size(); ++i)
    {
        const auto color = cv::Scalar(rng.uniform(0,255), 
                                rng.uniform(0, 255), 
                                rng.uniform(0, 255));


        const auto f1 = keypoints1[std::get<0>(matches[i])];
        cv::Point2f p1 = f1 / scale1;
        cv::circle(output, p1, 3, color, 2);

        const auto f2 = keypoints2[std::get<1>(matches[i])];
        cv::Point2f p2 = f2 / scale2 + cv::Point2f{image1.cols, 0};
        cv::circle(output, p2, 3, color, 2);

        cv::line(output, p1, p2, color, 2);
    }

    cv::resize(output, output, cv::Size(), 0.7, 0.7);
    cv::imshow("output", output);
    cv::waitKey(0);

    return 0;
}
