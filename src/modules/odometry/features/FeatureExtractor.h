#pragma once

#include <opencv4/opencv2/opencv.hpp>

class FeatureExtractor
{
public:
    FeatureExtractor()
    {
        this->detector = cv::ORB::create(1000);
    }

    std::tuple<std::vector<cv::KeyPoint>, cv::Mat> GetFeatures(const cv::Mat &image);

    void GetDescriptors();

private:
    cv::Ptr<cv::FeatureDetector> detector;

};