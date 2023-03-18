#pragma once

#include "types/DataTypes.h"

#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
// #include "opencv2/xfeatures2d.hpp"

class FeatureExtractor
{
public:
    FeatureExtractor()
    {
        this->detector = cv::ORB::create(1000, 1.6f, 6, 19);
        // this->detector = cv::SIFT::create(1000);
        // this->detector = cv::xfeatures2d::SURF::create(1000);
    }

    std::tuple<std::vector<cv::KeyPoint>, cv::Mat> GetFeatures(const cv::Mat &image);

    void GetDescriptors();

    static matcher_t GetMatcher()
    {
        return cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
        return cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
        return cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        // return std::make_unique<cv::BFMatcher>(cv::NORM_HAMMING, false);
        // return std::make_unique<cv::BFMatcher>(cv::NORM_L2, false);
    }


private:
    cv::Ptr<cv::FeatureDetector> detector;


};