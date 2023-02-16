#include "FeatureExtractor.h"


std::tuple<std::vector<cv::KeyPoint>, cv::Mat> FeatureExtractor::GetFeatures(const cv::Mat &image)
{
    std::vector<cv::KeyPoint> features;
    cv::Mat descriptors;

    this->detector->detectAndCompute(image, {}, features, descriptors);

    return { features, descriptors };
}

void FeatureExtractor::GetDescriptors()
{
}