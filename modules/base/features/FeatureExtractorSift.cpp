#include "FeatureExtractorSift.h"

namespace eacham
{

FeatureExtractorSift::FeatureExtractorSift()
{
    this->detector = cv::SIFT::create(20000, 3, 0.009, 10, 1.3);
    // explicit SIFT_Impl( int nfeatures = 0, int nOctaveLayers = 3,
    //                       double contrastThreshold = 0.04, double edgeThreshold = 10,
    //                       double sigma = 1.6, int descriptorType = CV_32F );
}

typename FeatureExtractorSift::ReturnType FeatureExtractorSift::Extract(const cv::Mat &image)
{
    std::vector<cv::KeyPoint> features;
    cv::Mat descriptors;
    this->detector->detectAndCompute(image, {}, features, descriptors);

    std::vector<cv::Point2f> points;

    std::transform(features.cbegin(), features.cend(), std::back_inserter(points),
                   [](const auto& kp) { return kp.pt; });

    return {points, descriptors};
}

}