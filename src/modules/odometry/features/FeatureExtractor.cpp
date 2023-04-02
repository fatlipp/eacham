#include "FeatureExtractor.h"


namespace eacham
{
FeatureExtractor::FeatureExtractor(const FeatureExtractorType &typeInp)
        : type(typeInp)
{
    const int winSize = 41;
    switch (type)
    {
    case FeatureExtractorType::ORB:
            // int  	nfeatures = 500,
            // float  	scaleFactor = 1.2f,
            // int  	nlevels = 8,
            // int  	edgeThreshold = 31, ignored border range
            // int  	firstLevel = 0,
            // int  	WTA_K = 2, - points to compare
            // int  	scoreType = ORB::HARRIS_SCORE,
            // int  	patchSize = 31, - windows size to BRIEF
            // int  	fastThreshold = 20  - if abs pixel difference larger than this value - potentially corner
            // decrease to get more features
        this->detector = cv::ORB::create(1500, 1.2f, 6, winSize, 0, 2, cv::ORB::HARRIS_SCORE, winSize, 15);
        break;
    case FeatureExtractorType::SIFT:
    // (int nfeatures=0, int nOctaveLayers=3, double contrastThreshold=0.04, double edgeThreshold=10, double sigma=1.6)
        this->detector = cv::SIFT::create(1500, 3, 0.04, 10, 1.6);
        break;
    case FeatureExtractorType::SURF:
        this->detector = cv::xfeatures2d::SURF::create(1500);
        break;
    }
}

std::tuple<std::vector<cv::KeyPoint>, cv::Mat> FeatureExtractor::GetFeatures(const cv::Mat &image)
{
    std::vector<cv::KeyPoint> features;
    cv::Mat descriptors;

    this->detector->detectAndCompute(image, {}, features, descriptors);

    return { features, descriptors };
}

}