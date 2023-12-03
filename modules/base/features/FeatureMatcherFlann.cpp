#include "base/features/FeatureMatcherFlann.h"
#include <opencv2/calib3d.hpp>

#include <unordered_set>

namespace eacham
{
FeatureMatcherFlann::FeatureMatcherFlann()
{
    mather = cv::DescriptorMatcher::create("FlannBased");
}

FeatureMatcherFlann::MatchType FeatureMatcherFlann::Match(const cv::Mat& descriptor1, const cv::Mat& descriptor2)
{
    std::vector<std::vector<cv::DMatch>> matches;
    mather->knnMatch(descriptor1, descriptor2, matches, 2);

    MatchType  matchesPair;
    std::unordered_set<int>  matchesPairMapSet;

    for (const auto& m : matches)
    {
        if (m[0].distance / m[1].distance < 0.8)
        {
            if (!matchesPairMapSet.contains(m[0].trainIdx))
            {
                matchesPair.push_back({m[0].queryIdx, m[0].trainIdx});
                matchesPairMapSet.insert(m[0].trainIdx);
            }
        }
    }

    return matchesPair;
}

}