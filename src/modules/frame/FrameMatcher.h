#pragma once

#include "tools/Tools3d.h"
#include <opencv2/calib3d.hpp>

#include <unordered_set>

namespace eacham
{
class FrameMatcher
{
public:
    FrameMatcher(const matcher_t& matcherInp)
        : mather(matcherInp)
    {}

public:
    std::vector<std::pair<unsigned, unsigned>> FindMatches(const IFrameLight& frame1, const IFrame& frame2)
    {
        const cv::Mat descriptor1 = GetDescriptors(frame1.GetPointsData());
        const cv::Mat descriptor2 = GetDescriptors(frame2.GetPointsData());

        std::vector<std::vector<cv::DMatch>> matches;
        mather->knnMatch(descriptor1, descriptor2, matches, 2);

        std::unordered_map<int, int>  matchesPairMap;
        std::unordered_set<int>  matchesPairMapSet;

        for (const auto& m : matches)
        {
            if (m[0].distance < 0.5f * m[1].distance)
            {
                if (matchesPairMapSet.contains(m[0].trainIdx))
                {
                    std::cout << "duplicate: " << m[0].trainIdx << std::endl;
                }
                else
                {
                    matchesPairMap.insert({m[0].queryIdx, m[0].trainIdx});
                    matchesPairMapSet.insert(m[0].trainIdx);
                }
            }
        }

        std::vector<std::pair<unsigned, unsigned>>  matchesPair; 
        for (const auto [key, value] : matchesPairMap)
        {
            matchesPair.push_back({key, value});
        }

        std::cout << "descriptor1: " << descriptor1.rows 
                 << ", descriptor2: " << descriptor2.rows 
                 << ", matches: " << matches.size()
                 << ", good: " << matchesPair.size()
                 << ", matchesPairMap: " << matchesPairMap.size()
                 << std::endl;

        return matchesPair;
    }

private:
    cv::Mat GetDescriptors(const std::vector<FramePointData>& pointsData)
    {   
        cv::Mat result = cv::Mat(pointsData.size(), pointsData[0].descriptor.cols, pointsData[0].descriptor.type());

        for (int i = 0; i < pointsData.size(); ++i)
        {
            pointsData[i].descriptor.copyTo(result.row(i));
        }

        return result;
    }

private:
    matcher_t mather;

};

}