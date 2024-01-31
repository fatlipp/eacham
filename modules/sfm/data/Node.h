#pragma once

#include <opencv4/opencv2/core.hpp>
#include <Eigen/Core>

#include <map>
#include <set>
#include <unordered_map>
#include <iostream>
#include <thread>

namespace eacham
{

template<typename FT, typename DT>
class Graph;

struct MatchTwoView
{
    Eigen::Matrix4d transform;
    std::vector<std::tuple<unsigned, unsigned, Eigen::Vector3d>> matches;
};

struct Factor
{
    unsigned id; // connected node id
    // bool isValid;
    float quality;
    std::unordered_map<unsigned, unsigned> matches;
    Eigen::Matrix4d transform;
};

template<typename FT, typename DT>
class Node
{
public:
    Node(const unsigned id)
        : id(id)
    {
    }

    void SetKeyPoints(const FT& keypoints)
    {
        this->keyPoints = keypoints;
    }

    void SetDescriptors(const DT& descriptors)
    {
        this->descriptors = descriptors;
    }

    void SetImage(const cv::Mat& image)
    {
        this->image = image.clone();
    }

    void SetTransform(const Eigen::Matrix4d& transform)
    {
        this->transform = transform;
    }

    void SetBestFactor(const unsigned id)
    {
        this->bestFactorId = id;
    }

    void SetPoint3d(const unsigned id2d, const unsigned id3d)
    {
        this->points3d[id2d] = id3d;
    }

    bool HasPoint3d(const unsigned id2d)
    {
        return points3d.count(id2d) > 0;
    }

    unsigned GetPoint3d(const unsigned id2d)
    {
        if (points3d.count(id2d) == 0)
        {
            throw std::runtime_error(
                cv::format("Node::GetPoint3d() point %i is not found", id2d));
        }

        return this->points3d[id2d];
    }

    const std::unordered_map<unsigned, unsigned>& GetPoints3d()
    {
        return this->points3d;
    }

    const auto& GetKeyPoint(const unsigned id)
    {
        if (id >= keyPoints.size())
        {
            throw std::runtime_error(
                cv::format("Node::GetKeyPoint() point %i is not found", id));
        }

        return this->keyPoints[id];
    }

    Eigen::Vector2d GetKeyPointEigen(const unsigned id)
    {
        if (id >= keyPoints.size())
        {
            throw std::runtime_error(
                cv::format("Node::GetKeyPointEigen() point %i is not found", id));
        }
        return {this->keyPoints[id].x, this->keyPoints[id].y};
    }

    const Eigen::Matrix4d& GetTransform() const
    {
        return transform;
    } 

    const FT& GetFeatures()
    {
        return this->keyPoints;
    }

    const DT& GetDescriptors()
    {
        return this->descriptors;
    }

    const cv::Mat& GetImage()
    {
        return this->image;
    }

    bool IsValid() const
    {
        return isValid;
    }

    void SetValid(const bool value)
    {
        isValid = value;
    }

public:
    Factor& AddFactor(const Node* node)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (factors.count(node->id) == 0)
        {
            factors.insert({node->id, {node->id}});
        }

        return factors[node->id];
    }

    bool HasFactor(const unsigned id) const
    {
        return factors.count(id) != 0;
    }

    Factor& GetFactor(const unsigned id)
    {
        if (factors.count(id) == 0)
        {
            throw std::runtime_error(
                cv::format("Node::GetFactor() factor %i is not found", id));
        }

        return factors[id];
    }

    unsigned GetBestFactor(const std::set<unsigned>& exludedIds)
    {
        float bestQuality = 0;
        unsigned bestQualityId = 0;

        for (const auto& [id1, factor] : factors)
        {
            if (exludedIds.count(id1) == 0 && factor.quality > bestQuality)
            {
                bestQuality = factor.quality;
                bestQualityId = id1;
            }
        }

        if (bestQuality > 0)
            return bestQualityId;

        return std::numeric_limits<unsigned>::max();
    }

    const std::unordered_map<unsigned, Factor>& GetFactors()
    {
        return factors;
    }

public:
    unsigned GetId() const
    {
        return id;
    }

private:
    unsigned id;
    unsigned bestFactorId = 999999;
    bool isValid = false;
    
    FT keyPoints;
    DT descriptors;
    cv::Mat image;
    std::unordered_map<unsigned, Factor> factors;
    std::unordered_map<unsigned, unsigned> points3d;
    Eigen::Matrix4d transform;

    std::mutex mutex;
    
    friend class Graph<FT, DT>;
};

}