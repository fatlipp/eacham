#pragma once

#include "sfm/data/Node.h"

#include <opencv4/opencv2/core.hpp>

#include <map>
#include <unordered_map>
#include <iostream>
#include <thread>

namespace eacham
{

template<typename FT, typename DT>
class Graph
{
public:
    void Create(const unsigned id, const FT& keypoints, 
        const DT& descriptors, const cv::Mat& image)
    {
        std::lock_guard<std::mutex> lock(mutex);

        auto* node = Create(id);
        node->SetKeyPoints(keypoints);
        node->SetDescriptors(descriptors);
        node->SetImage(image);
    } 

    void Connect(const unsigned id1, const unsigned id2, match_t&& matches)
    {
        Connect(Get(id1), Get(id2), std::move(matches));
    }

    void Connect(Node<FT, DT>* node1, Node<FT, DT>* node2, match_t&& matches)
    {
        auto& factor = node1->AddFactor(node2);

        factor.matches = std::move(matches);
        factor.quality = matches.size();
    }

    Node<FT, DT>* Get(const unsigned id)
    {
        if (nodes.find(id) == nodes.end())
        {
            std::cout << "Node id is not found: " << id << std::endl;
            return nullptr;
        }

        return nodes[id];
    }

    const std::map<unsigned, Node<FT, DT>*>& GetNodes()
    {
        return nodes;
    }

    std::tuple<unsigned, unsigned, unsigned> 
        GetBestPairForValid(const std::set<unsigned>& excluded = {})
    {
        float bestScore = 0;
        std::tuple<unsigned, unsigned, unsigned> bestPair{
            std::numeric_limits<unsigned>::max(),
            std::numeric_limits<unsigned>::max(),
            0
        };

        for (auto [id, node] : nodes)
        {
            if (!node->IsValid())
            {
                continue;
            }

            const auto factors = node->GetFactors();

            for (auto [id2, factor] : factors)
            {
                if (Get(id2)->IsValid() || excluded.count(id2) > 0)
                {
                    continue;
                }

                unsigned points3dCount = 0;

                for (const auto& [m1, m2] : factor.matches)
                {
                    if (node->HasPoint3d(m1) && !node->IsPoint3dTwoView(m1))
                    {
                        ++points3dCount;
                    }
                }

                if (bestScore > points3dCount)
                {
                    continue;
                }
                
                bestScore = points3dCount;
                bestPair = {id, id2, points3dCount};
            }
        }

        return bestPair;
    }

    void FixNode(const unsigned id)
    {
        fixedNodes.insert(id);
    }

    bool IsFixed(const unsigned id)
    {
        return fixedNodes.count(id) > 0;
    }

public:
    size_t Size() const
    {
        return nodes.size();
    }

private:
    Node<FT, DT>* Create(const unsigned id)
    {
        if (nodes.find(id) == nodes.end())
        {
            nodes[id] = new Node<FT, DT>(id);
        }

        return nodes[id];
    }

private:
    std::map<unsigned, Node<FT, DT>*> nodes;
    std::set<unsigned> fixedNodes;

    std::mutex mutex;
};


}