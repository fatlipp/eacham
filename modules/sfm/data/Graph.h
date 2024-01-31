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

    std::pair<unsigned, unsigned> GetBestPairForValid(const std::set<unsigned>& excluded = {})
    {
        float bestScore = 0;
        std::pair<unsigned, unsigned> bestPair{
            std::numeric_limits<unsigned>::max(),
            std::numeric_limits<unsigned>::max()
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
                if (excluded.count(id2) > 0 || bestScore > factor.quality)
                {
                    continue;
                }
                
                if (!Get(id2)->IsValid())
                {
                    bestScore = factor.quality;
                    bestPair = {id, id2};
                }
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

    void Print() const
    {
        // for (const auto& [id, node] : nodes)
        // {
        //     const auto* child = node->GetBestFactor();
        //     std::cout << "id: " << id << ", children: [";
        //     for (const auto& c : node->children)
        //         std::cout << c->id << ", ";
        //     std::cout << "], best: " << (child ? child->id : -1) << std::endl;
        // }
    }

public:
    void lock()
    {
        mutex.lock();
        // while (!mutex.try_lock()) 
        // {
        //     std::cout << "wait_for_lock()\n";
        //     std::this_thread::sleep_for(interval);
        // }
    }

    void unlock()
    {
        mutex.unlock();
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