#pragma once

#include "sfm/data/Frame.h"
#include "sfm/data/Types.h"
#include "sfm/data/Graph.h"

#include <execution>
#include <functional>

namespace eacham
{
// struct Pair
// {
//     unsigned id1;
//     unsigned id2;
// };

void Match(const std::vector<Frame>& frames, graph_t graph, matcher)
{
    std::vector<std::pair> pairs;
    for (int i = 0; i < frames.size(); ++i)
    {
        for (int j = i + 1; j < frames.size(); ++j)
        {
            pairs.push_back({i, j});
            pairs.push_back({j, i});
        }
    }

    std::vector<std::unordered_map<unsigned, unsigned>> matchedPairs;
    std::mutex mutex;

    std::for_each(std::execution::par_unseq, 
            pairs.begin(), pairs.end(),
            [&graph, &matcher, &node1](const auto& pair) {
        auto& [frame1, frame2] = pair;
        auto node1 = graph->Get(frame1.id);
        auto node2 = graph->Get(frame2.id);

        auto t12 = std::async(std::launch::async, &FeatureMatcherFlann::Match, &matcher, 
            node1->GetDescriptors(), node2->GetDescriptors());
        auto matches12 = t12.get();

        if (matches12.size() < 30)
        {
            return;
        }

        graph->Connect(node1, node2, bestMatches12);
    });
}

}