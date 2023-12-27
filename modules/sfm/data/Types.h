#pragma once

#include <opencv4/opencv2/core.hpp>
#include <set>

namespace eacham
{

template<typename FT, typename DT>
class Graph;

template<typename FT, typename DT>
class Node;

// using descriptor_tt = std::vector<std::array<float, 256>>;
using descriptor_tt = cv::Mat;
using graph_t = Graph<std::vector<cv::Point2f>, descriptor_tt>;
using node_t = Node<std::vector<cv::Point2f>, descriptor_tt>;

struct comp
{
    template<typename T>
    bool operator()(const T &l, const T &r) const
    {
        if (l.first == r.first) {
            return l.second > r.second;
        }
 
        return l.first < r.first;
    }
};

using match_t = std::set<std::pair<unsigned, unsigned>, comp>;

}