#pragma once

#include <opencv4/opencv2/core.hpp>

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
using match_t = std::vector<std::pair<unsigned, unsigned>>;

}