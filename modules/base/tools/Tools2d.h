#pragma once

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <iostream>

namespace eacham
{

static cv::Mat ConcatImages(const cv::Mat& image1, const cv::Mat& image2)
{
    cv::Mat output;
    cv::hconcat(image1, image2, output);

    return output;
}

static void ResizeImage(cv::Mat& image, const float scale)
{
    const float size = image.cols * image.rows * scale;

    cv::resize(image, image, cv::Size(), scale, scale);
}

template<typename T>
static std::vector<T> NormalizeKeypoints(const std::vector<T>& points, 
    const cv::Size& size)
{
    std::vector<T> result;

    std::cout << "w: " << size.width << ", h: " << size.height << std::endl;

    const auto centerX = size.width * 0.5f;
    const auto centerY = size.height * 0.5f;
    const auto scale = std::max(size.width, size.height) * 0.5f;

    for (const auto& p : points)
    {
        const auto x = (p.x - centerX) / scale;
        const auto y = (p.y - centerY) / scale;
        result.push_back(T{x, y});
    }

    return result;
}

}