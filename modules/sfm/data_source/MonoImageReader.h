#pragma once

#include "base/data_source/ICamera.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <istream>
#include <fstream>
#include <utility>
#include <string>


namespace eacham::dataset
{
using namespace std::chrono_literals;

class MonoImageReader : public ICamera<double, cv::Mat, std::string>
{
public:
    MonoImageReader(const std::string& imagesPath)
    {
        cv::glob(imagesPath + "/*.jpg", imagePathes, false);
        if (imagePathes.size() == 0)
            cv::glob(imagesPath + "/*.JPG", imagePathes, false);
        if (imagePathes.size() == 0)
            cv::glob(imagesPath + "/*.png", imagePathes, false);
        if (imagePathes.size() == 0)
            cv::glob(imagesPath + "/*.PNG", imagePathes, false);

        currentId = 0;
    }

    MonoImageReader(MonoImageReader&& other)
    {
        currentId = std::exchange(other.currentId, 0);
        imagePathes = std::move(other.imagePathes);
    }

public:
    void Read() override
    {
        if (currentId == imagePathes.size())
        {
            return;
        }

        const auto im = cv::imread(imagePathes[currentId]);
        
        std::lock_guard<std::mutex> lock(this->dataMutex);
        data = { currentId, im, imagePathes[currentId] };
        
        ++currentId;
    }

    bool HasNext() const override
    {
        return currentId < imagePathes.size();
    }

private:
    unsigned currentId;
    std::vector<std::string> imagePathes;
};

}