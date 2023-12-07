#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <istream>
#include <fstream>
#include <utility>
#include <string>

#include "core/data_source/ICamera.h"
#include "core/data_source/IDataset.h"
#include "core/data_source/IGroundTruth.h"

namespace eacham::dataset
{
using namespace std::chrono_literals;

struct DatasetInputData
{
    std::string calibPath;
    std::string images;
    unsigned startFrameId;
    std::string gt;

};

class MonoCameraDataset : public ICamera<double, cv::Mat, std::string>
{
public:
    MonoCameraDataset(const std::string& calibPath, const std::string& imagesPath, const unsigned startFrameId)
        : calibPath {calibPath}
    {
        cv::glob(imagesPath + "/*.jpg", imagePatches, false);
        if (imagePatches.size() == 0)
            cv::glob(imagesPath + "/*.JPG", imagePatches, false);
        if (imagePatches.size() == 0)
            cv::glob(imagesPath + "/*.png", imagePatches, false);
        if (imagePatches.size() == 0)
            cv::glob(imagesPath + "/*.PNG", imagePatches, false);
        currentId = startFrameId;
    }

public:
    void Read() override
    {
        if (currentId == imagePatches.size())
        {
            return;
        }

        std::cout << "Process Mono: " << currentId << "\n";

        auto im = cv::imread(imagePatches[currentId]);
        
        std::lock_guard<std::mutex> lock(this->dataMutex);
        data = { currentId, im, imagePatches[currentId] };
        
        currentId += 1;
    }

    bool HasNext() const override
    {
        return currentId < imagePatches.size();
    }

private:
    const std::string calibPath;
    const std::string imagePath;

    std::vector<std::string> imagePatches;
    unsigned currentId;
};

class MonoCameraGt : public IGroundTruth<Eigen::Matrix4d>
{
public:
    MonoCameraGt(const std::string& gtPath)
    {
    }

public:
    typename MonoCameraGt::ReturnType Get() const override
    {
        return Eigen::Matrix4d::Identity();
    }
};

using CameraT = MonoCameraDataset;
using GtT = MonoCameraGt;

class SfmInputSource : public IDataset<CameraT, GtT>
{
public:
    SfmInputSource(const DatasetInputData& data)
        : IDataset(std::make_unique<CameraT>(data.calibPath, data.images, data.startFrameId), std::make_unique<GtT>(data.gt)) 
        {}
};

}