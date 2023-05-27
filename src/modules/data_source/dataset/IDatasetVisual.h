#pragma once

#include <string>
#include <tuple>
#include <istream>
#include <fstream>

#include <Eigen/Core>

#include "data_source/IDataSourceCamera.h"
#include "data_source/dataset/IDataset.h"

namespace eacham
{

template<typename T>
class IDatasetVisual : public IDataSourceCamera<T>, public IDataset
{
public:
    IDatasetVisual(const CameraType cameraType, const std::string& gtPosePath)
        : IDataSourceCamera<T>(cameraType)
        , IDataset(gtPosePath)
        , datasetUpdated(false)
    {
    }

public:
    T Get() const override
    {
        while (!this->dataUpdated)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }

        std::lock_guard<std::mutex> lock(this->dataMutex);
        this->dataUpdated = false;

        // prevent visualization unsync issue
        this->timestampOld = this->timestamp;
        this->groundTruthPosOld = this->groundTruthPos;
        this->datasetUpdated = true;

        return {this->timestamp, this->imageLeft.clone(), this->imageRight.clone()};
    }

    std::tuple<double, Eigen::Matrix4f> GetGtPose() const override
    {
        while (!this->dataUpdated)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }

        std::lock_guard<std::mutex> lock(this->dataMutex);
        
        return {this->timestampOld, this->groundTruthPosOld};
    }

protected:
    mutable double timestampOld;
    mutable Eigen::Matrix4f groundTruthPosOld;
    mutable bool datasetUpdated;
};

}