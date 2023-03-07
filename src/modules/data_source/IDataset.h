#pragma once

#include <string>
#include <tuple>
#include <eigen3/Eigen/Core>

namespace data_source
{

template<typename T>
class IDataset
{
public:
    IDataset() = default;

    virtual ~IDataset() = default;

    virtual void ReadNext() = 0;

    virtual Eigen::Matrix4f GetGtPose() const = 0;

private:
    const std::string sourcePath;
};

}