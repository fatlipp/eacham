#pragma once

#include <string>
#include <tuple>
#include <opencv4/opencv2/opencv.hpp>


namespace data_source
{

template<typename T>
class IDataSource
{
public:
    using ReturnType = T;

public:
    IDataSource() = default;

    virtual ~IDataSource() = default;

    virtual T GetNext() const = 0;

private:
    const std::string sourcePath;
};

}