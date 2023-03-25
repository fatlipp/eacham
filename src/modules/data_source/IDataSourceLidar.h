#pragma once

#include <string>
#include <tuple>
#include <opencv2/opencv.hpp>


namespace eacham
{

template<typename T>
class IDataSourceLidar
{
public:
    IDataSourceLidar() = default;

    virtual ~IDataSourceLidar() = default;

    virtual T Get() const = 0;
};

}