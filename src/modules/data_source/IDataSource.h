#pragma once

#include <string>
#include <tuple>
#include <opencv2/opencv.hpp>


namespace eacham
{

template<typename T>
class IDataSource
{
public:
    IDataSource() = default;

    virtual ~IDataSource() = default;

    virtual T Get() const = 0;

    virtual void Clear() {}
};

}