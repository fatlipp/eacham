#pragma once

#include <opencv2/opencv.hpp>

namespace eacham
{

template<typename... Ts>
class IDataSource
{
public:
using ReturnType = std::tuple<Ts...>;

public:
    virtual ~IDataSource() = default;

    virtual ReturnType Get() const = 0;
};

}