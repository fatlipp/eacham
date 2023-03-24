#pragma once

#include <iostream>
#include <eigen3/Eigen/Core>

namespace eacham
{

template<typename T>
class IOdometry
{
public:
    ~IOdometry() = default;

    virtual bool Proceed(const T &data) = 0;
    
    virtual Eigen::Matrix4f GetOdometry() = 0;
};

} // namespace eacham
