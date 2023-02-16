#pragma once

#include <iostream>
#include <eigen3/Eigen/Core>

namespace odometry
{

template<typename T>
class IOdometry
{
public:
    ~IOdometry() = default;
    
    virtual Eigen::Matrix4f GetOdometry(const T &data) = 0;
};

} // namespace odometry
