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

public:
    virtual bool Process(const T &data) = 0;
    
    virtual Eigen::Matrix4f GetOdometry() const
    {
        return this->odometry;
    }

    const Eigen::Matrix4f& GetPosition() const
    {
        return this->position;
    }

protected:
    Eigen::Matrix4f odometry;
    Eigen::Matrix4f position;

};

} // namespace eacham
