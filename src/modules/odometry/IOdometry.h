#pragma once

#include <iostream>
#include <eigen3/Eigen/Core>

namespace eacham
{

// TODO: non thread-safe
template<typename T>
class IOdometry
{
public:
    IOdometry()
        : odometry(Eigen::Matrix4f::Identity())
        , position(Eigen::Matrix4f::Identity())
    {
    }
    
    virtual ~IOdometry() = default;

public:
    virtual bool Process(const T &data) = 0;

    virtual void Reset()
    {
        this->odometry = Eigen::Matrix4f::Identity();
        this->position = Eigen::Matrix4f::Identity();
    }
    
    virtual Eigen::Matrix4f GetOdometry() const
    {
        return this->odometry;
    }
    
    const Eigen::Matrix4f& GetPosition() const
    {
        return this->position;
    }

    void SetPosition(const Eigen::Matrix4f& pos)
    {
        this->position = pos;
    }

protected:
    Eigen::Matrix4f odometry;
    Eigen::Matrix4f position;
};

} // namespace eacham
