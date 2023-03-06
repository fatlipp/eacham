#pragma once

#include <vector>
#include <list>

#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

#include "IOdometry.h"
#include "tools/Tools3d.h"
#include "types/DataTypes.h"
#include "data_source/IDataSourceCamera.h"
#include "odometry/frame/FrameCreator.h"
#include "odometry/frame/KeyFrame.h"
#include "odometry/optimization/LocalFramesOptimizer.h"
#include "../visualization/Render.h"
#include "MotionEstimator.h"

namespace odometry
{
template<typename T>
class VisualOdometry : public IOdometry<T>
{
public:
    VisualOdometry(const data_source::IDataSourceCamera<T>* camera)
        : camera(camera)
    {
        this->motionEstimator = { camera->GetParameters(), camera->GetDistortion() };

        this->transform = Eigen::Affine3f::Identity();
    }

    void SetCameraMatrix()
    {
        
    }

    Eigen::Matrix4f GetOdometry(const T &data) ;

private:
    const data_source::IDataSourceCamera<T>* camera;

    std::list<Frame> frames;
    FrameCreator frameCreator;
    MotionEstimator motionEstimator;
    LocalFramesOptimizer localOptimizer;
    Eigen::Affine3f transform;
};

} // namespace odometry


namespace odometry
{

template<typename T>
Eigen::Matrix4f VisualOdometry<T>::GetOdometry(const T &data)
{
    std::cout << "--------------------------------------------------------------------" << std::endl;

    const Frame frame = frameCreator.Create(data, camera->GetParameters());

    if (frame.isValid())
    {
        Eigen::Matrix4f odom;

        if (frames.size() > 0)
        {
            const double dt = frame.timestamp - frames.back().timestamp;

            Eigen::Matrix4f odom = motionEstimator.Estimate(frames.back(), frame);

            this->transform.matrix() = this->transform.matrix() * odom;
        }

        frames.push_back(frame);

        if (frames.size() > 5)
        {
            frames.pop_front();

            // optimize
            const bool isOptimized = localOptimizer.Optimize(frames);

            if (isOptimized)
            {
                std::cout << "Optimized.." << std::endl;
            }

        }
    }

    return this->transform.matrix();
}

}