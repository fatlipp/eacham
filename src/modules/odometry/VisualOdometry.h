#pragma once

#include <vector>
#include <list>

#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

#include "IOdometry.h"
#include "tools/Tools3d.h"
#include "types/DataTypes.h"
#include "odometry/map/LocalMap.h"
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

        this->localMap = LocalMap(10U);
    }

    void SetCameraMatrix()
    {
        
    }

    Eigen::Matrix4f GetOdometry(const T &data) ;

private:
    const data_source::IDataSourceCamera<T>* camera;

    LocalMap localMap;
    Frame lastFrame;
    FrameCreator frameCreator;
    MotionEstimator motionEstimator;
    Eigen::Affine3f transform;

    LocalFramesOptimizer localOptimizer;
};

} // namespace odometry


namespace odometry
{

template<typename T>
Eigen::Matrix4f VisualOdometry<T>::GetOdometry(const T &data)
{
    std::cout << "--------------------------------------------------------------------" << std::endl;

    Frame frame = frameCreator.Create(data, camera->GetParameters());

    if (frame.isValid())
    {
        Eigen::Matrix4f odom;

        if (lastFrame.isValid())
        {
            const double dt = frame.GetTimestamp() - lastFrame.GetTimestamp();

            Eigen::Matrix4f odom = motionEstimator.Estimate(lastFrame, frame);
            frame.SetOdometry(odom);
            frame.SetPosition(lastFrame.GetPosition() * odom);

            this->transform.matrix() = this->transform.matrix() * odom;
        }
        else
        {
            frame.SetOdometry(Eigen::Matrix4f::Identity());
            frame.SetPosition(Eigen::Matrix4f::Identity());
        }

        localMap.AddFrame(frame);
        localOptimizer.Optimize(localMap);

        this->lastFrame = frame;
    }

    return this->transform.matrix();
}

}