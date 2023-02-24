#pragma once

#include <vector>
#include <list>

#include "IOdometry.h"
#include "types/DataTypes.h"
#include "data_source/IDataSourceCamera.h"
#include "odometry/frame/FrameCreator.h"
#include "odometry/frame/KeyFrame.h"
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
    }

    void SetCameraMatrix()
    {
        
    }

    Eigen::Matrix4f GetOdometry(const T &data, render::Render &renderer) ;

private:
    const data_source::IDataSourceCamera<T>* camera;


    std::list<Frame> frames;
    FrameCreator frameCreator;
    MotionEstimator motionEstimator;
};

} // namespace odometry


namespace odometry
{

template<typename T>
Eigen::Matrix4f VisualOdometry<T>::GetOdometry(const T &data, render::Render &renderer)
{
    renderer.ClearPoints();

    const Frame frame = frameCreator.Create(data, camera->GetParameters());

    if (frame.isValid())
    {
        Eigen::Matrix4f odom;

        if (frames.size() > 0)
        {
            odom = motionEstimator.Estimate(frame, frames.back());
        }

        frames.push_back(frame);

        if (frames.size() > 10)
        {
            frames.pop_front();
        }

        // vis
        {
            cv::Mat_<float> worldToCam(3, 3);
            worldToCam <<   -1, 0, 0,
                            0, -1, 0,
                            0, 0, 1;

            for (const auto& p : frame.GetPoints3d())
            {
                cv::Mat_<float> src(3/*rows*/,1 /* cols */); 
                src(0,0)=p.x; 
                src(1,0)=p.y; 
                src(2,0)=p.z; 

                const cv::Mat mm = worldToCam * src;
                renderer.AddPoint({mm.at<float>(0, 0), mm.at<float>(1, 0), mm.at<float>(2, 0)});
            }
        }

        return odom;
    }

    return Eigen::Matrix4f::Identity();
}

}