#pragma once

#include "odometry/IFrameToFrameOdometry.h"
#include "visualization/IDrawable.h"
#include "visualization/view/ViewTools.h"

#include <pangolin/gl/gldraw.h>
#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

namespace eacham
{

template<typename T>
class VisualOdometryF2FView  : public IDrawable
{
public:
    VisualOdometryF2FView (IFrameToFrameOdometry<T>* odom)
        : odometry(odom)
    {
    }

public:
    void Draw(pangolin::OpenGlRenderState& state) override
    {   
        Eigen::Matrix4f zeroPos = Eigen::Matrix4f::Identity();
        view_tools::DrawCamera(zeroPos, Eigen::Vector3f{1, 1, 1});

        const Eigen::Vector3f color = Eigen::Vector3f{1, 0, 0};

        const auto frame = odometry->GetLastFrame();
        const auto framePos = frame->GetPosition();

        // std::cout << "framePos:\n" << framePos << std::endl;

        view_tools::DrawCamera(framePos, color);

        glPointSize(7);
        glBegin(GL_POINTS);
        glColor3f(color.x(), color.y(), color.z());

        for (const auto& point : frame->GetPointsData())
        {   
            auto poss = tools::transformPoint3d(point.position3d, framePos);

            glColor3f(color.x(), color.y(), color.z());
            glVertex3f(poss.x, poss.y, poss.z);
        }

        glEnd();

        pangolin::OpenGlMatrix followPoint;
        followPoint.SetIdentity();
        followPoint.m[12] = framePos(0, 3);
        followPoint.m[13] = framePos(1, 3);
        followPoint.m[14] = framePos(2, 3);

        // state.Follow(followPoint);
    }

private:
    IFrameToFrameOdometry<T>* const odometry;
    
};

}