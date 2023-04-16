#pragma once

#include "odometry/IFrameToMapOdometry.h"
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
class VisualOdometryF2MView : public IDrawable
{
public:
    VisualOdometryF2MView(IFrameToMapOdometry<T>* odom)
        : odometry(odom)
    {
    }

public:
    void Draw(pangolin::OpenGlRenderState& state) override
    {   
        const auto frames = odometry->GetLocalMap()->GetFrames();

        unsigned num = 0;

        Eigen::Matrix4f zeroPos = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f latestPos = Eigen::Matrix4f::Identity();
        view_tools::DrawCamera(zeroPos, Eigen::Vector3f{1, 1, 1});
        
        for (const auto& frame : frames)
        {
            const Eigen::Vector3f color = (num == frames.size() - 1) ? Eigen::Vector3f{1, 0, 0} : Eigen::Vector3f{1, 1, 0};
            const auto framePos = frame.GetPosition();

            view_tools::DrawCamera(framePos, color);

            if (num > 0)
            {
                glBegin(GL_LINES);
                glColor3f(color.x(), color.y(), color.z());
                glLineWidth(3);
                glVertex3f(framePos(0, 3), framePos(1, 3), framePos(2, 3));
                glVertex3f(latestPos(0, 3), latestPos(1, 3), latestPos(2, 3));
                glEnd();
            }

            glPointSize(7);
            glBegin(GL_POINTS);
            glColor3f(color.x(), color.y(), color.z());

            for (const auto& point : frame.GetPointsData())
            {   
                auto poss = tools::transformPoint3d(point.position3d, framePos);

                glColor3f(color.x(), color.y(), color.z());
                glVertex3f(poss.x, poss.y, poss.z);
            }

            glEnd();

            latestPos = framePos;
            ++num;
        }

        pangolin::OpenGlMatrix followPoint;
        followPoint.SetIdentity();
        followPoint.m[12] = latestPos(0, 3);
        followPoint.m[13] = latestPos(1, 3);
        followPoint.m[14] = latestPos(2, 3);

        state.Follow(followPoint);
    }

private:
    IFrameToMapOdometry<T>* const odometry;
    
};

}