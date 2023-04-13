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
class VisualOdometryView : public IDrawable
{
public:
    VisualOdometryView(IFrameToMapOdometry<T>* odom)
        : odometry(odom)
    {
    }

public:
    void Draw() override
    {   
        const auto frames = odometry->GetLocalMap()->GetFrames();

        Eigen::Matrix4f prevPos;

        unsigned num = 0;

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
                glVertex3f(prevPos(0, 3), prevPos(1, 3), prevPos(2, 3));
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

            prevPos = framePos;
            ++num;
        }
    }

private:
    IFrameToMapOdometry<T>* const odometry;
    
};

}