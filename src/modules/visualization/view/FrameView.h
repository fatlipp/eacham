#pragma once

#include "odometry/FrameToFrameOdometry.h"
#include "visualization/IDrawable.h"
#include "visualization/view/ViewTools.h"

#include <pangolin/gl/gldraw.h>
#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

namespace eacham
{

class FrameView : public IDrawable
{
public:
    void SetPosition(const Eigen::Matrix4f& position)
    {
        std::lock_guard<std::mutex> lock(positionMutex);
        this->position = position;
    }

public:
    void Draw(pangolin::OpenGlRenderState& state) override
    {
        Eigen::Matrix4f zeroPos = Eigen::Matrix4f::Identity();
        view_tools::DrawCamera(zeroPos, Eigen::Vector3f{1, 1, 1});

        const Eigen::Vector3f color = Eigen::Vector3f{1, 0, 0};

        std::lock_guard<std::mutex> lock(positionMutex);
        view_tools::DrawCamera(position, color);

        // glPointSize(7);
        // glBegin(GL_POINTS);
        // glColor3f(color.x(), color.y(), color.z());
        // for (const auto& point : frame.GetPointsData())
        // {   
        //     auto poss = tools::transformPoint3d(point.position3d, framePos);
        //     glColor3f(color.x(), color.y(), color.z());
        //     glVertex3f(poss.x, poss.y, poss.z);
        // }
        // glEnd();

        // pangolin::OpenGlMatrix followPoint;
        // followPoint.SetIdentity();
        // followPoint.m[12] = framePos(0, 3);
        // followPoint.m[13] = framePos(1, 3);
        // followPoint.m[14] = framePos(2, 3);

        // state.Follow(followPoint);
    }

private:
    Eigen::Matrix4f position;
    std::mutex positionMutex;
    
};

}