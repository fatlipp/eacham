#pragma once

#include "visualization/render/IDrawable.h"
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
    void Draw(pangolin::OpenGlRenderState& state) override
    {
        // std::lock_guard<std::mutex> lock(positionMutex);

        // for (unsigned i = 0; const auto& cam : cameras)
        // {
        //     const Eigen::Vector3f color = (i == 0) ? Eigen::Vector3f{1, 0, 0} : Eigen::Vector3f{0, 1, 0};

        //     view_tools::DrawCamera(cam.cast<float>(), color);
        //     view_tools::DrawCloud(Eigen::Matrix4f::Identity(), clouds[i], color);

        //     if (i > 0)
        //     {
        //         Eigen::Vector3f v1 = cameras[i - 1].cast<float>().block<3, 1>(0, 3);
        //         Eigen::Vector3f v2 = cam.cast<float>().block<3, 1>(0, 3);
                
        //         glLineWidth(5);
        //         glBegin(GL_LINES);
        //         glColor3f(1.0F, 0.0F, 0.0F);
        //         glVertex3f(v1.x(), v1.y(), v1.z());
        //         glColor3f(0.0F, 1.0F, 0.0F);
        //         glVertex3f(v2.x(), v2.y(), v2.z());
        //         glEnd();
        //     }

        //     ++i;
        // }
    }

public:
    void AddCameraPosition(const Eigen::Matrix4d& position)
    {
        std::lock_guard<std::mutex> lock(positionMutex);
        this->cameras.push_back(position);
    }
    void AddFrameCloud(const std::vector<Eigen::Vector3d>& cloud)
    {
        std::lock_guard<std::mutex> lock(positionMutex);
        this->clouds.push_back(cloud);
    }

private:
    std::vector<Eigen::Matrix4d> cameras;
    std::vector<std::vector<Eigen::Vector3d>> clouds;
    std::mutex positionMutex;
    
};

}