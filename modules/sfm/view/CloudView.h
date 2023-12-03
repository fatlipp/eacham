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

class CloudView : public IDrawable
{
public:
    CloudView(const Eigen::Matrix4d& position, const std::vector<Eigen::Vector3d>& cloud, const Eigen::Vector3f& color)
    {
        std::lock_guard<std::mutex> lock(mutex);
        this->position = position;
        this->cloud = cloud;
        this->color = color;
    }

public:
    void Draw(pangolin::OpenGlRenderState& state) override
    {
        // std::lock_guard<std::mutex> lock(mutex);

        // view_tools::DrawCamera(position.cast<float>(), color);
        // // view_tools::DrawCloud(position.cast<float>(), cloud, color);

        // std::vector<Eigen::Vector3d> cloud2;
        // for (const auto& p : cloud)
        // {
        //     const auto pos = tools::transformPoint3d(p, position);
        //     cloud2.push_back(pos);
        // }
        // view_tools::DrawCloud(Eigen::Matrix4f::Identity(), cloud2, color);
    }

private:
    Eigen::Matrix4d position;
    std::vector<Eigen::Vector3d> cloud;
    Eigen::Vector3f color;

    std::mutex mutex;
    
};

}