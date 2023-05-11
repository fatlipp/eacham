#pragma once

#include "map/Map.h"
#include "visualization/IDrawable.h"
#include "visualization/view/ViewTools.h"

#include <pangolin/gl/gldraw.h>
#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

namespace eacham
{

class MapView : public IDrawable
{
public:
    MapView(Map* map)
    {
        this->map = map;
    }

public:
    void Draw(pangolin::OpenGlRenderState& state) override
    {
        Eigen::Matrix4f zeroPos = Eigen::Matrix4f::Identity();
        view_tools::DrawCamera(zeroPos, Eigen::Vector3f{1, 1, 1});

        const Eigen::Vector3f color = Eigen::Vector3f{1, 0, 0};

        for (const auto& frame : map->GetFrames())
        {
            view_tools::DrawCamera(frame.position, color);
        }

        glPointSize(10);
        glBegin(GL_POINTS);
        glColor3f(color.x(), color.y(), color.z());
        for (const auto& point : map->GetPoints())
        {   
            auto poss = point.position;

            if (point.observers > 1)
            {
                glColor3f(0, 1, 0);
                glVertex3f(poss.x, poss.y, poss.z);
            }
        }
        glEnd();
    }

private:
    Map* map;
    std::mutex mapMutex;
    
};

}