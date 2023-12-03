#pragma once

#include "sfm/data/Map.h"

#include "visualization/render/IDrawable.h"
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
    MapView(const std::shared_ptr<Map>& map)
    {
        this->map = map;
        this->minObservers = 2;
        this->pointSize = 2;
    }

public:
    void Draw(pangolin::OpenGlRenderState& state) override
    {
        map->lock();

        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> pts3d;

        for (const auto& [id, mapPointData] : map->GetAll())
        {
            // if (mapPointData.isValid)
            if (mapPointData.isValid && mapPointData.observers.size() >= minObservers)
                pts3d.push_back({mapPointData.point3d, mapPointData.color});
        }

        view_tools::DrawCloud(Eigen::Matrix4f::Identity(), pts3d, pointSize);
        
        map->unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

public:
    void SetGraph(const std::shared_ptr<Map>& map)
    {
        std::lock_guard<std::mutex> lock(mutex);
        this->map = map;
    }

    void SetMinObservers(const int value)
    {
        minObservers = value;
    }

    void SetPointSize(const int value)
    {
        pointSize = value;
    }

private:
    std::mutex mutex;
    std::shared_ptr<Map> map;

    int minObservers;
    int pointSize;
    
};

}