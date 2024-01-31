#pragma once

#include "sfm/data/Types.h"
#include "sfm/data/Graph.h"

#include "visualization/render/IDrawable.h"
#include "visualization/view/ViewTools.h"

#include <pangolin/gl/gldraw.h>
#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

namespace eacham
{

class GraphView : public IDrawable
{
public:
    GraphView(const std::shared_ptr<graph_t>& graph)
    {
        this->graph = graph;
    }

public:
    void Draw(pangolin::OpenGlRenderState& state) override
    {
        Eigen::Matrix4d positionPrev = Eigen::Matrix4d::Identity();

        unsigned count = 0;

        std::set<unsigned> exludeIds;

        const Eigen::Vector3f color = Eigen::Vector3f{1, 0, 0};

        for (unsigned count = 0; const auto [id, node] : graph->GetNodes())
        {
            if (node->GetPoints3d().size() == 0)
            {
                continue;
            }

            Eigen::Matrix4d position = node->GetTransform().inverse();
            view_tools::DrawCamera(position.cast<float>(), 
                count > 0 ? Eigen::Vector3f{0, 1, 0} : Eigen::Vector3f{1, 0, 0});

            ++count;

            // if (count == 1)
            {
                // positionPrev = position;
                continue;
            }
            
            Eigen::Vector3f v1 = positionPrev.cast<float>().block<3, 1>(0, 3);
            Eigen::Vector3f v2 = position.cast<float>().block<3, 1>(0, 3);

            glLineWidth(5);
            glBegin(GL_LINES);
            glColor3f(1.0F, 0.0F, 0.0F);
            glVertex3f(v1.x(), v1.y(), v1.z());
            glColor3f(0.0F, 1.0F, 0.0F);
            glVertex3f(v2.x(), v2.y(), v2.z());
            glEnd();

            positionPrev = position;
        }
        
        // graph->unlock();
        // std::cout << "\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

public:
    void SetGraph(const std::shared_ptr<graph_t>& graph)
    {
        std::lock_guard<std::mutex> lock(mutex);
        this->graph = graph;
    }

private:
    std::mutex mutex;
    std::shared_ptr<graph_t> graph;
    
};

}