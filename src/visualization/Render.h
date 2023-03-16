#pragma once

#include <thread>
#include <future>

#include <eigen3/Eigen/Core>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>
#include <pangolin/gl/gldraw.h>

#include "../modules/odometry/frame/Frame.h"

namespace render
{
class Render
{
public:
    Render()
    {
        this->isRunning = true;
        this->fut = std::async(std::launch::async, &Render::Start, this);
        // destructor will wait for get()
    }

    void Start()
    {
        pangolin::CreateWindowAndBind("Main",640,480);
        glEnable(GL_DEPTH_TEST);

        // Define Projection and initial ModelView matrix
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1280, 720, 420, 420, 320, 240 ,0.01, 10000),
            pangolin::ModelViewLookAt(-2, 2, -2, 0,0,0, pangolin::AxisY)
        );

        // Create Interactive View in window
        pangolin::Handler3D handler(s_cam);
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -1280.0f/720.0f)
                .SetHandler(&handler);

        pangolin::OpenGlMatrix Twc, Twr;
        Twc.SetIdentity();

        while ( this->isRunning && !pangolin::ShouldQuit() )
        {
            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);

            const float &w = 1;
            const float h = w*0.75;
            const float z = w*0.6;

            glPushMatrix();
            glMultMatrixd(Twc.m);

            glLineWidth(1);
            glColor3f(0.0f,1.0f,0.0f);

            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);
            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);
            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);
            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();

            {
                std::lock_guard<std::mutex> lock(mute);

                glPointSize(5);
                glBegin(GL_POINTS);
                glColor3f(0.0, 1.0, 0.0);
                for (const auto& point : this->points)
                {
                    glVertex3f(point.x(), point.y(), point.z());
                }
                glEnd();

                glPointSize(15);
                glBegin(GL_POINTS);
                glColor3f(1.0, 0.0, 0.0);
                
                for (const auto& point : this->frames)
                {
                    glVertex3f(point.x(), point.y(), point.z());
                }
                glEnd();

                // Swap frames and Process Events
                pangolin::FinishFrame();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void SetCameraPosition(const Eigen::Matrix4f &pos)
    {
        std::lock_guard<std::mutex> lock(mute);
        this->cameraPos = pos;

    }

    void AddFramePoint(const Eigen::Matrix4f &pos)
    {
        std::lock_guard<std::mutex> lock(mute);

        this->frames.push_back({pos(0, 3), pos(1, 3), pos(2, 3)});
    }

    void AddPoint(const Eigen::Vector3f &pos)
    {
    }

    void ClearPoints()
    {
        std::lock_guard<std::mutex> lock(mute);
        this->points.clear();
    }

    void DrawMap(const std::vector<odometry::FramePoint3d> &points)
    {
        std::lock_guard<std::mutex> lock(mute);

        for (auto &p : points)
        {
            this->points.push_back({p.position.x, p.position.y, p.position.z});
        }
    }

    void Stop()
    {
        this->isRunning = false;

        if (fut.valid())
        {
            fut.get();
        }
    }
private:
    std::future<void> fut;
    std::atomic<bool> isRunning;
    std::mutex mute;

    Eigen::Matrix4f cameraPos;
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3f> frames;
};

}