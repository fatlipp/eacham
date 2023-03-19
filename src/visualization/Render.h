#pragma once

#include <thread>
#include <future>

#include <eigen3/Eigen/Core>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/widgets.h>
#include <pangolin/display/default_font.h>
#include <pangolin/handler/handler.h>

#include "../modules/odometry/frame/Frame.h"

namespace eacham
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
        const int WIDTH = 1280;
        const int HEIGHT = 500;

        const int UI_WIDTH = 120;

        pangolin::CreateWindowAndBind("Main", WIDTH + UI_WIDTH, HEIGHT);
        glEnable(GL_DEPTH_TEST);
        glEnable (GL_BLEND);

        pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
        pangolin::Var<bool> buttonPlay("ui.Play", false, false);
        pangolin::Var<bool> buttonStep("ui.Step", false, false);
        pangolin::Var<bool> buttonClose("ui.Close", false, false);

        // Define Projection and initial ModelView matrix
        pangolin::OpenGlRenderState cameraState(
            pangolin::ProjectionMatrix(WIDTH, HEIGHT, 420, 420, static_cast<float>(WIDTH) / 2.0f, static_cast<float>(HEIGHT) / 2.0f ,0.01, 10000),
            pangolin::ModelViewLookAt(-2, 2, -2, 0,0,0, pangolin::AxisY)
        );

        // Create Interactive View in window
        pangolin::Handler3D handler(cameraState);
        pangolin::View& displayCam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -static_cast<float>(WIDTH) / static_cast<float>(HEIGHT))
                .SetHandler(&handler);

        pangolin::OpenGlMatrix Twc, Twr;
        Twc.SetIdentity();

        while ( this->isRunning && !pangolin::ShouldQuit() )
        {
            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            {
                if (pangolin::Pushed(buttonPlay) && onPlayClick != nullptr)
                {
                    onPlayClick();
                }
                if (pangolin::Pushed(buttonStep) && onStepClick != nullptr)
                {
                    onStepClick();
                }
                if (pangolin::Pushed(buttonClose) && onCloseClick != nullptr)
                {
                    onCloseClick();
                }
            }

            {
                displayCam.Activate(cameraState);

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
            }

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

                for (int i = 0; i < this->frames.size(); ++i)
                {
                    auto point = this->frames[i];

                    if (i == 0)
                    {
                        glColor3f(1.0, 1.0, 0.0);
                    }
                    else if (i == this->frames.size() - 1)
                    {
                        glColor3f(0.0, 1.0, 1.0);

                        glEnd();

                        glBegin(GL_LINES);
                        glLineWidth(3);
                        auto point2 = this->frames[i - 1];
                        glVertex3f(point.x(), point.y(), point.z());
                        glVertex3f(point2.x(), point2.y(), point2.z());
                        glEnd();

                        glPointSize(15);
                        glBegin(GL_POINTS);
                        glColor3f(0.0, 1.0, 1.0);
                    }
                    else
                    {
                        glEnd();

                        glBegin(GL_LINES);
                        glLineWidth(3);
                        auto point2 = this->frames[i - 1];
                        glVertex3f(point.x(), point.y(), point.z());
                        glVertex3f(point2.x(), point2.y(), point2.z());
                        glEnd();

                        glPointSize(15);
                        glBegin(GL_POINTS);
                        glColor3f(1.0, 0.0, 0.0);
                    }
                    glVertex3f(point.x(), point.y(), point.z());
                }
                glEnd();

                glPointSize(12);
                glBegin(GL_POINTS);
                glColor3f(0.0, 0.0, 1.0);
                
                for (const auto& point : this->framesGT)
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

    void AddFGTPoint(const Eigen::Matrix4f &pos)
    {
        std::lock_guard<std::mutex> lock(mute);

        this->framesGT.push_back({pos(0, 3), pos(1, 3), pos(2, 3)});
    }

    void AddPoint(const Eigen::Vector3f &pos)
    {
    }

    void ClearPoints()
    {
        std::lock_guard<std::mutex> lock(mute);
        this->points.clear();
    }

    void DrawMapPoints(const std::vector<MapPoint3d> &points)
    {
        std::lock_guard<std::mutex> lock(mute);

        for (auto &p : points)
        {
            this->points.push_back({p.position.x, p.position.y, p.position.z});
        }
    }

    void DrawMapFrames(const std::list<Frame> &framesInp)
    {
        std::lock_guard<std::mutex> lock(mute);

        frames.clear();

        for (auto &frame : framesInp)
        {
            const auto pos = frame.GetPosition();
            frames.push_back({pos(0, 3), pos(1, 3), pos(2, 3)});
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

    void SetOnPlayClick(std::function<void()> onPlayClick)
    {
        this->onPlayClick = onPlayClick;
    }

    void SetOnStepClick(std::function<void()> onStepClick)
    {
        this->onStepClick = onStepClick;
    }

    void SetOnCloseClick(std::function<void()> onCloseClick)
    {
        this->onCloseClick = onCloseClick;
    }

private:
    std::future<void> fut;
    std::atomic<bool> isRunning;
    std::mutex mute;

    Eigen::Matrix4f cameraPos;
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3f> frames;
    std::vector<Eigen::Vector3f> framesGT;

    std::function<void()> onPlayClick;
    std::function<void()> onStepClick;
    std::function<void()> onCloseClick;
};

}