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
#include "../modules/tools/Tools3d.h"

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

    bool IsActive()
    {
        return this->isRunning;
    }

    void DrawCamera(const Eigen::Matrix4f &Twc, const Eigen::Vector3f &color = Eigen::Vector3f(0, 0, 1.0f))
    {
        const float w = 0.2f;
        const float h = 0.1f;
        const float z = 0.1f;

        glPushMatrix();

        glMultMatrixf((GLfloat*)Twc.data());

        glLineWidth(3);
        glColor3f(color.x(), color.y(), color.z());
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w,-h,z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w,-h,z);
        glVertex3f(0, 0, 0);
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

        glPointSize(10);
        glBegin(GL_POINTS);
        glVertex3f(0, 0, 0);
        glEnd();

        glPopMatrix();

        glEnd();
    }

    void Start()
    {
        const int WIDTH = 1280;
        const int HEIGHT = 800;

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
            pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY)
        );

        // Create Interactive View in window
        pangolin::Handler3D handler(cameraState);
        pangolin::View& displayCam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -static_cast<float>(WIDTH) / static_cast<float>(HEIGHT))
                .SetHandler(&handler);

        pangolin::OpenGlMatrix Twc, Twr;
        Twc.SetIdentity();

        while (this->isRunning && !pangolin::ShouldQuit())
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

            displayCam.Activate(cameraState);

            glScalef(-1.f, -1.f, 1.f);

            // point: [0, 0, 0]
            DrawCamera(Eigen::Matrix4f::Identity());

            {
                std::lock_guard<std::mutex> lock(mute);

                for (int i = 0; i < this->frames.size(); ++i)
                {
                    const Eigen::Vector3f color = (i == this->frames.size() - 1) ? Eigen::Vector3f{1, 0, 0} : Eigen::Vector3f{1, 1, 0};
                    const auto framePos = this->frames[i].GetPosition();

                    DrawCamera(framePos, color);

                    if (i > 0)
                    {
                        const auto framePosPrev = this->frames[i - 1].GetPosition();

                        glBegin(GL_LINES);
                        glColor3f(color.x(), color.y(), color.z());
                        glLineWidth(3);
                        glVertex3f(framePos(0, 3), framePos(1, 3), framePos(2, 3));
                        glVertex3f(framePosPrev(0, 3), framePosPrev(1, 3), framePosPrev(2, 3));
                        glEnd();
                    }

                    glPointSize(7);
                    glBegin(GL_POINTS);
                    glColor3f(color.x(), color.y(), color.z());
                    int cc = 0;
                    for (const auto& point : this->frames[i].GetPointsData())
                    {   
                        // auto poss = point.position3d;//transformPoint3d(point.position3d, framePos);
                        auto poss = transformPoint3d(point.position3d, framePos);

                        // if (i == 0)
                        //     poss.z -= 0.7f;

                        // if (point.isInlier && (cc % 2 == 0))
                        //     glVertex3f(poss.x, poss.y, poss.z);
                        // if (!point.isInlier && (cc % 2 != 0))
                        //     glVertex3f(poss.x, poss.y, poss.z);
                        if (point.isInlier)
                            glColor3f(color.x(), color.y(), color.z());
                        else
                            glColor3f(color.x() * 0.5f, color.y() * 0.5f, color.z() * 0.5f);
                        glVertex3f(poss.x, poss.y, poss.z);

                        cc++;
                    }
                    glEnd();
                }

                for (int i = 0; i < this->framesGT.size(); ++i)
                {
                    const Eigen::Vector3f color = {0, 1, 0};
                    const auto framePos = this->framesGT[i];

                    DrawCamera(framePos, color);

                    if (i > 0)
                    {
                        const auto framePosPrev = this->framesGT[i - 1];

                        glBegin(GL_LINES);
                        glColor3f(color.x(), color.y(), color.z());
                        glLineWidth(3);
                        glVertex3f(framePos(0, 3), framePos(1, 3), framePos(2, 3));
                        glVertex3f(framePosPrev(0, 3), framePosPrev(1, 3), framePosPrev(2, 3));
                        glEnd();
                    }
                }

                glPointSize(5);
                glBegin(GL_POINTS);
                glColor3f(0.0, 1.0, 0.0);
                for (const auto& point : this->points)
                {
                    glVertex3f(point.x(), point.y(), point.z());
                }
                glEnd();

                glPointSize(5);
                glBegin(GL_POINTS);
                glColor3f(0.0, 1.0, 1.0);
                for (int i = 0; i < this->lidarData.size(); ++i)
                {
                    const auto point = this->lidarData[i];
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

    void AddFGTPoint(const Eigen::Matrix4f &pos)
    {
        std::lock_guard<std::mutex> lock(mute);

        this->framesGT.push_back(pos);
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
            frames.push_back(frame);
            // frames.push_back(frame.GetPosition());
        }
    }

    void DrawLidarData(const lidardata_t &lidarData)
    {
        std::lock_guard<std::mutex> lock(mute);
        this->lidarData = lidarData;
    }

    void DrawFrame(const Frame &frame)
    {
    }

    void Stop()
    {
        this->isRunning = false;

        // if (fut.valid())
        // {
        //     fut.get();
        // }
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
    std::vector<Frame> frames;
    std::vector<Eigen::Matrix4f> framesGT;
    lidardata_t lidarData;

    std::function<void()> onPlayClick;
    std::function<void()> onStepClick;
    std::function<void()> onCloseClick;
};

}