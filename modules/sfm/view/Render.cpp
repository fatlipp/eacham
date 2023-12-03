#include "sfm/view/Render.h"
#include "base/tools/Tools3d.h"

#include "sfm/view/MapView.h"

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
#include <pangolin/gl/glplatform.h>
#include <pangolin/gl/glfont.h>

namespace eacham
{

void Render::Activate()
{
    this->isRunning = true;
    this->drawThread = std::async(std::launch::async, &Render::Loop, this);
}

void Render::Loop()
{
    const int WIDTH = 1280;
    const int HEIGHT = 800;

    const int UI_WIDTH = 180;

    pangolin::CreateWindowAndBind("Main", WIDTH + UI_WIDTH, HEIGHT);
    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND);

    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
    pangolin::Var<bool> buttonPlay("ui.Play", false, false);
    pangolin::Var<bool> buttonStep("ui.Step", false, false);
    pangolin::Var<bool> buttonBa("ui.BA", false, false);
    pangolin::Var<bool> buttonReset("ui.Reset", false, false);
    pangolin::Var<bool> buttonClose("ui.Close", false, false);

    int minObserversCount = 3;
    pangolin::Var<int> minObserversText("ui. Min Observers", minObserversCount);
    pangolin::Var<bool> buttonDecMinObservers("ui.-", false, false);
    pangolin::Var<bool> buttonAddMinObservers("ui.+", false, false);

    int pointSize = 3;
    pangolin::Var<int> pointSizeText("ui. Point Size", pointSize);
    pangolin::Var<bool> buttonDecPointSize("ui.point_size.-", false, false);
    pangolin::Var<bool> buttonAddPointSize("ui.point_size.+", false, false);

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
            if (pangolin::Pushed(buttonBa) && onBAClick != nullptr)
            {
                onBAClick();
            }
            if (pangolin::Pushed(buttonReset) && onResetClick != nullptr)
            {
                onResetClick();
            }
            if (pangolin::Pushed(buttonClose) && onCloseClick != nullptr)
            {
                onCloseClick();
            }

            if (pangolin::Pushed(buttonDecMinObservers) && minObserversCount > 0)
            {
                --minObserversCount;

                minObserversText = minObserversCount;
            }
            if (pangolin::Pushed(buttonAddMinObservers) && minObserversCount < 10)
            {
                ++minObserversCount;

                minObserversText = minObserversCount;
            }

            if (pangolin::Pushed(buttonDecPointSize) && pointSize > 0)
            {
                --pointSize;

                pointSizeText = pointSize;
            }
            if (pangolin::Pushed(buttonAddPointSize) && pointSize < 10)
            {
                ++pointSize;

                pointSizeText = pointSize;
            }
        }

        displayCam.Activate(cameraState);
        glScalef(-1.f, -1.f, 1.f);
        
        Draw(cameraState);

        for (auto& d : drawables)
        {
            if (dynamic_cast<MapView*>(d.get()))
            {
                dynamic_cast<MapView*>(d.get())->SetMinObservers(minObserversCount);
                dynamic_cast<MapView*>(d.get())->SetPointSize(pointSize);
            }
        }

        pangolin::FinishFrame();

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void Render::Stop()
{
    this->isRunning = false;
}

}