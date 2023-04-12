#include "visualization/Render.h"

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

#include "tools/Tools3d.h"


namespace eacham
{

void Render::Start()
{
    this->isRunning = true;
    this->drawThread = std::async(std::launch::async, &Render::Loop, this);
    // destructor will wait for get()
}

void Render::Loop()
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
        
        Draw();

        pangolin::FinishFrame();

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void Render::Stop()
{
    this->isRunning = false;
}

}