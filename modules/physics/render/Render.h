#pragma once

#include <functional>

enum class ButtonState
{
    NONE,
    ROTATE,
    SCALE,
    MOVE
};

class Render
{
public:
    Render(const uint width, const uint height, const uint fov)
        : width { width } 
        , height { height } 
        , fov { fov } 
        , deltaTime { 0.1 }
        , buttonState(ButtonState::NONE)
    {}

public:
    void SetKeyboardCallback(std::function<void(const unsigned char, 
        const int, const int)>&& cb)
    {
        keyboardCallback = std::move(cb);
    }

    void SetCudaKernelCallback(std::function<void(float)>&& cb)
    {
        cudaKernelCallback = std::move(cb);
    }

    void SetDrawCallback(std::function<void(const int, const int)>&& cb)
    {
        drawCallback = std::move(cb);
    }

    void SetPostRenderCallback(std::function<void()>&& cb)
    {
        postRenderCallback = std::move(cb);
    }

    void SetMouseClickCallback(
        std::function<void(const int, const int, const int, const int)>&& cb)
    {
        mouseClickCallback = std::move(cb);
    }

public:
    void Start();
    void OnIdle();
    void OnDisplay();
    void OnReshape(int w, int h);
    void OnMouse(int button, int state, int x, int y);
    void OnMouseMotion(int x, int y);

private:
    uint width;
    uint height;
    uint fov;
    float deltaTime;
    std::function<void(float)> cudaKernelCallback;
    std::function<void(const int, const int)> drawCallback;
    std::function<void(const unsigned char, const int, const int)> keyboardCallback;
    std::function<void()> postRenderCallback;
    std::function<void(const int, const int, const int, const int)> mouseClickCallback;

    // camera control
    int mouseX;
    int mouseY;
    ButtonState buttonState;
    std::array<float, 6> cameraTrans = {0, 0, 0, 0, 0, 0};
    std::array<float, 6> cameraTransDelta = {0, 0, -75, 0, 0, 0};
    const float inertia = 1.0f;

};