#pragma once

#include <functional>

class Render
{
public:
    Render(const uint width, const uint height, const uint fov)
        : width { width } 
        , height { height } 
        , fov { fov } 
        , deltaTime { 0.1 } 
    {}

public:
    void SetCudaKernelCallback(std::function<void(float)>&& cb)
    {
        cudaKernelCallback = std::move(cb);
    }

    void SetDrawCallback(std::function<void(const int, const int)>&& cb)
    {
        drawCallback = std::move(cb);
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

    int ox;
    int oy;
    int buttonState = 0;

    float cameraPos[3] = {0, 0, -7};
    float cameraRot[3] = {0, 0, 0};
    float cameraPosDelta[3] = {0, 0, 0};
    float cameraRotDelta[3] = {0, 0, 0};
    const float inertia = 1.1f;

};