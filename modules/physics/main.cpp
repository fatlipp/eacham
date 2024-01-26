#include "config/ConfigParser.h"
#include "config/SceneConfig.h"
#include "cuda/CudaHelper.h"
#include "cuda/Operators.cuh"
#include "soft/CudaRigid.cuh"

#include "render/OpenGlTools.h"
#include "render/Render.h"
#include "render/RenderText.h"
#include "tools/BlockTimer.h"
#include "tools/Format.h"

#include <GLES3/gl3.h>
#include <GL/glu.h>
#include <GL/glext.h>
#include <GL/freeglut.h>
#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/ext/matrix_projection.hpp>
// glm::translate, glm::rotate, glm::scale, glm::perspective
#include <glm/gtx/io.hpp>

#include <thread>
#include <chrono>
#include <iostream>
#include <nlohmann/json.hpp>

int main(int argc, char** argv)
{
    if (!cuda::initCuda())
    {
        std::cout << "CUDA device is not found\n";
        return -1;
    }

    const uint width = 1280;
    const uint height = 720;
    const uint fov = 60;

    if (!gl::initGL(width, height, "CUDA tests"))
    {
        std::cout << "OpenGL is not initialized\n";
        return -1;
    }

    SceneConfig config = 
        config::Parse<SceneConfig>("/home/blackdyce/Projects/eacham/modules/physics/Config.json");

    auto getParticlesPerObject = [](const ObjectConfig& obj) {
        if (obj.type == ObjectType::Particle) return 1;
        if (obj.type == ObjectType::Cube2d2) return 4;
        if (obj.type == ObjectType::Cube2d3) return 8;
        if (obj.type == ObjectType::Cube3d2) return 8;
        return 0;
    };

    for (int i = 0; i < config.objects.size(); ++i)
    {
        auto& obj = config.objects[i];

        std::tie(obj.vboVertices, obj.vboColor) = 
            gl::createVBO(getParticlesPerObject(obj), obj.count);
    }

    CudaRigid cudaCore;
    cudaCore.Initialize(config);

    GLuint program = gl::compileProgram();

    constexpr float scale = std::tan(fov * 0.5f * 3.141592f / 180.0f);
    bool simulate = false;
    bool restart = false;
    bool drawCube = true;
    bool drawLines = false;
    bool drawGroundPlane = true;

    Render render { width, height, fov };
    RenderText renderText;
    BlockTimer kernelTimer;
    BlockTimer renderTimer;

    render.SetKeyboardCallback(
        [&config, &drawCube, &drawLines, &drawGroundPlane, &simulate, &restart,
         &cudaCore, &getParticlesPerObject]
        (const unsigned char key, const int mouseX, const int mouseY) {
            if (key == ' ') simulate = !simulate;
            if (key == 'r' || key == 'R') restart = true;
            if (key == 'c' || key == 'C') drawCube = !drawCube;
            if (key == 'l' || key == 'L') drawLines = !drawLines;
            if (key == 'g' || key == 'G') drawGroundPlane = !drawGroundPlane;
            if (key == 'e' || key == 'E') 
            {
                config.bound *= 2.0f;
                cudaCore.ExtendWalls(2.0f);
            }

            if ((key == 'f' || key == 'F') && config.command.type == "instantiate") 
            {
                auto obj = config.objects[config.command.index];

                std::tie(obj.vboVertices, obj.vboColor) = 
                    gl::createVBO(getParticlesPerObject(obj), obj.count);

                cudaCore.AddObject(obj, config.command.pos.x, config.command.pos.y);

                config.objects.push_back(std::move(obj));
            }  
        });
    render.SetCudaKernelCallback([&cudaCore, &config, &kernelTimer, &simulate, &restart](float dt) {
            kernelTimer.Start();
            if (restart)
            {
                cudaCore.Reset();
                restart = false;

                if (!simulate)
                {
                    cudaCore.Call(config.deltaTime);
                }
            }
            else if (simulate)
            {
                cudaCore.Call(config.deltaTime);
            }
            kernelTimer.Stop();
        });
    
    render.SetDrawCallback([&program, &config, &getParticlesPerObject,
         &drawLines, &drawCube, &drawGroundPlane, &renderTimer]
        (const int width, const int height) {
            renderTimer.Start();
            glEnable(GL_POINT_SPRITE_ARB);
            glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
            glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
            glDepthMask(GL_TRUE);
            glEnable(GL_DEPTH_TEST);

            glUseProgram(program);

            GLfloat mvMat[16]; 
            glGetFloatv(GL_MODELVIEW_MATRIX, mvMat); 
            glUniformMatrix4fv(glGetUniformLocation(program, "modelview"), 1, GL_FALSE, 
                mvMat);

            GLfloat pMat[16]; 
            glGetFloatv(GL_PROJECTION_MATRIX, pMat); 
            glUniformMatrix4fv(glGetUniformLocation(program, "projection"), 1, GL_FALSE, 
                pMat);

            glUniform1f(glGetUniformLocation(program, "pointScale"), height / scale);

            GLint col = glGetAttribLocation(program, "vertexColor");

            glEnableVertexAttribArray(0);
            glEnableVertexAttribArray(col);

            for (const auto& obj : config.objects)
            {
                glUniform1f(glGetUniformLocation(program, "pointRadius"),
                            obj.radius);

                // color buffer
                glBindBuffer(GL_ARRAY_BUFFER, obj.vboColor);
                glVertexAttribPointer(col, 3, GL_FLOAT, GL_FALSE, 0, 0);

                // posY buffer
                glBindBuffer(GL_ARRAY_BUFFER, obj.vboVertices);
                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
                glBindBuffer(GL_ARRAY_BUFFER, 0);

                // draw
                glDrawArrays(GL_POINTS, 0, obj.count * getParticlesPerObject(obj));
            }

            glDisableVertexAttribArray(0);
            glDisableVertexAttribArray(col);

            glUseProgram(0);
            
            // draw lines
            if (drawLines)
            {
                glEnableVertexAttribArray(0);
                glLineWidth(3);
                for (const auto& obj : config.objects)
                {
                    // glBindBuffer(GL_ARRAY_BUFFER, obj.vboVertices);
                    // glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

                    // // 3d
                    // if (obj.type == 2)
                    // {
                    //     for (int k = 0; k < obj.count; k += 1)
                    //     {
                    //         for (int i = k; i < k + 4; ++i)
                    //         {
                    //             glDrawArrays(GL_LINE_LOOP, i * 4, 4);
                    //         }
                    //         for (int i = k + 4; i < 8; ++i)
                    //         {
                    //             glDrawArrays(GL_LINE_LOOP, i * 4, 4);
                    //         }
                    //     }
                    //     continue;
                    // }

                    // // 2d
                    // const uint size = ((obj.type == 0 ? 4 : 8));
                    // for (int i = 0; i < obj.count; ++i)
                    // {
                    //     glDrawArrays(GL_LINE_LOOP, i * size, size);
                    // }
                }
                glLineWidth(1);
                glDisableVertexAttribArray(0);
            }

            glColor3f(1.0, 1.0, 1.0);
            glutWireCube(1.0f);

            glDepthMask(GL_FALSE);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            // draw a cube
            auto drawPlane = [](const auto& origin, const auto& side1, const auto& side2){
                glBegin(GL_QUADS);
                glVertex3f(origin.x, origin.y, origin.z);
                glVertex3f(origin.x + side1.x, origin.y+side1.y, origin.z+side1.z);
                glVertex3f(origin.x + side1.x + side2.x, origin.y+side1.y+side2.y, origin.z+side1.z+side2.z);
                glVertex3f(origin.x + side2.x, origin.y+side2.y, origin.z+side2.z);
                glEnd();
            };
            if (drawCube)
            {
                float alpha = 0.4f;
                // front
                // {
                //     const float3 origin {
                //         config.bound.min.x, config.bound.min.y, config.bound.max.z};
                //     const float3 side1{0, config.bound.max.y - config.bound.min.y, 0};
                //     const float3 side2{config.bound.max.x - config.bound.min.x, 0, 0};
                    
                //     glColor4f(0.7, 0.7, 0.3, alpha);
                //     drawPlane(origin, side1, side2);
                // }
                // back
                {
                    const float3 origin {
                        config.bound.min.x, config.bound.min.y, config.bound.min.z};
                    const float3 side1{0, config.bound.max.y - config.bound.min.y, 0};
                    const float3 side2{config.bound.max.x - config.bound.min.x, 0, 0};
                    
                    glColor4f(0.7, 0.5, 0.3, alpha);
                    drawPlane(origin, side1, side2);
                }
                // left
                {
                    const float3 origin {
                        config.bound.min.x, config.bound.min.y, config.bound.max.z};
                    const float3 side1{0, config.bound.max.y - config.bound.min.y, 0};
                    const float3 side2{0, 0, config.bound.min.z - config.bound.max.z};
                    
                    glColor4f(0.2, 0.7, 0.7, alpha);
                    drawPlane(origin, side1, side2);
                }

                // right
                {
                    const float3 origin {
                        config.bound.max.x, config.bound.min.y, config.bound.max.z};
                    const float3 side1{0, config.bound.max.y - config.bound.min.y, 0};
                    const float3 side2{0, 0, config.bound.min.z - config.bound.max.z};
                    
                    glColor4f(0.2, 0.7, 0.7, alpha);
                    drawPlane(origin, side1, side2);
                }
            }
            // draw a ground plane
            if (drawGroundPlane)
            {
                const float3 origin {
                    config.bound.min.x, config.bound.min.y, config.bound.min.z};
                const float3 side1{0, 0, config.bound.max.z - config.bound.min.z};
                const float3 side2{config.bound.max.x - config.bound.min.x, 0, 0};

                glColor4f(0.3, 0.7, 0.3, 0.8f);
                drawPlane(origin, side1, side2);
            }
            glColor3f(1, 1, 1);
            glDisable(GL_BLEND);
            glDepthMask(GL_TRUE);
            glDisable(GL_POINT_SPRITE_ARB);

            renderTimer.Stop();
        });

    render.SetPostRenderCallback([&kernelTimer, &renderTimer, &renderText, &cudaCore]() {
            int posY = 20;
            renderText.AddText(20, posY, "Stats:", {1, 1, 1});
            posY += 20;
            renderText.AddText(20, posY, format("Kernel: {} ms", kernelTimer.Get()), {0, 1, 0});
            posY += 20;
            renderText.AddText(20, posY, format("Render: {} ms", renderTimer.Get()), 
                {0, 1, 0});
            posY += 20;

            const float time = kernelTimer.Get() + renderTimer.Get();
            const int fps = time > 0.0001f ? 1000.0f / time : 1000.0f;
            renderText.AddText(20, posY, format("FPS: {}", fps), 
                {0, 1, 0});
            posY += 30;

            renderText.AddText(20, posY, format("Particles: {}", cudaCore.GetParticlesCount()), 
                {1, 1, 0});
            posY += 20;
            renderText.AddText(20, posY, format("Springs: {}", cudaCore.GetSpringsCount()), 
                {1, 1, 0});

            posY += 50;
            renderText.AddText(20, posY, "Control:", {1, 1, 1});
            posY += 30;
            renderText.AddText(20, posY, " 'R' - start/restart simulation", {1, 1, 1});
            posY += 20;
            renderText.AddText(20, posY, " 'SPACE' - pause simulation", {1, 1, 1});
            posY += 30;
            renderText.AddText(20, posY, " 'G' - show/hide ground", {1, 1, 1});
            posY += 20;
            renderText.AddText(20, posY, " 'L' - show/hide lines", {1, 1, 1});
            posY += 20;
            renderText.AddText(20, posY, " 'C' - show/hide bounds", {1, 1, 1});
            posY += 20;
            renderText.AddText(20, posY, " 'E' - extend bounds", {1, 1, 1});
            posY += 20;
            renderText.AddText(20, posY, " 'F' - call command", {1, 1, 1});
            posY += 20;

            renderText.Draw(width, height);
            renderText.Clear();
        });
    render.Start();

    return 0;
}