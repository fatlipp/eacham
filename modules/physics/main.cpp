#include "cuda/CudaHelper.h"
#include "particles/CudaParticles.cuh"
#include "rigid/CudaRigid.cuh"

#include "render/OpenGlTools.h"
#include "render/Render.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#include <GL/freeglut.h>

#include <iostream>

// class A
// {
// public:
//     A() { std::cout << "ctr of A()\n";}
//     ~A() { std::cout << "~dtr of A()\n";}


//     A& A(const A&) { std::cout << "ctr of A()\n"; return *this;}

//     int a = 0;
// };

// void * operator new(size_t size)
// {
//     std::cout << "New operator overloading: " << size << std::endl;
//     void * p = malloc(size);
//     return p;
// }

// void func1(std::string&& a)
// {
//     std::cout << "func1: " << a << "\n";
// }

// void func2(const std::string& a)
// {
//     std::cout << "func2: " << a << "\n";
// }

// template<typename T>
// void func3(T&& a)
// {
//     std::cout << "func3: " << a << "\n";
// }

int main(int argc, char** argv)
{
    // std::vector<A> a;
    // std::cout << "---\n";
    // a.push_back({});
    // std::cout << "---\n";
    // a.push_back({});
    // std::cout << "---\n";
    // a.push_back({});
    // std::cout << "---\n";
    // a.push_back({});
    // std::cout << "---\n";
    // a.push_back({});
    // std::cout << "---\n";
    // return 0;

    if (!cuda::initCuda())
    {
        std::cout << "CUDA device is not found\n";
        return -1;
    }

    const uint width = 1280;
    const uint height = 720;
    const uint fov = 80;

    if (!gl::initGL(width, height, "CUDA tests"))
    {
        std::cout << "OpenGL is not initialized\n";
        return -1;
    }

    const uint DIMENSION = 3;

    auto vbo1 = gl::createVBO(4 * 10, DIMENSION);
    auto vbo2 = gl::createVBO(8 * 10, DIMENSION);

    Config config { };
    config.objects.push_back(ObjectConfig{
            .type = 0,
            .radius = 0.3,
            .vbo = vbo1,
            .count = 10
        });
    config.objects.push_back(ObjectConfig{
            .type = 1,
            .radius = 0.3,
            .vbo = vbo2,
            .count = 5
        });

    CudaRigid cudaCore;
    cudaCore.Initialize(config);

    GLuint program = gl::compileProgram();

    Render render { width, height, fov };
    render.SetCudaKernelCallback([&cudaCore](float dt){
            cudaCore.Call(0.5f);
        });

    render.SetDrawCallback([&program, &config]
        (const int width, const int height) {
            glColor3f(1.0, 1.0, 1.0);
            glutWireCube(4.0 * 2.0f);

            glEnable(GL_POINT_SPRITE_ARB);
            glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
            glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
            glDepthMask(GL_TRUE);
            glEnable(GL_DEPTH_TEST);

            glUseProgram(program);

            GLfloat colorMat[3] { 1, 0, 1};
            glUniform3fv(glGetUniformLocation(program, "color"), 1, colorMat);

            GLfloat mvMat[16]; 
            glGetFloatv(GL_MODELVIEW_MATRIX, mvMat); 
            glUniformMatrix4fv(glGetUniformLocation(program, "modelview"), 1, GL_FALSE, 
                mvMat);

            GLfloat pMat[16]; 
            glGetFloatv(GL_PROJECTION_MATRIX, pMat); 
            glUniformMatrix4fv(glGetUniformLocation(program, "projection"), 1, GL_FALSE, 
                pMat);

            glColor3f(1, 1, 1);
            glEnableVertexAttribArray(0);
            for (const auto& [type, radius, vbo, count] : config.objects)
            {
                glUniform1f(glGetUniformLocation(program, "pointScale"),
                            height / tanf(fov * 0.5f * 3.141592f / 180.0f));
                glUniform1f(glGetUniformLocation(program, "pointRadius"),
                            radius);

                glBindBuffer(GL_ARRAY_BUFFER, vbo);
                glVertexAttribPointer(0, DIMENSION, GL_FLOAT, GL_FALSE, 0, 0);
                glDrawArrays(GL_POINTS, 0, count * (type == 0 ? 4 : 8));
            }

            glUseProgram(0);
            glDisableVertexAttribArray(0);

            glEnableVertexAttribArray(0);
            glLineWidth(7);
            for (const auto& [type, radius, vbo, count] : config.objects)
            {
                glBindBuffer(GL_ARRAY_BUFFER, vbo);
                glVertexAttribPointer(0, DIMENSION, GL_FLOAT, GL_FALSE, 0, 0);
                
                const uint size = (type == 0 ? 4 : 8);

                for (int i = 0; i < count; ++i)
                {
                    glDrawArrays(GL_LINE_LOOP, i * size, size);
                }
            }
            glLineWidth(1);
            glDisableVertexAttribArray(0);
            
            glDisable(GL_POINT_SPRITE_ARB);
        });

    render.Start();

    return 0;
}