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

int main(int argc, char** argv)
{
    if (!cuda::initCuda())
    {
        std::cout << "CUDA device is not found\n";
        return -1;
    }

    const uint width = 640;
    const uint height = 480;
    const uint fov = 60;

    if (!gl::initGL(width, height, "CUDA tests"))
    {
        std::cout << "OpenGL is not initialized\n";
        return -1;
    }

    ParticlesConfig config {
        .numParticles = 600,
        .particleRaius = 0.3,
        .dataSize = 6
    };

    GLuint program = gl::compileProgram();
    GLuint vbo = gl::createVBO(config.numParticles, config.dataSize);

    CudaParticles particles { config };
    particles.Initialize(vbo);

    Render render { width, height, fov };
    render.SetCudaKernelCallback([&particles](float dt){
            particles.Call(dt);
        });

    render.SetDrawCallback([&program, &vbo, &config]
        (const int width, const int height) {
            glEnable(GL_POINT_SPRITE_ARB);
            glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
            glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
            glDepthMask(GL_TRUE);
            glEnable(GL_DEPTH_TEST);

            glUseProgram(program);
            glUniform1f(glGetUniformLocation(program, "pointScale"),
                        height / tanf(fov * 0.5f * 3.141592f / 180.0f));
            glUniform1f(glGetUniformLocation(program, "pointRadius"),
                        config.particleRaius);
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
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * config.dataSize, 0);
            glDrawArrays(GL_POINTS, 0, config.numParticles);
            glDisableVertexAttribArray(0);
            glUseProgram(0);

            glDisable(GL_POINT_SPRITE_ARB);
        });

    render.Start();

    return 0;
}