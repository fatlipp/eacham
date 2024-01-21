#include "render/Render.h"

#include "helpers/Tools.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>


#define TO_LAMBDA(M) [this]{M();}
#define TO_LAMBDA_ARGS(M) [this](int x, int y){M(x, y);}
#define TO_LAMBDA_ARGS_4(M) [this](int button, int state, int x, int y){M(button, state, x, y);}

void Render::Start()
{
    glutIdleFunc(lambda_to_pointer(TO_LAMBDA(OnIdle)));
    glutDisplayFunc(lambda_to_pointer(TO_LAMBDA(OnDisplay)));
    glutReshapeFunc(lambda_to_pointer(TO_LAMBDA_ARGS(OnReshape)));

    glutMouseFunc(lambda_to_pointer(TO_LAMBDA_ARGS_4(OnMouse)));
    glutMotionFunc(lambda_to_pointer(TO_LAMBDA_ARGS(OnMouseMotion)));

    glutMainLoop();
}

void Render::OnIdle()
{
    glutPostRedisplay();
}

void Render::OnDisplay() 
{
    if (cudaKernelCallback != nullptr)
    {
      cudaKernelCallback(deltaTime);
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    for (int c = 0; c < 3; ++c) 
    {
        cameraPosDelta[c] += (cameraPos[c] - cameraPosDelta[c]) * inertia;
        cameraRotDelta[c] += (cameraRot[c] - cameraRotDelta[c]) * inertia;
    }

    glTranslatef(cameraPosDelta[0], cameraPosDelta[1], cameraPosDelta[2]);
    glRotatef(cameraRotDelta[0], 1.0, 0.0, 0.0);
    glRotatef(cameraRotDelta[1], 0.0, 1.0, 0.0);

    if (drawCallback != nullptr)
      drawCallback(width, height);

    glutSwapBuffers();
    glutReportErrors();
}

void Render::OnReshape(int w, int h) 
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (float)w / (float)h, 0.1, 500.0);

    glMatrixMode(GL_MODELVIEW);
    glViewport(0, 0, w, h);

    width = w;
    height = h;
}

void Render::OnMouse(int button, int state, int x, int y) 
{
  if (state == GLUT_DOWN) 
  {
    buttonState |= 1 << button;
  } 
  else if (state == GLUT_UP) 
  {
    buttonState = 0;
  }

  int mods = glutGetModifiers();

  if (mods & GLUT_ACTIVE_SHIFT) 
  {
    buttonState = 2;
  } 
  else if (mods & GLUT_ACTIVE_CTRL) 
  {
    buttonState = 3;
  }

  ox = x;
  oy = y;

  glutPostRedisplay();
}

void Render::OnMouseMotion(int x, int y) 
{
    float dx = (float)(x - ox);
    float dy = (float)(y - oy);

    if (buttonState == 3) 
    {
        // left+middle = zoom
        cameraPos[2] += (dy / 100.0f) * 0.5f * std::abs(cameraPos[2]);
    } 
    else if (buttonState & 2) 
    {
        // middle = translate
        cameraPos[0] += dx / 100.0f;
        cameraPos[1] -= dy / 100.0f;
    } 
    else if (buttonState & 1) 
    {
        // left = rotate
        cameraRot[0] += dy / 5.0f;
        cameraRot[1] += dx / 5.0f;
    }

    ox = x;
    oy = y;

    glutPostRedisplay();
}