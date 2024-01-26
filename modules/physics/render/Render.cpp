#include "render/Render.h"

#include <iostream>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>

#define TO_LAMBDA(M) [this]{M();}
#define TO_LAMBDA_ARGS_2(M) [this](int x, int y){M(x, y);}
#define TO_LAMBDA_ARGS_3(M) [this](unsigned char key, int x, int y){M(key, x, y);}
#define TO_LAMBDA_ARGS_4(M) [this](int button, int state, int x, int y){M(button, state, x, y);}


void Render::Start()
{
  auto l_to_ptr = [](auto lambda) {
        static auto lambda_copy = lambda;

        return []<typename... Args>(Args... args) {
            return lambda_copy(args...);
        };
    };

  glutIdleFunc(l_to_ptr(TO_LAMBDA(OnIdle)));
  glutDisplayFunc(l_to_ptr(TO_LAMBDA(OnDisplay)));
  glutReshapeFunc(l_to_ptr(TO_LAMBDA_ARGS_2(OnReshape)));

  glutMouseFunc(l_to_ptr(TO_LAMBDA_ARGS_4(OnMouse)));
  glutMotionFunc(l_to_ptr(TO_LAMBDA_ARGS_2(OnMouseMotion)));
  glutKeyboardFunc(l_to_ptr(keyboardCallback));

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
    cameraTrans[c] += (cameraTransDelta[c] - cameraTrans[c]) * inertia;
    cameraTrans[c + 3] += (cameraTransDelta[c + 3] - cameraTrans[c + 3]) * inertia;
  }

  glTranslatef(cameraTrans[0], cameraTrans[1], cameraTrans[2]);
  glRotatef(cameraTrans[3], 1.0, 0.0, 0.0);
  glRotatef(cameraTrans[4], 0.0, 1.0, 0.0);

  if (drawCallback != nullptr)
    drawCallback(width, height);

  if (postRenderCallback != nullptr)
    postRenderCallback();

  glutSwapBuffers();
  glutReportErrors();
}

void Render::OnReshape(int w, int h) 
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (float)w / (float)h, 0.1, 5000.0);

    glMatrixMode(GL_MODELVIEW);
    glViewport(0, 0, w, h);

    width = w;
    height = h;
}

void Render::OnMouse(int button, int state, int x, int y) 
{
  if (state == GLUT_UP) 
  {
    buttonState = ButtonState::ROTATE;
  }
  else
  {
    const int modifiers = glutGetModifiers();

    if (modifiers & GLUT_ACTIVE_SHIFT || button == 1)
    {
      buttonState = ButtonState::MOVE;
    } 
    else if (modifiers & GLUT_ACTIVE_CTRL || button == 2) 
    {
      buttonState = ButtonState::SCALE;
    }
    else if (button == 0)
    {
      buttonState = ButtonState::ROTATE;
    }
    else
    {
      buttonState = ButtonState::NONE;
    }

    if (mouseClickCallback != nullptr)
      mouseClickCallback(x, y, width, height);
  }

  mouseX = x;
  mouseY = y;

  glutPostRedisplay();
}

void Render::OnMouseMotion(int x, int y) 
{
    const float dx = (float)(x - mouseX);
    const float dy = (float)(y - mouseY);

    switch (buttonState)
    {
    case ButtonState::ROTATE:
      cameraTransDelta[3] += dy / 5.0f;
      cameraTransDelta[4] += dx / 5.0f;
      break;
    case ButtonState::SCALE:
      cameraTransDelta[2] += (dy / 200.0f) * std::abs(cameraTransDelta[2]);
      break;
    case ButtonState::MOVE:
      float speed = 1000.0f / std::abs(cameraTrans[2]);
      cameraTransDelta[0] += dx / speed;
      cameraTransDelta[1] -= dy / speed;
      break;
    
    }

    mouseX = x;
    mouseY = y;

    glutPostRedisplay();
}