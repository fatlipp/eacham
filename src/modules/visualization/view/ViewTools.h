#pragma once

#include <Eigen/Core>

#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>


namespace eacham::view_tools
{

static void DrawCamera(const Eigen::Matrix4f &Twc, const Eigen::Vector3f &color = Eigen::Vector3f(0, 0, 1.0f))
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

}