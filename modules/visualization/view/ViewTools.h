#pragma once

#include <Eigen/Core>

#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

namespace eacham::view_tools
{

static void DrawCamera(const Eigen::Matrix4f &transform, const Eigen::Vector3f &color = Eigen::Vector3f(0, 0, 1.0f))
{
    const float w = 0.1f;
    const float h = 0.1f;
    const float z = 0.1f;

    glPushMatrix();

    glMultMatrixf((GLfloat*)transform.data());

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
}

static void DrawCloud(const Eigen::Matrix4f &transform, 
    const std::vector<Eigen::Vector3d>& cloud, 
    const unsigned pointSize,
    const Eigen::Vector3f &color = Eigen::Vector3f(0, 0, 1.0f))
{
    glPushMatrix();
    glMultMatrixf((GLfloat*)transform.data());

    glColor3f(color.x(), color.y(), color.z());

    glPointSize(pointSize);
    glBegin(GL_POINTS);
    for (const auto point : cloud)
    {
        glVertex3f(point.x(), point.y(), point.z());
    }
    glEnd();

    glPopMatrix();
}

static void DrawCloud(const Eigen::Matrix4f &transform, 
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& cloud,
    const unsigned pointSize)
{
    glPushMatrix();
    glMultMatrixf((GLfloat*)transform.data());

    for (const auto [point, color] : cloud)
    {
        glColor3f(color.x(), color.y(), color.z());
        glPointSize(pointSize);
        glBegin(GL_POINTS);
            glVertex3f(point.x(), point.y(), point.z());
        glEnd();
    }

    glEnd();

    glPopMatrix();
}

}