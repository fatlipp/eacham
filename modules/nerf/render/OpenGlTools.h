#pragma once

#include <iostream>

#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/freeglut.h>

namespace gl
{

bool initGL(const int width, const int height, const std::string& name) 
{
    int argc = 1;
    char *argv[1] = {(char*)""};

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(width, height);
    glutCreateWindow(name.c_str());

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.925, 0.25, 0.25, 1.0);

    glutReportErrors();

    return true;
}

uint createVBO(const uint count, const uint itemSize)
{
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * count * itemSize, 0, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    return vbo;
}

GLuint compileProgram() 
{
    #define STRINGIFY(A) \
        #A

     std::string vertexShaderSource = "#version 330\n";
     vertexShaderSource = vertexShaderSource + STRINGIFY(
        
        layout(location = 0) in vec3 aPos;
        uniform mat4 modelview;
        uniform mat4 projection;

        uniform float pointRadius;
        uniform float pointScale;

        void main()
        {
            vec3 posEye = vec3(modelview * vec4(aPos, 1.0));
            float dist = length(posEye);

            gl_PointSize = pointRadius * (pointScale / dist);
            gl_Position = projection * modelview * vec4(aPos, 1.0);
        });

    // pixel shader for rendering points as shaded spheres
    const char *spherePixelShaderSource = STRINGIFY(
        uniform vec3 color;
        void main()
        {
            const vec3 lightDir = vec3(0.577, 0.577, 0.577);

            // calculate normal from texture coordinates
            vec3 N;
            N.xy = gl_TexCoord[0].xy*vec2(2.0, -2.0) + vec2(-1.0, 1.0);
            float mag = dot(N.xy, N.xy);

            if (mag > 1.0) discard;   // kill pixels outside circle

            N.z = sqrt(1.0-mag);

            float diffuse = max(0.0, dot(lightDir, N));

            gl_FragColor = vec4(color, 1.0) * diffuse;
        });

    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    
    const char *ff = vertexShaderSource.c_str();

    glShaderSource(vertexShader, 1, &ff, 0);
    glShaderSource(fragmentShader, 1, &spherePixelShaderSource, 0);

    glCompileShader(vertexShader);
    glCompileShader(fragmentShader);

    GLuint program = glCreateProgram();

    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);

    glLinkProgram(program);

    // check if program linked
    GLint success = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &success);

    if (!success) {
        char temp[256];
        glGetProgramInfoLog(program, 256, 0, temp);
        printf("Failed to link program:\n%s\n", temp);
        glDeleteProgram(program);
        program = 0;

        throw "Shader compilation error";
    }
    else
    {
        std::cout << "shaders are OK\n";
    }
    #define STRINGIFY(A) #A

    glClampColorARB(GL_CLAMP_VERTEX_COLOR_ARB, GL_FALSE);
    glClampColorARB(GL_CLAMP_FRAGMENT_COLOR_ARB, GL_FALSE);

    return program;
}

}