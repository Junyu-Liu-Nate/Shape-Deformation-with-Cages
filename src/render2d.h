#pragma once

// Defined before including GLEW to suppress deprecation messages on macOS
#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include "GL/glew.h"
#include <QOpenGLWidget>
#include <iostream>


class Render2D : public QOpenGLWidget
{
public:
    Render2D(QWidget *parent = nullptr);

protected:
    void initializeGL();
    void finish();

private:
    QImage m_image;

    GLuint m_shader; // shader program id
    GLuint m_vbo;    // VBO id
    GLuint m_vao;    // VAO id
    GLuint m_texture; // texture id

    static GLuint createShaderProgram(const char * vertex_file_path, const char * fragment_file_path);
    static GLuint createShader(GLenum shaderType, const char *filepath);

    void paintGL();
    void resizeGL(int w, int h);
};
