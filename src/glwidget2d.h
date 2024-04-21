#pragma once

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include <glwidget3d.h>

class GLWidget2D : public GLWidget3D
{
public:
    GLWidget2D();

private:
    void initializeGL()         override;
    void paintGL()              override;
    void mousePressEvent  (QMouseEvent *event) override;
    void mouseMoveEvent   (QMouseEvent *event) override;

    Cage2D  m_cage;
};

