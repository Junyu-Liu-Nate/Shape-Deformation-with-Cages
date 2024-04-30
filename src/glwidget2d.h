#pragma once

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include <glwidget3d.h>
#include "cage2d.h"

class GLWidget2D : public GLWidget3D
{
public:
    GLWidget2D();

    void init();
    void setTextureFilePath(const QString &path);
    void setCageFilePath(const QString &path);

private:
    void initializeGL()         override;
    void paintGL()              override;
    void mousePressEvent  (QMouseEvent *event) override;
    void mouseMoveEvent   (QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

    Cage2D  m_cage;
    SelectMode m_rightClickSelectModeOnCage;
    SelectMode m_rightClickSelectModeOnCtrlPt;

    int m_lastSelectedVertexOnCage = -1;
    int m_lastSelectedVertexOnCtrlPt = -1;
};

