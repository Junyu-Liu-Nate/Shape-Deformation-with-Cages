#pragma once

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include <staticglwidget3d.h>
#include "synccage2d.h"

class StaticGLWidget2D : public StaticGLWidget3D
{
public:
    StaticGLWidget2D(SyncCage2D *m_syncCage);

    void init();
    void setTextureFilePath(const QString &path);
    void setCageFilePath(const QString &path);

private:
    void initializeGL()         override;
    void paintGL()              override;
    void mousePressEvent  (QMouseEvent *event) override;
    void mouseMoveEvent   (QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;

    SyncCage2D *m_syncCage2d;

    static SelectMode m_rightClickSelectModeOnCage;
    static SelectMode m_rightClickSelectModeOnCtrlPt;

    static int m_lastSelectedVertexOnCage;
    static int m_lastSelectedVertexOnCtrlPt;
};
