#pragma once

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include "synccage3d.h"
#include "graphics/camera.h"
#include "graphics/shader.h"

#include <QOpenGLWidget>
#include <QElapsedTimer>
#include <QTimer>
#include <memory>

class StaticGLWidget3D : public QOpenGLWidget
{
    Q_OBJECT

public:
    StaticGLWidget3D(QWidget *parent = nullptr, SyncCage3D *syncCage = nullptr);
    ~StaticGLWidget3D();

    void init();
    void setObjectFilePath(const QString &path);
    void setCageFilePath(const QString &path);

protected:
    Eigen::Vector3f transformToWorldRay(int x, int y);

private:
    static const int FRAMES_TO_AVERAGE = 30;

    // Basic OpenGL Overrides
    void initializeGL()         override;
    void paintGL()              override;
    void resizeGL(int w, int h) override;

    // Event Listeners
    void mousePressEvent  (QMouseEvent *event) override;
    void mouseMoveEvent   (QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent       (QWheelEvent *event) override;
    void keyPressEvent    (QKeyEvent   *event) override;
    void keyReleaseEvent  (QKeyEvent   *event) override;

private slots:
    // Physics Tick
    void tick();

protected:
    SyncCage3D *m_syncCage;;

    static Camera  m_camera;
    Shader *m_defaultShader;
    Shader *m_pointShader;

    static float m_movementScaling;
    static float m_vertexSelectionThreshold;
    static float m_vSize;

    // Timing
    QElapsedTimer m_deltaTimeProvider; // For measuring elapsed time
    QTimer        m_intervalTimer;     // For triggering timed events

    // Mouse handler stuff
    static int m_lastX;
    static int m_lastY;
    static bool m_leftCapture;
    static bool m_rightCapture;
    static bool m_shiftFlag;

    // Execution flow control
    static bool m_initialized;

private:
    // Movement
    static int m_forward;
    static int m_sideways;
    static int m_vertical;

    // Mouse handler stuff
    static int m_lastSelectedVertex;
    static SelectMode m_rightClickSelectMode;
};
