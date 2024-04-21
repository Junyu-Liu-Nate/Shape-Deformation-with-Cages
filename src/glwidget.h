#pragma once

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include "cage3d.h"
#include "cage2d.h"
#include "graphics/camera.h"
#include "graphics/shader.h"

#include <QOpenGLWidget>
#include <QElapsedTimer>
#include <QTimer>
#include <memory>

enum RenderMode {
    Render2D,
    Render3D
};

class GLWidget : public QOpenGLWidget
{
    Q_OBJECT

public:
    GLWidget(RenderMode mode, QWidget *parent = nullptr);
    ~GLWidget();

private:
    static const int FRAMES_TO_AVERAGE = 30;

    Eigen::Vector3f transformToWorldRay(int x, int y);

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

private:
    RenderMode m_mode;
//    Cage3D    m_arap;
    Cage2D  m_arap;
    Camera  m_camera;
    Shader *m_defaultShader;
    Shader *m_pointShader;

    float m_movementScaling;
    float m_vertexSelectionThreshold;
    float m_vSize;

    // Timing
    QElapsedTimer m_deltaTimeProvider; // For measuring elapsed time
    QTimer        m_intervalTimer;     // For triggering timed events

    // Movement
    int m_forward;
    int m_sideways;
    int m_vertical;

    // Mouse handler stuff
    int m_lastX;
    int m_lastY;
    bool m_leftCapture;
    bool m_rightCapture;
    SelectMode m_rightClickSelectMode;
    int m_lastSelectedVertex = -1;
};
