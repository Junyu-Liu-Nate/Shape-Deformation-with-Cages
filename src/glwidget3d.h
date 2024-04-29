#pragma once

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include "cage3d.h"
//#include "cage2d.h"
#include "graphics/camera.h"
#include "graphics/shader.h"

#include <QOpenGLWidget>
#include <QElapsedTimer>
#include <QTimer>
#include <memory>

class GLWidget3D : public QOpenGLWidget
{
    Q_OBJECT

public:
    GLWidget3D(QWidget *parent = nullptr);
    ~GLWidget3D();

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
    Camera  m_camera;
    Shader *m_defaultShader;
    Shader *m_pointShader;

    float m_movementScaling;
    float m_vertexSelectionThreshold;
    float m_vSize;

    // Timing
    QElapsedTimer m_deltaTimeProvider; // For measuring elapsed time
    QTimer        m_intervalTimer;     // For triggering timed events

    // Mouse handler stuff
    int m_lastX;
    int m_lastY;
    bool m_leftCapture;
    bool m_rightCapture;
    SelectMode m_rightClickSelectMode;
    int m_lastSelectedVertex = -1;
    bool m_shiftFlag = false;

    // Execution flow control
    bool m_initialized = false;

private:
    Cage3D  m_arap;

    // Movement
    int m_forward;
    int m_sideways;
    int m_vertical;
};
