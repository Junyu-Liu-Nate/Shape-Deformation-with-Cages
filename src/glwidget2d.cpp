#include "glwidget2d.h"

#include <QApplication>
#include <QKeyEvent>

#define SPEED 1.5
#define ROTATE_SPEED 0.0025

GLWidget2D::GLWidget2D() :
    m_cage(Mode2D::MVC),
    m_rightClickSelectModeOnCage(SelectMode::None),
    m_rightClickSelectModeOnCtrlPt(SelectMode::None)
{}

void GLWidget2D::initializeGL()
{
    if (!m_initialized) {
        return;
    }

    // Initialize GL extension wrangler
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err != GLEW_OK) fprintf(stderr, "Error while initializing GLEW: %s\n", glewGetErrorString(err));
    fprintf(stdout, "Successfully initialized GLEW %s\n", glewGetString(GLEW_VERSION));

    // Set clear color to white
    glClearColor(1, 1, 1, 1);

    // Enable depth-testing and backface culling
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    // Initialize shaders
    m_defaultShader = new Shader(":resources/shaders/texture.vert", ":resources/shaders/texture.frag");
    m_pointShader = new Shader(":resources/shaders/anchorPoint.vert", ":resources/shaders/anchorPoint.geom", ":resources/shaders/anchorPoint.frag");

    // Initialize ARAP, and get parameters needed to decide the camera position, etc
    Vector3f coeffMin, coeffMax;
    m_cage.init(coeffMin, coeffMax);

    Vector3f center = (coeffMax + coeffMin) / 2.0f;
    float extentLength  = (coeffMax - coeffMin).norm();

    // Screen-space size of vertex points
    m_vSize = 0.005 * extentLength;

    // Scale all movement by this amount
    m_movementScaling = extentLength * 0.5;

    // When raycasting, select closest vertex within this distance
    m_vertexSelectionThreshold = extentLength * 0.025;

    // Note for maintainers: Z-up
    float fovY = 120;
    float nearPlane = 0.0001f;
    float farPlane  = 3 * extentLength;

    // Initialize camera with a reasonable transform
    Eigen::Vector3f eye    = center - Eigen::Vector3f::UnitZ() * extentLength;
    Eigen::Vector3f target = center;
    m_camera.lookAt(eye, target);
    m_camera.setOrbitPoint(target);
    m_camera.setPerspective(120, width() / static_cast<float>(height()), nearPlane, farPlane);

    m_deltaTimeProvider.start();
    m_intervalTimer.start(1000 / 60);
}

void GLWidget2D::paintGL()
{
    if (!m_initialized) {
        return;
    }

    glClearColor(1, 1, 1, 1);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_defaultShader->bind();
    m_defaultShader->setUniform("proj", m_camera.getProjection());
    m_defaultShader->setUniform("view", m_camera.getView());
    m_cage.draw(m_defaultShader, GL_TRIANGLES);
    m_defaultShader->unbind();

    glClear(GL_DEPTH_BUFFER_BIT);

    m_pointShader->bind();
    m_pointShader->setUniform("proj",   m_camera.getProjection());
    m_pointShader->setUniform("view",   m_camera.getView());
    m_pointShader->setUniform("vSize",  m_vSize);
    m_pointShader->setUniform("width",  width());
    m_pointShader->setUniform("height", height());
    m_cage.draw(m_pointShader, GL_POINTS);
    m_pointShader->unbind();
}

void GLWidget2D::mousePressEvent(QMouseEvent *event)
{
    // Get current mouse coordinates
    const int currX = event->position().x();
    const int currY = event->position().y();

    // Get closest vertex to ray
    const Vector3f ray = transformToWorldRay(currX, currY);
    const int closestVertexOnCage = m_cage.getClosestVertexOnCage(m_camera.getPosition(), ray, m_vertexSelectionThreshold);
    const int closestVertexOnCtrlPt = m_cage.getClosestVertexOnCtrlPt(m_camera.getPosition(), ray, m_vertexSelectionThreshold);

    // Switch on button
    switch (event->button()) {
    case Qt::MouseButton::RightButton: {
        // Capture
        m_rightCapture = true;
        // Anchor/un-anchor the vertex
        m_rightClickSelectModeOnCage = m_cage.selectOnCage(m_pointShader, closestVertexOnCage);
        m_rightClickSelectModeOnCtrlPt = m_cage.selectOnCtrlPt(m_pointShader, closestVertexOnCtrlPt);
        break;
    }
    case Qt::MouseButton::LeftButton: {
        // Capture
        m_leftCapture = true;
        // Select this vertex
        m_lastSelectedVertexOnCage = closestVertexOnCage;
        m_lastSelectedVertexOnCtrlPt = closestVertexOnCtrlPt;
        break;
    }
    default: break;
    }

    // Set last mouse coordinates
    m_lastX = currX;
    m_lastY = currY;
}

void GLWidget2D::mouseMoveEvent(QMouseEvent *event)
{
    // Return if neither mouse button is currently held down
    if (!(m_leftCapture || m_rightCapture)) {
        return;
    }

    // Get current mouse coordinates
    const int currX = event->position().x();
    const int currY = event->position().y();

    // Find ray
    const Vector3f ray = transformToWorldRay(event->position().x(), event->position().y());
    Vector3f pos;

    // If right is held down
    if (m_rightCapture) {
        // Get closest vertex to ray
        const int closestVertexOnCage = m_cage.getClosestVertexOnCage(m_camera.getPosition(), ray, m_vertexSelectionThreshold);

        // Anchor/un-anchor the vertex on cage
        if (m_rightClickSelectModeOnCage == SelectMode::None) {
            m_rightClickSelectModeOnCage = m_cage.selectOnCage(m_pointShader, closestVertexOnCage);
        } else {
            m_cage.selectWithSpecifiedModeOnCage(m_pointShader, closestVertexOnCage, m_rightClickSelectModeOnCage);
        }

        // Get closest vertex to ray
        const int closestVertexOnCtrlPt = m_cage.getClosestVertexOnCtrlPt(m_camera.getPosition(), ray, m_vertexSelectionThreshold);

        // Anchor/un-anchor the vertex on cage
        if (m_rightClickSelectModeOnCtrlPt == SelectMode::None) {
            m_rightClickSelectModeOnCtrlPt = m_cage.selectOnCtrlPt(m_pointShader, closestVertexOnCtrlPt);
        } else {
            m_cage.selectWithSpecifiedModeOnCtrlPt(m_pointShader, closestVertexOnCtrlPt, m_rightClickSelectModeOnCtrlPt);
        }

        return;
    }

    // Selected point on cage is an anchor point and shift is pressed
    if (m_lastSelectedVertexOnCage != -1 && m_shiftFlag && m_cage.getAnchorPosOnCage(m_lastSelectedVertexOnCage, pos, ray, m_camera.getPosition())) {
        m_cage.moveAllAnchors(m_lastSelectedVertexOnCage, pos);
    }
    // Selected point on cage is an anchor point
    else if (m_lastSelectedVertexOnCage != -1 && m_cage.getAnchorPosOnCage(m_lastSelectedVertexOnCage, pos, ray, m_camera.getPosition())) {
        // Move it
        m_cage.move(m_lastSelectedVertexOnCage, pos);
    }
    // Selected point on control point is an anchor point
    else if (m_lastSelectedVertexOnCtrlPt != -1 && m_cage.getAnchorPosOnCtrlPt(m_lastSelectedVertexOnCtrlPt, pos, ray, m_camera.getPosition())) {
        m_cage.moveCtrlPt(m_lastSelectedVertexOnCtrlPt, pos);
    } else {
        // Rotate the camera
        const int deltaX = currX - m_lastX;
        const int deltaY = currY - m_lastY;
        if (deltaX != 0 || deltaY != 0) {
            m_camera.rotate(deltaY * ROTATE_SPEED, -deltaX * ROTATE_SPEED);
        }
    }

    // Set last mouse coordinates
    m_lastX = currX;
    m_lastY = currY;
}

void GLWidget2D::mouseReleaseEvent(QMouseEvent *event)
{
    m_leftCapture = false;
    m_lastSelectedVertexOnCage = -1;
    m_lastSelectedVertexOnCtrlPt = -1;

    m_rightCapture = false;
    m_rightClickSelectModeOnCage = SelectMode::None;
    m_rightClickSelectModeOnCtrlPt = SelectMode::None;
}

void GLWidget2D::setTextureFilePath(const QString &path)
{
    m_cage.setTextureFilePath(path);
}

void GLWidget2D::setCageFilePath(const QString &path)
{
    m_cage.setCageFilePath(path);
}

void GLWidget2D::init()
{
    if (!m_cage.isTextureFilePathSet() || !m_cage.isCageFilePathSet()) {
        return;
    }

    m_initialized = true;
}

