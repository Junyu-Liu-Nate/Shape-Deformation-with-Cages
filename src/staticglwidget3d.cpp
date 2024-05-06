#include "StaticGLWidget3D.h"

#include <QApplication>
#include <QKeyEvent>
#include <iostream>

#define SPEED 1.5
#define ROTATE_SPEED 0.0025

using namespace std;
using namespace Eigen;


Camera StaticGLWidget3D::m_camera = Camera();

float StaticGLWidget3D::m_movementScaling = 0.f;
float StaticGLWidget3D::m_vertexSelectionThreshold = 0.f;
float StaticGLWidget3D::m_vSize = 0.f;

int StaticGLWidget3D::m_lastX = 0;
int StaticGLWidget3D::m_lastY = 0;
bool StaticGLWidget3D::m_leftCapture = false;
bool StaticGLWidget3D::m_rightCapture = false;
bool StaticGLWidget3D::m_shiftFlag = false;

bool StaticGLWidget3D::m_initialized = false;

int StaticGLWidget3D::m_forward = 0;
int StaticGLWidget3D::m_sideways = 0;
int StaticGLWidget3D::m_vertical = 0;

int StaticGLWidget3D::m_lastSelectedVertex = -1;
SelectMode StaticGLWidget3D::m_rightClickSelectMode = SelectMode::None;

StaticGLWidget3D::StaticGLWidget3D(QWidget *parent, SyncCage3D *syncCage) :
    QOpenGLWidget(parent),
    m_syncCage3d(syncCage),
    m_defaultShader(),
    m_pointShader(),
    // Movement
    m_deltaTimeProvider(),
    m_intervalTimer()
{
    // GLWidget needs all mouse move events, not just mouse drag events
    setMouseTracking(true);

    // Hide the cursor since this is a fullscreen app
    QApplication::setOverrideCursor(Qt::ArrowCursor);

    // GLWidget needs keyboard focus
    setFocusPolicy(Qt::StrongFocus);

    // Function tick() will be called once per interva
    connect(&m_intervalTimer, SIGNAL(timeout()), this, SLOT(tick()));
}

StaticGLWidget3D::~StaticGLWidget3D()
{
    if (m_defaultShader != nullptr) delete m_defaultShader;
    if (m_pointShader   != nullptr) delete m_pointShader;
}

// ================== Basic OpenGL Overrides

void StaticGLWidget3D::initializeGL()
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
    m_defaultShader = new Shader(":resources/shaders/shader.vert", ":resources/shaders/shader.frag");
    m_pointShader = new Shader(":resources/shaders/anchorPoint.vert", ":resources/shaders/anchorPoint.geom", ":resources/shaders/anchorPoint.frag");

    // Initialize ARAP, and get parameters needed to decide the camera position, etc
    Vector3f coeffMin, coeffMax;
    m_syncCage3d->init(coeffMin, coeffMax);

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

void StaticGLWidget3D::paintGL()
{
    if (!m_initialized) {
        return;
    }

    glClearColor(1, 1, 1, 1);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_defaultShader->bind();
    m_defaultShader->setUniform("proj", m_camera.getProjection());
    m_defaultShader->setUniform("view", m_camera.getView());
    m_syncCage3d->draw(m_defaultShader, GL_TRIANGLES);
    m_defaultShader->unbind();

    glClear(GL_DEPTH_BUFFER_BIT);

    m_pointShader->bind();
    m_pointShader->setUniform("proj",   m_camera.getProjection());
    m_pointShader->setUniform("view",   m_camera.getView());
    m_pointShader->setUniform("vSize",  m_vSize);
    m_pointShader->setUniform("width",  width());
    m_pointShader->setUniform("height", height());
    m_syncCage3d->draw(m_pointShader, GL_POINTS);
    m_pointShader->unbind();
}

void StaticGLWidget3D::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    m_camera.setAspect(static_cast<float>(w) / h);
}

// ================== Event Listeners

Eigen::Vector3f StaticGLWidget3D::transformToWorldRay(int x, int y)
{
    Eigen::Vector4f clipCoords = Eigen::Vector4f(
        (float(x) / width()) * 2.f - 1.f,
        1.f - (float(y) / height()) * 2.f,
        -1.f,
        1.f);

    Eigen::Vector4f transformed_coords = m_camera.getProjection().inverse() * clipCoords;
    transformed_coords = Eigen::Vector4f(transformed_coords.x(), transformed_coords.y(), -1.f, 0.f);
    transformed_coords = m_camera.getView().inverse() * transformed_coords;

    return Eigen::Vector3f(transformed_coords.x(), transformed_coords.y(), transformed_coords.z()).normalized();
}

void StaticGLWidget3D::mousePressEvent(QMouseEvent *event)
{
    // Get current mouse coordinates
    const int currX = event->position().x();
    const int currY = event->position().y();

    // Get closest vertex to ray
    const Vector3f ray = transformToWorldRay(currX, currY);
    const int closest_vertex = m_syncCage3d->getClosestVertex(m_camera.getPosition(), ray, m_vertexSelectionThreshold);

    // Switch on button
    switch (event->button()) {
    case Qt::MouseButton::RightButton: {
        // Capture
        m_rightCapture = true;
        // Anchor/un-anchor the vertex
        m_rightClickSelectMode = m_syncCage3d->select(m_pointShader, closest_vertex);
        break;
    }
    case Qt::MouseButton::LeftButton: {
        // Capture
        m_leftCapture = true;
        // Select this vertex
        m_lastSelectedVertex = closest_vertex;
        break;
    }
    default: break;
    }

    // Set last mouse coordinates
    m_lastX = currX;
    m_lastY = currY;
}

void StaticGLWidget3D::mouseMoveEvent(QMouseEvent *event)
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
        const int closest_vertex = m_syncCage3d->getClosestVertex(m_camera.getPosition(), ray, m_vertexSelectionThreshold);

        // Anchor/un-anchor the vertex
        if (m_rightClickSelectMode == SelectMode::None) {
            m_rightClickSelectMode = m_syncCage3d->select(m_pointShader, closest_vertex);
        } else {
            m_syncCage3d->selectWithSpecifiedMode(m_pointShader, closest_vertex, m_rightClickSelectMode);
        }

        return;
    }

    if (m_lastSelectedVertex != -1 && m_shiftFlag && m_syncCage3d->getAnchorPos(m_lastSelectedVertex, pos, ray, m_camera.getPosition())) {
        m_syncCage3d->moveAllAnchors(m_lastSelectedVertex, pos);
    }
    // If the selected point is an anchor point
    else if (m_lastSelectedVertex != -1 && m_syncCage3d->getAnchorPos(m_lastSelectedVertex, pos, ray, m_camera.getPosition())) {
        // Move it
        m_syncCage3d->move(m_lastSelectedVertex, pos);
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

void StaticGLWidget3D::mouseReleaseEvent(QMouseEvent *event)
{
    m_leftCapture = false;
    m_lastSelectedVertex = -1;

    m_rightCapture = false;
    m_rightClickSelectMode = SelectMode::None;
}

void StaticGLWidget3D::wheelEvent(QWheelEvent *event)
{
    float zoom = 1 - event->pixelDelta().y() * 0.1f / 120.f;
    m_camera.zoom(zoom);
}

void StaticGLWidget3D::keyPressEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat()) return;

    switch (event->key())
    {
    case Qt::Key_W: m_forward  += SPEED; break;
    case Qt::Key_S: m_forward  -= SPEED; break;
    case Qt::Key_A: m_sideways -= SPEED; break;
    case Qt::Key_D: m_sideways += SPEED; break;
    case Qt::Key_F: m_vertical -= SPEED; break;
    case Qt::Key_R: m_vertical += SPEED; break;
    case Qt::Key_C: m_camera.toggleIsOrbiting(); break;
    case Qt::Key_Equal: m_vSize *= 11.0f / 10.0f; break;
    case Qt::Key_Minus: m_vSize *= 10.0f / 11.0f; break;
    case Qt::Key_Escape: QApplication::quit(); break;
    case Qt::Key_Shift: m_shiftFlag = true; break;
    case Qt::Key_0: m_syncCage3d->clearAnchors(); break;
    }
}

void StaticGLWidget3D::keyReleaseEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat()) return;

    switch (event->key())
    {
    case Qt::Key_W: m_forward  -= SPEED; break;
    case Qt::Key_S: m_forward  += SPEED; break;
    case Qt::Key_A: m_sideways += SPEED; break;
    case Qt::Key_D: m_sideways -= SPEED; break;
    case Qt::Key_F: m_vertical += SPEED; break;
    case Qt::Key_R: m_vertical -= SPEED; break;
    case Qt::Key_Shift: m_shiftFlag = false; break;
    }
}

// ================== Physics Tick

void StaticGLWidget3D::tick()
{
    float deltaSeconds = m_deltaTimeProvider.restart() / 1000.f;

    // Move camera
    auto look = m_camera.getLook();
    look.y() = 0;
    look.normalize();
    Eigen::Vector3f perp(-look.z(), 0, look.x());
    Eigen::Vector3f moveVec = m_forward * look.normalized() + m_sideways * perp.normalized() + m_vertical * Eigen::Vector3f::UnitY();
    moveVec *= m_movementScaling;
    moveVec *= deltaSeconds;
    m_camera.move(moveVec);

    // Flag this view for repainting (Qt will call paintGL() soon after)
    update();
}

void StaticGLWidget3D::setObjectFilePath(const QString &path)
{
    m_syncCage3d->setObjectFilePath(path);
}

void StaticGLWidget3D::setCageFilePath(const QString &path)
{
    m_syncCage3d->setCageFilePath(path);
}

void StaticGLWidget3D::init()
{
    if (!m_syncCage3d->isObjectFilePathSet() || !m_syncCage3d->isCageFilePathSet()) {
        return;
    }

    m_initialized = true;
}
