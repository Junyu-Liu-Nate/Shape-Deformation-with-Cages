#include "graphics/camera.h"

#include <iostream>

Camera::Camera()
    : m_position(0,0,0),
      m_pitch(0),
      m_yaw(0),
      m_look(0, 0, 1),
      m_orbitPoint(0, 0, 0),
      m_isOrbiting(false),
      m_view(Eigen::Matrix4f::Identity()),
      m_proj(Eigen::Matrix4f::Identity()),
      m_viewDirty(true),
      m_projDirty(true),
      m_fovY(90),
      m_aspect(1),
      m_near(0.1f),
      m_far(50.f),
      m_zoom(1)
{}

// ================== Position

void Camera::setPosition(const Eigen::Vector3f &position)
{
    m_position  = position;
    m_viewDirty = true;
}

void Camera::move(const Eigen::Vector3f &deltaPosition)
{
    if (deltaPosition.squaredNorm() == 0) return;

    m_position += deltaPosition;

    if (m_isOrbiting) {
        m_orbitPoint += deltaPosition;
    }

    m_viewDirty = true;
}

Eigen::Vector3f Camera::getPosition()
{
    return m_position;
}

// ================== Rotation

void Camera::setRotation(float pitch, float yaw)
{
    m_pitch = pitch;
    m_yaw   = yaw;
    m_viewDirty = true;
    updateLook();
}

void Camera::rotate(float deltaPitch, float deltaYaw)
{
    m_pitch += deltaPitch;
    m_yaw   += deltaYaw;
    m_pitch  = std::clamp(m_pitch, (float) -M_PI_2 + 0.01f, (float) M_PI_2 - 0.01f);
    m_viewDirty = true;
    updateLook();

    if (m_isOrbiting) {
        m_position = m_orbitPoint - m_look * m_zoom;
    }
}

// ================== Position and Rotation

void Camera::lookAt(const Eigen::Vector3f &eye, const Eigen::Vector3f &target)
{
    m_position  = eye;
    m_look      = (target - eye).normalized();
    m_viewDirty = true;
    updatePitchAndYaw();
}

// ================== Orbiting

void Camera::setOrbitPoint(const Eigen::Vector3f &orbitPoint)
{
    m_orbitPoint = orbitPoint;
    m_viewDirty = true;
}

bool Camera::getIsOrbiting()
{
    return m_isOrbiting;
}

void Camera::setIsOrbiting(bool isOrbiting)
{
    m_isOrbiting = isOrbiting;
    m_viewDirty = true;
}

void Camera::toggleIsOrbiting()
{
    m_isOrbiting = !m_isOrbiting;
    m_viewDirty = true;

    if (m_isOrbiting) {
        m_zoom = (m_orbitPoint - m_position).norm();
        m_look = (m_orbitPoint - m_position).normalized();
        updatePitchAndYaw();
    }
}

void Camera::zoom(float zoomMultiplier)
{
    if (!m_isOrbiting) return;

    m_zoom *= zoomMultiplier;
    m_position = m_orbitPoint - m_look * m_zoom;
    m_viewDirty = true;
}

// ================== Important Getters

const Eigen::Matrix4f &Camera::getView()
{
    if (m_viewDirty) {
        Eigen::Matrix3f R;
        Eigen::Vector3f f = m_look.normalized();
        Eigen::Vector3f u = Eigen::Vector3f::UnitY();
        Eigen::Vector3f s = f.cross(u).normalized();
        u = s.cross(f);
        R.col(0) = s;
        R.col(1) = u;
        R.col(2) = -f;
        m_view.topLeftCorner<3, 3>() = R.transpose();
        m_view.topRightCorner<3, 1>() = -R.transpose() * m_position;
        m_view(3, 3) = 1.f;
        m_viewDirty = false;
    }
    return m_view;
}

const Eigen::Matrix4f &Camera::getProjection()
{
    if (m_projDirty) {
        float theta = m_fovY * 0.5f;
        float invRange = 1.f / (m_far - m_near);
        float invtan = 1.f / tanf(theta);
        m_proj(0, 0) = invtan / m_aspect;
        m_proj(1, 1) = invtan;
        m_proj(2, 2) = -(m_near + m_far) * invRange;
        m_proj(3, 2) = -1;
        m_proj(2, 3) = -2 * m_near * m_far * invRange;
        m_proj(3, 3) = 0;
        m_projDirty = false;
    }
    return m_proj;
}

const Eigen::Vector3f &Camera::getLook()
{
    return m_look;
}

// ================== Intrinsics

void Camera::setPerspective(float fovY, float aspect, float near, float far)
{
    m_fovY = fovY;
    m_aspect = aspect;
    m_near = near;
    m_far = far;
    m_projDirty = true;
}

void Camera::setAspect(float aspect)
{
    m_aspect = aspect;
    m_projDirty = true;
}

// ================== Private Helpers

void Camera::updateLook()
{
    m_look = Eigen::Vector3f(0, 0, 1);
    m_look = Eigen::AngleAxis<float>(m_pitch, Eigen::Vector3f::UnitX()) * m_look;
    m_look = Eigen::AngleAxis<float>(m_yaw,   Eigen::Vector3f::UnitY()) * m_look;
    m_look = m_look.normalized();
}

void Camera::updatePitchAndYaw()
{
    m_pitch = asinf(-m_look.y());
    m_yaw   = atan2f(m_look.x(), m_look.z());
}
