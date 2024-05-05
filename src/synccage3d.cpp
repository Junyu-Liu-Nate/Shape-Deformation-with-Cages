#include "SyncCage3D.h"

SyncCage3D::SyncCage3D(bool useGreen) : Cage3D(useGreen) {}

void SyncCage3D::linkCage(SyncCage3D *other) {
    m_linkedCage = other;
}

void SyncCage3D::move(int vertex, Vector3f targetPosition)
{
    if (m_isSynced) {
        return;
    }

    std::vector<Eigen::Vector3f> new_vertices = m_shape_cage.getVertices();
    const std::unordered_set<int>& anchors = m_shape_cage.getAnchors();

    // Update cage vertex positions
    updateCage(new_vertices, vertex, targetPosition);
    heMesh.updateVertexPos(new_vertices);

    // Update object vertex positions
    object3D.updateVertices(heMesh);
    std::vector<Eigen::Vector3f> new_object_vertices = object3D.getVertices();

    m_shape_cage.setVertices(new_vertices);
    m_shape_object.setVertices(new_object_vertices);

    m_isSynced = true;
    if (!m_linkedCage->isSynced()) {
        m_linkedCage->move(vertex, targetPosition);
    }
    m_isSynced = false;
}

void SyncCage3D::moveAllAnchors(int vertex, Vector3f pos)
{
    if (m_isSynced) {
        return;
    }

    std::vector<Eigen::Vector3f> new_vertices = m_shape_cage.getVertices();
    const std::unordered_set<int>& anchors = m_shape_cage.getAnchors();

    Vector3f delta = pos - new_vertices[vertex];

    // Apply delta to all anchors
    for (int a : anchors) {
        new_vertices[a] += delta;
    }

    // Update cage vertex positions
    updateCage(new_vertices, vertex, pos);
    heMesh.updateVertexPos(new_vertices);

    // Update object vertex positions
    object3D.updateVertices(heMesh);
    std::vector<Eigen::Vector3f> new_object_vertices = object3D.getVertices();

    m_shape_cage.setVertices(new_vertices);
    m_shape_object.setVertices(new_object_vertices);

    m_isSynced = true;
    if (!m_linkedCage->isSynced()) {
        m_linkedCage->moveAllAnchors(vertex, pos, anchors);
    }
    m_isSynced = false;
}

void SyncCage3D::moveAllAnchors(int vertex, Vector3f pos, const std::unordered_set<int>& anchors)
{
    std::vector<Eigen::Vector3f> new_vertices = m_shape_cage.getVertices();

    Vector3f delta = pos - new_vertices[vertex];

    // Apply delta to all anchors
    for (int a : anchors) {
        new_vertices[a] += delta;
    }

    // Update cage vertex positions
    updateCage(new_vertices, vertex, pos);
    heMesh.updateVertexPos(new_vertices);

    // Update object vertex positions
    object3D.updateVertices(heMesh);
    std::vector<Eigen::Vector3f> new_object_vertices = object3D.getVertices();

    m_shape_cage.setVertices(new_vertices);
    m_shape_object.setVertices(new_object_vertices);
}

bool SyncCage3D::isSynced()
{
    return m_isSynced;
}
