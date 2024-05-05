#include "synccage2d.h"

SyncCage2D::SyncCage2D(Mode2D mode) : Cage2D(mode) {}

void SyncCage2D::linkCage(SyncCage2D *other)
{
    m_linkedCage = other;
}

// Move an anchored vertex, defined by its index, to targetPosition
void SyncCage2D::move(int vertex, Vector3f targetPosition)
{
    if (m_isSynced) {
        return;
    }

    std::vector<Eigen::Vector3f> new_vertices = m_shape_cage.getVertices();
    const std::unordered_set<int>& anchors = m_shape_cage.getAnchors();

    // Update cage vertex positions
    updateCage(new_vertices, vertex, targetPosition);

    // Update object vertex positions
    //    object2D.updateVertices(cagePoints, cageEdges);
    object2D.updateVertices(cagePoints, cageEdges, controlPoints);
    std::vector<Eigen::Vector3f> new_object_vertices = object2D.getVertices();

    m_shape_cage.setVertices2d(new_vertices);
    m_shape_object.setVertices2d(new_object_vertices);

    m_isSynced = true;
    if (!m_linkedCage->isSynced()) {
        m_linkedCage->move(vertex, targetPosition);
    }
    m_isSynced = false;
}

void SyncCage2D::moveAllAnchors(int vertex, Vector3f pos)
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

    // Update object vertex positions
    //    object2D.updateVertices(cagePoints, cageEdges);
    object2D.updateVertices(cagePoints, cageEdges, controlPoints);
    std::vector<Eigen::Vector3f> new_object_vertices = object2D.getVertices();

    m_shape_cage.setVertices2d(new_vertices);
    m_shape_object.setVertices2d(new_object_vertices);

    m_isSynced = true;
    if (!m_linkedCage->isSynced()) {
        m_linkedCage->moveAllAnchors(vertex, pos, anchors);
    }
    m_isSynced = false;
}

void SyncCage2D::moveAllAnchors(int vertex, Vector3f pos, const std::unordered_set<int>& anchors)
{
    std::vector<Eigen::Vector3f> new_vertices = m_shape_cage.getVertices();

    Vector3f delta = pos - new_vertices[vertex];

    // Apply delta to all anchors
    for (int a : anchors) {
        new_vertices[a] += delta;
    }

    // Update cage vertex positions
    updateCage(new_vertices, vertex, pos);

    // Update object vertex positions
    //    object2D.updateVertices(cagePoints, cageEdges);
    object2D.updateVertices(cagePoints, cageEdges, controlPoints);
    std::vector<Eigen::Vector3f> new_object_vertices = object2D.getVertices();

    m_shape_cage.setVertices2d(new_vertices);
    m_shape_object.setVertices2d(new_object_vertices);
}

bool SyncCage2D::isSynced()
{
    return m_isSynced;
}
