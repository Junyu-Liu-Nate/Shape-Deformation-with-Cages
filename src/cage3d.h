#pragma once

#include "graphics/shape.h"
#include "Eigen/StdList"
#include "Eigen/StdVector"
#include <Eigen/Sparse>
#include <QtConcurrent>
#include "mesh_struct/halfedgemesh.h"
#include "object3d.h"
#include <float.h> // For DBL_MAX
#include <QtConcurrent/QtConcurrentMap>

class Shader;

class Cage3D
{
private:
    std::string m_objectFilePath;
    std::string m_cageFilePath;

protected:
    Shape m_shape_cage;
    Shape m_shape_object;

    // false for MVC, true for Green Coordinate
    bool m_useGreen = false;

public:
    Cage3D(bool useGreen);

    void init(Eigen::Vector3f &min, Eigen::Vector3f &max);
    virtual void move(int vertex, Eigen::Vector3f pos);
    virtual void moveAllAnchors(int vertex, Eigen::Vector3f pos);

    static bool m_showSkeleton;
    HalfEdgeMesh heMesh;
    
    void updateCage(std::vector<Eigen::Vector3f> new_vertices, int vertex, Vector3f targetPosition);

    Object3D object3D;
    void buildVertexList(vector<Vector3f> objectVertices);
    void updatePosition();

    bool rayIntersectsTriangle(const Eigen::Vector3d& P, const Eigen::Vector3d& D, const Face& face);
    bool isPointOutsideMesh(const Eigen::Vector3d& point, HalfEdgeMesh& mesh);
    bool isPointOnBoundary(const Eigen::Vector3d& point, HalfEdgeMesh& mesh);

    // ================== Students, If You Choose To Modify The Code Below, It's On You

    int getClosestVertex(Eigen::Vector3f start, Eigen::Vector3f ray, float threshold)
    {
        return m_shape_cage.getClosestVertex(start, ray, threshold);
    }

    void draw(Shader *shader, GLenum mode)
    {
        if (mode == GL_POINTS) {
            m_shape_cage.draw(shader, mode);
        }else {
            m_shape_cage.draw(shader, GL_LINE_LOOP);
            if (!m_showSkeleton) {
                m_shape_object.draw(shader, GL_TRIANGLES);
            } else {
                m_shape_object.draw(shader, GL_LINE_LOOP);
            }
        }
    }

    SelectMode select(Shader *shader, int vertex)
    {
        return m_shape_cage.select(shader, vertex);
    }

    bool selectWithSpecifiedMode(Shader *shader, int vertex, SelectMode mode)
    {
        return m_shape_cage.selectWithSpecifiedMode(shader, vertex, mode);
    }

    bool getAnchorPos(int lastSelected, Eigen::Vector3f& pos, Eigen::Vector3f ray, Eigen::Vector3f start)
    {
        return m_shape_cage.getAnchorPos(lastSelected, pos, ray, start);
    }

    void setObjectFilePath(const QString &path);
    bool isObjectFilePathSet();
    void setCageFilePath(const QString &path);
    bool isCageFilePathSet();

    bool clearAnchors() {
        m_shape_cage.clearAnchors();
    }
};
