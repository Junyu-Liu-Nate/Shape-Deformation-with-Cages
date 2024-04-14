#ifndef CAGE2D_H
#define CAGE2D_H

#include "graphics/shape.h"
#include "Eigen/StdList"
#include "Eigen/StdVector"
#include <Eigen/Sparse>
#include <QtConcurrent>
#include "mesh_struct/halfedgemesh.h"
#include "object2d.h"

class Shader;

class Cage2D
{
private:
    Shape m_shape_cage;
    Shape m_shape_object;

public:
    Cage2D();

    void init(Eigen::Vector3f &min, Eigen::Vector3f &max);
    void move(int vertex, Eigen::Vector3f pos);

    // Hardcoded cage points and edges
    vector<Vector2f> cagePoints;
    vector<std::pair<Vector2f, Vector2f>> cageEdges;

    void updateCage(std::vector<Eigen::Vector3f> new_vertices, int vertex, Vector3f targetPosition);

    //----- For test only: 2D case
    Object2D object2D;
    void buildVertexList2D(vector<Vector3f> objectVertices);

    // ================== Students, If You Choose To Modify The Code Below, It's On You

    int getClosestVertex(Eigen::Vector3f start, Eigen::Vector3f ray, float threshold)
    {
        return m_shape_cage.getClosestVertex(start, ray, threshold);
    }

    void draw(Shader *shader, GLenum mode)
    {
        m_shape_cage.draw(shader, mode);
        m_shape_object.draw(shader, mode);
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
};

#endif // CAGE2D_H
