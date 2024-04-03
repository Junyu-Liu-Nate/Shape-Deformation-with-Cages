#pragma once

#include "graphics/shape.h"
#include "Eigen/StdList"
#include "Eigen/StdVector"
#include <Eigen/Sparse>
#include <QtConcurrent>
#include "mesh_struct/halfedgemesh.h"

class Shader;

class ARAP
{
private:
    Shape m_shape;

public:
    ARAP();

    void init(Eigen::Vector3f &min, Eigen::Vector3f &max);
    void move(int vertex, Eigen::Vector3f pos);

    HalfEdgeMesh heMesh;

    //----------------------- ARAP computation  -----------------------//
    void iterativeOptimize(std::vector<Eigen::Vector3f> new_vertices, const std::unordered_set<int>& anchors, int vertex, Vector3f targetPosition);
    void iterativeOptimizeParallel(std::vector<Eigen::Vector3f> new_vertices, const std::unordered_set<int>& anchors, int vertex, Vector3f targetPosition);

    void initialize(std::vector<Eigen::Vector3f> new_vertices, int vertex, Vector3f targetPosition);
    void update(Eigen::MatrixXf p, const std::unordered_set<int>& anchors);

    void computeEdgeWeights();
    float cotangentOfAngle(const Eigen::Vector3f& u, const Eigen::Vector3f& v);
    void computeVertexRotation();

    Eigen::MatrixXf L;
    Eigen::MatrixXf RHS;
    void assembleL(const std::unordered_set<int>& anchors);
    void assenbleRHS(const std::unordered_set<int>& anchors);

    // ================== Students, If You Choose To Modify The Code Below, It's On You

    int getClosestVertex(Eigen::Vector3f start, Eigen::Vector3f ray, float threshold)
    {
        return m_shape.getClosestVertex(start, ray, threshold);
    }

    void draw(Shader *shader, GLenum mode)
    {
        m_shape.draw(shader, mode);
    }

    SelectMode select(Shader *shader, int vertex)
    {
        return m_shape.select(shader, vertex);
    }

    bool selectWithSpecifiedMode(Shader *shader, int vertex, SelectMode mode)
    {
        return m_shape.selectWithSpecifiedMode(shader, vertex, mode);
    }

    bool getAnchorPos(int lastSelected, Eigen::Vector3f& pos, Eigen::Vector3f ray, Eigen::Vector3f start)
    {
        return m_shape.getAnchorPos(lastSelected, pos, ray, start);
    }
};
