#ifndef CAGE2D_H
#define CAGE2D_H

#include "graphics/shape.h"
#include "Eigen/StdList"
#include "Eigen/StdVector"
#include <Eigen/Sparse>
#include <QtConcurrent>
#include "mesh_struct/margincage2d.h"
#include "object2d.h"
#include "common.h"

class Shader;

class Cage2D
{
private:
    Shape m_shape_cage;
    Shape m_shape_object;
    Shape m_shape_control_points;

    std::string m_textureFilePath;
    std::string m_cageFilePath;

public:
    Cage2D();

    void init(Eigen::Vector3f &min, Eigen::Vector3f &max);
    void move(int vertex, Eigen::Vector3f pos);
    void moveAllAnchors(int vertex, Eigen::Vector3f pos);
    void moveCtrlPt(int vertex, Vector3f targetPosition);

    vector<TwoDVertex> cagePoints;
    vector<TwoDEdge> cageEdges;

    void updateCage(std::vector<Eigen::Vector3f>& new_vertices, int vertex, Vector3f targetPosition);

    void findMarginEdges(vector<Vector3i>& triangles, vector<Vector3f>& vertices, vector<Vector3f>& controlPts);

    void tessellateMesh(vector<Vector3i>& faces, vector<Vector3f>& vertices, int finalRow, int finalCol, vector<Vector2f> &uvCoords);

    //----- Linear 2D case
    Object2D object2D;
    void buildVertexList2D(vector<Vector3f> objectVertices, const vector<Vector3f> vertices, const vector<Vector3i> triangles);

    void setTextureFilePath(const QString &path);
    bool isTextureFilePathSet();
    void setCageFilePath(const QString &path);
    bool isCageFilePathSet();

    //----- High order 2D case
    int degree = 3;
    unordered_map<std::tuple<int, int, int>, ControlPoint, tuple_hash> controlPoints;

    // ---Check whether an object vertex is inside the cage or not
    bool isPointInTriangle(const Vector3f& pt, const Vector3f& v1, const Vector3f& v2, const Vector3f& v3);
    bool isPointInsideMesh(const Vector3f& point, const vector<Vector3f>& vertices, const vector<Vector3i>& triangles);
    bool isPointOnEdge(const Eigen::Vector3f& point, const Eigen::Vector3f& edgeStart, const Eigen::Vector3f& edgeEnd);
    bool isPointOnBoundary(const Vector3f& point);

    // ================== Students, If You Choose To Modify The Code Below, It's On You

    int getClosestVertexOnCage(Eigen::Vector3f start, Eigen::Vector3f ray, float threshold)
    {
        return m_shape_cage.getClosestVertex(start, ray, threshold);
    }

    int getClosestVertexOnCtrlPt(Eigen::Vector3f start, Eigen::Vector3f ray, float threshold)
    {
        return m_shape_control_points.getClosestVertex(start, ray, threshold);
    }

    void draw(Shader *shader, GLenum mode)
    {
        if (mode == GL_POINTS) {
            m_shape_cage.draw(shader, mode);
            m_shape_control_points.draw(shader, mode);
        } else {
            m_shape_cage.draw(shader, GL_LINES);
            m_shape_object.draw(shader, GL_TRIANGLES);
//            m_shape_object.draw(shader, GL_LINES);
        }
    }

    SelectMode selectOnCage(Shader *shader, int vertex)
    {
        return m_shape_cage.select(shader, vertex);
    }

    SelectMode selectOnCtrlPt(Shader *shader, int vertex)
    {
        return m_shape_control_points.select(shader, vertex);
    }

    bool selectWithSpecifiedModeOnCage(Shader *shader, int vertex, SelectMode mode)
    {
        return m_shape_cage.selectWithSpecifiedMode(shader, vertex, mode);
    }

    bool selectWithSpecifiedModeOnCtrlPt(Shader *shader, int vertex, SelectMode mode)
    {
        return m_shape_control_points.selectWithSpecifiedMode(shader, vertex, mode);
    }

    bool getAnchorPosOnCage(int lastSelected, Eigen::Vector3f& pos, Eigen::Vector3f ray, Eigen::Vector3f start)
    {
        return m_shape_cage.getAnchorPos(lastSelected, pos, ray, start);
    }

    bool getAnchorPosOnCtrlPt(int lastSelected, Eigen::Vector3f& pos, Eigen::Vector3f ray, Eigen::Vector3f start)
    {
        return m_shape_control_points.getAnchorPos(lastSelected, pos, ray, start);
    }
};

#endif // CAGE2D_H
