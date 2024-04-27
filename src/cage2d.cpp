#include "cage2d.h"
#include "graphics/meshloader.h"

#include <iostream>
#include <set>
#include <map>
#include <vector>

using namespace std;
using namespace Eigen;

Cage2D::Cage2D()
{

}

void Cage2D::init(Eigen::Vector3f &coeffMin, Eigen::Vector3f &coeffMax)
{
    vector<Vector3f> vertices;
    vector<Vector3i> triangles;
    vector<Vector3f> controlPts;

    //----- load in the cage. Later test with complex shapes
    if (MeshLoader::loadTriMesh(m_cageFilePath, vertices, triangles)) {
        m_shape_cage.init(vertices, triangles);
    }

    cagePoints.resize(vertices.size());
    for (int i = 0; i < vertices.size(); i++) {
        TwoDVertex twoDVertex;
        twoDVertex.idx = i;
        twoDVertex.position = Vector2f(vertices.at(i).x(), vertices.at(i).y());
    }

    findMarginEdges(triangles, vertices, controlPts);

    //----- Read in object
    vector<Vector3f> objectVertices;
    vector<Vector3i> objectTriangles;

    if (MeshLoader::loadTriMesh("meshes/2d/square.obj", objectVertices, objectTriangles)) {
        vector<Vector2f> uvCoords;
        tessellateMesh(objectTriangles, objectVertices, 20, 20, uvCoords); // DOUBLE CHECK THIS
        m_shape_object.initWithTexture(objectVertices, objectTriangles, uvCoords, m_textureFilePath);
    }

    buildVertexList2D(objectVertices);

    m_shape_control_points.init(controlPts, vector<Vector3i>()); // Setup rendering for control points
    // TODO: How to draw and control these vertices

    //----- Students, please don't touch this code: get min and max for viewport stuff
    MatrixX3f all_vertices = MatrixX3f(vertices.size(), 3);
    int i = 0;
    for (unsigned long i = 0; i < vertices.size(); ++i) {
        all_vertices.row(i) = vertices[i];
    }
    coeffMin = all_vertices.colwise().minCoeff();
    coeffMax = all_vertices.colwise().maxCoeff();
}

// Move an anchored vertex, defined by its index, to targetPosition
void Cage2D::move(int vertex, Vector3f targetPosition)
{
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

    // TODO: Add updates for Bezier curve control points
}

void Cage2D::moveAllAnchors(int vertex, Vector3f pos)
{
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

    // TODO: Add updates for Bezier curve control points
}

// Set the cage vertex position to target position
void Cage2D::updateCage(std::vector<Eigen::Vector3f>& new_vertices, int vertex, Vector3f targetPosition) {
    for (int i = 0; i < new_vertices.size(); i++) {
        if (i == vertex) {
            cagePoints.at(i).position = Vector2f(targetPosition.x(), targetPosition.y());
        }
        else {
            cagePoints.at(i).position = Vector2f(new_vertices.at(i).x(), new_vertices.at(i).y());
        }
        new_vertices.at(i) = Vector3f(cagePoints.at(i).position.x(), cagePoints.at(i).position.y(), 0);
    }
}

//---------- Build coordinates for all vertices ----------//
void Cage2D::buildVertexList2D(vector<Vector3f> objectVertices) {
    object2D.vertexList.resize(objectVertices.size());
    for (int i = 0; i < objectVertices.size(); i++) {
        ObjectVertex2D objectVertex;
        objectVertex.position = Vector2f(objectVertices.at(i).x(), objectVertices.at(i).y());

        // Build 2D Green Coordinates
        objectVertex.greenCord.constructGreenCoordinates(objectVertex.position, cagePoints, cageEdges);

        // Build 2D Higher Order Green Coordinates
        objectVertex.gcHigherOrder.constructGCHigherOrder(objectVertex.position, cagePoints, cageEdges);

        // Build 2D MVC Coordinates
        objectVertex.mvcCoord.constructMVC(objectVertex.position, cagePoints);

        object2D.vertexList.at(i) = objectVertex;
    }
}

//---------- Extract boundary edges from cage mesh ----------//
// Define a custom hash function in the correct namespace scope
struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1,T2>& pair) const {
        auto hash1 = std::hash<T1>{}(pair.first);
        auto hash2 = std::hash<T2>{}(pair.second);
        return hash1 ^ (hash2 << 1);  // Combine hashes
    }
};

void Cage2D::findMarginEdges(vector<Vector3i>& triangles, vector<Vector3f>& vertices, vector<Vector3f>& controlPts) {
    cagePoints.resize(vertices.size());
    for (int i = 0; i < vertices.size(); i++) {
        cagePoints[i].idx = i;
        cagePoints[i].position = Vector2f(vertices[i].x(), vertices[i].y());
    }

    unordered_map<pair<int, int>, int, pair_hash> edgeCount;
    unordered_map<pair<int, int>, pair<int, int>, pair_hash> edgeToVertices;

    for (const auto& tri : triangles) {
        vector<pair<int, int>> edges = {
            {tri[0], tri[1]},
            {tri[1], tri[2]},
            {tri[2], tri[0]}
        };

        for (auto& edge : edges) {
            auto sortedEdge = minmax(edge.first, edge.second);
            edgeCount[sortedEdge]++;
            if (edgeCount[sortedEdge] == 1) {
                edgeToVertices[sortedEdge] = edge;
            }
        }
    }

    for (const auto& e : edgeCount) {
        if (e.second == 1) {  // Boundary edge found
            auto verticesPair = edgeToVertices[e.first];
            TwoDEdge cageEdge;
            cageEdge.isMargin = true;
            cageEdge.edge = {&cagePoints[verticesPair.first], &cagePoints[verticesPair.second]};
            cageEdge.originalLength = (cagePoints[verticesPair.first].position - cagePoints[verticesPair.second].position).norm();
            cageEdges.push_back(cageEdge);

            // Mark vertices as margin
            cagePoints[verticesPair.first].isMargin = true;
            cagePoints[verticesPair.second].isMargin = true;

            int numControlPoints = max(0, degree - 1);
            for (int i = 0; i < numControlPoints; i++) {
                ControlPoint newControlPoint;
                newControlPoint.position = cagePoints[verticesPair.first].position + (i + 1) * (cagePoints[verticesPair.second].position - cagePoints[verticesPair.first].position) / degree;
                newControlPoint.idx = controlPoints.size();
                controlPts.push_back(Vector3f(newControlPoint.position.x(), newControlPoint.position.y(), 0));
                controlPoints[make_tuple(cagePoints[verticesPair.first].idx, cagePoints[verticesPair.second].idx, i + 1)] = newControlPoint;
            }

        }
    }
}

//---------- Tessellate object mesh ----------//
void Cage2D::tessellateMesh(vector<Vector3i>& faces, vector<Vector3f>& vertices, int rowNum, int colNum, vector<Vector2f> &uvCoords)
{
    // Initialize min and max coordinates with the first vertex to find the bounding box
    float minX = vertices[0].x();
    float maxX = vertices[0].x();
    float minY = vertices[0].y();
    float maxY = vertices[0].y();

    // Iterate through all vertices to find min and max coordinates
    for (const auto& vert : vertices) {
        if (vert.x() < minX) minX = vert.x();
        if (vert.x() > maxX) maxX = vert.x();
        if (vert.y() < minY) minY = vert.y();
        if (vert.y() > maxY) maxY = vert.y();
    }

    // compute the tessellated edge width and height
    float stepX = (maxX - minX) / (float)colNum;
    float stepY = (maxY - minY) / (float)rowNum;

    vertices.clear();
    // Generate vertices
    for (int c = 0; c <= colNum; c++) {
        for (int r = 0; r <= rowNum; r++) {
            float x = minX + c * stepX;
            float y = minY + r * stepY;
            vertices.push_back(Vector3f(x, y, 0.0f));
            uvCoords.push_back(Vector2f(1.f - float(c) / colNum, 1.f - float(r) / rowNum));
        }
    }

    faces.clear();
    // Generate triangles
    for (int r = 0; r < rowNum; r++) {
        for (int c = 0; c < colNum; c++) {
            int index0 = r * (colNum + 1) + c;
            int index1 = index0 + 1;
            int index2 = (r + 1) * (colNum + 1) + c;
            int index3 = index2 + 1;

            // First triangle
            faces.push_back(Vector3i(index0, index2, index1));
            faces.push_back(Vector3i(index2, index0, index1));

            // Second triangle
            faces.push_back(Vector3i(index1, index2, index3));
            faces.push_back(Vector3i(index2, index1, index3));
        }
    }
}

//---------- Texture related ----------//
void Cage2D::setTextureFilePath(const QString &path)
{
    m_textureFilePath = path.toStdString();
}

bool Cage2D::isTextureFilePathSet()
{
    if (m_textureFilePath.empty()) {
        return false;
    }
    return true;
}

void Cage2D::setCageFilePath(const QString &path)
{
    m_cageFilePath = path.toStdString();
}

bool Cage2D::isCageFilePathSet()
{
    if (m_cageFilePath.empty()) {
        return false;
    }
    return true;
}
