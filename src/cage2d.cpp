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

    //--- Hardcode cage
    // cage points
    cagePoints.resize(4);
    cagePoints.clear();
    cagePoints.push_back(Vector2f(1, -1));
    cagePoints.push_back(Vector2f(1, 1));
    cagePoints.push_back(Vector2f(-1, 1));
    cagePoints.push_back(Vector2f(-1, -1));

    // cage edges
    cageEdges.resize(4);
    cageEdges.clear();
    for(int i = 0; i < cagePoints.size(); ++i) {
        std::pair<Vector2f, Vector2f> edge;

        // Last position, do the first minus the last
        if (i == cagePoints.size() - 1) {
            edge.first = cagePoints[i];
            edge.second = cagePoints[0];
        }

        // Store the value of the next element minus the current element
        else {
            edge.first = cagePoints[i];
            edge.second = cagePoints[i + 1];
        }
        cageEdges.push_back(edge);
    }

    // cage mesh - used for rendering
    vertices.clear();
    vertices.push_back(Vector3f(1, -1, 0));
    vertices.push_back(Vector3f(1, 1, 0));
    vertices.push_back(Vector3f(-1, 1, 0));
    vertices.push_back(Vector3f(-1, -1, 0));
    triangles.clear();
    triangles.push_back(Vector3i(0,1,2));
    triangles.push_back(Vector3i(2,3,0));
    triangles.push_back(Vector3i(1,0,2));
    triangles.push_back(Vector3i(2,0,3));

    m_shape_cage.init(vertices, triangles);

    //----- Read in object
    vector<Vector3f> objectVertices;
    vector<Vector3i> objectTriangles;

    if (MeshLoader::loadTriMesh("meshes/2d/rectangle.obj", objectVertices, objectTriangles)) {
        m_shape_object.init(objectVertices, objectTriangles);
    }

    buildVertexList2D(objectVertices);

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
    object2D.updateVertices(cagePoints, cageEdges);
    std::vector<Eigen::Vector3f> new_object_vertices = object2D.getVertices();

    m_shape_cage.setVertices(new_vertices);
    m_shape_object.setVertices(new_object_vertices);
}

// Set the cage vertex position to target position
void Cage2D::updateCage(std::vector<Eigen::Vector3f>& new_vertices, int vertex, Vector3f targetPosition) {
    for (int i = 0; i < new_vertices.size(); i++) {
        if (i == vertex) {
            cagePoints.at(i) = Vector2f(targetPosition.x(), targetPosition.y());
        }
        else {
            cagePoints.at(i) = Vector2f(new_vertices.at(i).x(), new_vertices.at(i).y());

        }
        new_vertices.at(i) = Vector3f(cagePoints.at(i).x(), cagePoints.at(i).y(), 0);
    }
}

// Build coordinates for all vertices
void Cage2D::buildVertexList2D(vector<Vector3f> objectVertices) {
    object2D.vertexList.resize(objectVertices.size());
    for (int i = 0; i < objectVertices.size(); i++) {
        ObjectVertex2D objectVertex;
        objectVertex.position = Vector2f(objectVertices.at(i).x(), objectVertices.at(i).y());

        // Build 2D Green Coordinates
        objectVertex.greenCord.constructGreenCoordinates(objectVertex.position, cagePoints, cageEdges);

        object2D.vertexList.at(i) = objectVertex;
    }
}
