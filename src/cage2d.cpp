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

    //----- Read in cage
    if (MeshLoader::loadTriMesh("meshes/3d/bar_cage_partial.obj", vertices, triangles)) {
        m_shape_cage.init(vertices, triangles);
    }

    // Build halfedge structure for the cage
    heMesh.buildHalfEdgeStructure(vertices, triangles);
    heMesh.updateVertexPos(vertices);

    //----- Read in object
    vector<Vector3f> objectVertices;
    vector<Vector3i> objectTriangles;

    if (MeshLoader::loadTriMesh("meshes/3d/bar.obj", objectVertices, objectTriangles)) {
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
    heMesh.updateVertexPos(new_vertices);

    // Update object vertex positions
    object2D.updateVertices();
    std::vector<Eigen::Vector3f> new_object_vertices = object2D.getVertices();

    m_shape_cage.setVertices(new_vertices);
    m_shape_object.setVertices(new_object_vertices);
}

// Set the cage vertex position to target position
void Cage2D::updateCage(std::vector<Eigen::Vector3f> new_vertices, int vertex, Vector3f targetPosition) {
#pragma omp parallel for
    for (int i = 0; i < new_vertices.size(); i++) {
        if (i == vertex) {
            heMesh.vertices.at(i).position = targetPosition;
        }
        else {
            heMesh.vertices.at(i).position = new_vertices.at(i);
        }
    }
}

void Cage2D::buildVertexList2D(vector<Vector3f> objectVertices) {
    object2D.vertexList.resize(objectVertices.size());
    for (int i = 0; i < objectVertices.size(); i++) {
        ObjectVertex2D objectVertex;
        objectVertex.position = Vector2f(objectVertices.at(i).x(), objectVertices.at(i).y());

        // Build 2D Green Coordinates
        objectVertex.greenCord.constructGreenCoordinates(objectVertex.position);

        object2D.vertexList.at(i) = objectVertex;
    }
}
