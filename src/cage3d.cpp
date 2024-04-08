#include "cage3d.h"
#include "graphics/meshloader.h"

#include <iostream>
#include <set>
#include <map>
#include <vector>

using namespace std;
using namespace Eigen;

Cage3D::Cage3D() {}

void Cage3D::init(Eigen::Vector3f &coeffMin, Eigen::Vector3f &coeffMax)
{
    vector<Vector3f> vertices;
    vector<Vector3i> triangles;

    //--- Read in cage
    if (MeshLoader::loadTriMesh("meshes/3d/bar_cage_3d.OBJ", vertices, triangles)) {
        m_shape_cage.init(vertices, triangles);
    }

    // Build halfedge structure for the cage
    std::cout << triangles.size() << std::endl;
    heMesh.buildHalfEdgeStructure(vertices, triangles);

    //--- Read in object
    vector<Vector3f> objectVertices;
    vector<Vector3i> objectTriangles;

    if (MeshLoader::loadTriMesh("meshes/3d/bar_model_3d.OBJ", objectVertices, objectTriangles)) {
        m_shape_object.init(objectVertices, objectTriangles);
    }

    buildVertexList(objectVertices);

    // Students, please don't touch this code: get min and max for viewport stuff
    MatrixX3f all_vertices = MatrixX3f(vertices.size(), 3);
    int i = 0;
    for (unsigned long i = 0; i < vertices.size(); ++i) {
        all_vertices.row(i) = vertices[i];
    }
    coeffMin = all_vertices.colwise().minCoeff();
    coeffMax = all_vertices.colwise().maxCoeff();
}

// Move an anchored vertex, defined by its index, to targetPosition
void Cage3D::move(int vertex, Vector3f targetPosition)
{
    std::vector<Eigen::Vector3f> new_vertices = m_shape_cage.getVertices();
    const std::unordered_set<int>& anchors = m_shape_cage.getAnchors();

    // Update cage vertex positions
    initialize(new_vertices, vertex, targetPosition);
    heMesh.updateVertexPos(new_vertices);

    // Update object vertex positions
    object3D.updateVertices(heMesh);
    std::vector<Eigen::Vector3f> new_object_vertices = object3D.getVertices();

    // Here are some helpful controls for the application
    //
    // - You start in first-person camera mode
    //   - WASD to move, left-click and drag to rotate
    //   - R and F to move vertically up and down
    //
    // - C to change to orbit camera mode
    //
    // - Right-click (and, optionally, drag) to anchor/unanchor points
    //   - Left-click an anchored point to move it around
    //
    // - Minus and equal keys (click repeatedly) to change the size of the vertices

    m_shape_cage.setVertices(new_vertices);
    std::cout << new_object_vertices.at(0) << std::endl << std::endl;
    m_shape_object.setVertices(new_object_vertices);
}

// Set the cage vertex position to target position
void Cage3D::initialize(std::vector<Eigen::Vector3f> new_vertices, int vertex, Vector3f targetPosition) {
    #pragma omp parallel for
    for (int i = 0; i < new_vertices.size(); i++) {
        if (i == vertex) {
            heMesh.vertices.at(i).nextPosition = targetPosition;
        }
        else {
            heMesh.vertices.at(i).nextPosition = new_vertices.at(i);
        }
    }
}

void Cage3D::buildVertexList(vector<Vector3f> objectVertices) {
    object3D.vertexList.resize(objectVertices.size());
    for (int i = 0; i < objectVertices.size(); i++) {
        ObjectVertex objectVertex;
        objectVertex.position = objectVertices.at(i);
        objectVertex.greenCord.constructGreenCoordinates(objectVertex.position, heMesh);

        object3D.vertexList.at(i) = objectVertex;
    }
}
