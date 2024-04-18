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

    //----- Read in cage
    if (MeshLoader::loadTriMesh("meshes/3d/simple/bar/bar_cage_complex.obj", vertices, triangles)) {
        m_shape_cage.init(vertices, triangles);
    }

    // Build halfedge structure for the cage
    heMesh.buildHalfEdgeStructure(vertices, triangles);
    heMesh.updateVertexPos(vertices);

    //----- Read in object
    vector<Vector3f> objectVertices;
    vector<Vector3i> objectTriangles;

    if (MeshLoader::loadTriMesh("meshes/3d/simple/bar/bar.obj", objectVertices, objectTriangles)) {
        m_shape_object.init(objectVertices, objectTriangles);
    }

    buildVertexList(objectVertices);

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
void Cage3D::move(int vertex, Vector3f targetPosition)
{
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
}

// Set the cage vertex position to target position
void Cage3D::updateCage(std::vector<Eigen::Vector3f> new_vertices, int vertex, Vector3f targetPosition) {
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

//---- Build the Green Coordinates for all vertices
void Cage3D::buildVertexList(vector<Vector3f> objectVertices) {
    object3D.vertexList.resize(objectVertices.size());
    #pragma omp parallel for
    for (int i = 0; i < objectVertices.size(); i++) {
        ObjectVertex objectVertex;
        objectVertex.position = objectVertices.at(i);

        // Build Green Coordinates
//        objectVertex.greenCord.constructGreenCoordinates(objectVertex.position, heMesh);
//        if (!isPointOutsideMesh(objectVertex.position, heMesh)) {
//            objectVertex.greenCord.constructGreenCoordinates(objectVertex.position, heMesh);
//        }
//        else {
//            objectVertex.greenCord.constructGreenCoordinatesExterior(objectVertex.position, heMesh);
//        }

        // Build MVC Coordinates
        objectVertex.mvcCoord.constructMVC(objectVertex.position, heMesh);

        object3D.vertexList.at(i) = objectVertex;
    }
}


//---- Check whether a vertex is outside of the cage
bool Cage3D::rayIntersectsTriangle(const Eigen::Vector3f& P, const Eigen::Vector3f& D, const Face& face) {
    const float EPSILON = 0.0000001f;
    Eigen::Vector3f vertex0 = face.halfEdges[0]->vertex->position;
    Eigen::Vector3f vertex1 = face.halfEdges[1]->vertex->position;
    Eigen::Vector3f vertex2 = face.halfEdges[2]->vertex->position;
    Eigen::Vector3f edge1, edge2, h, s, q;
    float a, f, u, v;
    edge1 = vertex1 - vertex0;
    edge2 = vertex2 - vertex0;
    h = D.cross(edge2);
    a = edge1.dot(h);
    if (a > -EPSILON && a < EPSILON)
        return false;    // This ray is parallel to this triangle.
    f = 1.0/a;
    s = P - vertex0;
    u = f * (s.dot(h));
    if (u < 0.0 || u > 1.0)
        return false;
    q = s.cross(edge1);
    v = f * D.dot(q);
    if (v < 0.0 || u + v > 1.0)
        return false;
    // At this point we know that there is a line intersection but not if it's within the segment
    float t = f * edge2.dot(q);
    if (t > EPSILON) // ray intersection
        return true;
    else // This means that there is a line intersection but not a ray intersection.
        return false;
}

bool Cage3D::isPointOutsideMesh(const Eigen::Vector3f& point, HalfEdgeMesh& mesh) {
    int intersections = 0;
    Eigen::Vector3f rayDir(1.0f, 0.0f, 0.0f); // Arbitrary direction

    for (const auto& face : mesh.faces) {
        if (rayIntersectsTriangle(point, rayDir, face)) {
            intersections++;
        }
    }

    return intersections % 2 == 0;
}
