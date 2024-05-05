#include "cage3d.h"
#include "graphics/meshloader.h"

#include <iostream>
#include <set>
#include <map>
#include <vector>

using namespace std;
using namespace Eigen;

Cage3D::Cage3D(bool useGreen) :
    object3D(useGreen)
{
    m_useGreen = useGreen;
}

void Cage3D::init(Eigen::Vector3f &coeffMin, Eigen::Vector3f &coeffMax)
{
    vector<Vector3f> vertices;
    vector<Vector3i> triangles;

    //----- Read in cage
    if (MeshLoader::loadTriMesh(m_cageFilePath, vertices, triangles)) {
        m_shape_cage.init(vertices, triangles);
    }

    // Create a new vector of Vector3d
    std::vector<Eigen::Vector3d> verticesD;
    verticesD.reserve(vertices.size());
    // Convert each Vector3f to Vector3d and add to the new vector
    for (const auto& v : vertices) {
        verticesD.push_back(v.cast<double>());
    }

    // Build halfedge structure for the cage
    heMesh.buildHalfEdgeStructure(verticesD, triangles);
    heMesh.updateVertexPos(vertices);

    //----- Read in object
    vector<Vector3f> objectVertices;
    vector<Vector3i> objectTriangles;

    if (MeshLoader::loadTriMesh(m_objectFilePath, objectVertices, objectTriangles)) {
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

void Cage3D::moveAllAnchors(int vertex, Vector3f pos)
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
            heMesh.vertices.at(i).position = targetPosition.cast<double>();
        }
        else {
            heMesh.vertices.at(i).position = new_vertices.at(i).cast<double>();
        }
    }
}

//---- Build the Green Coordinates for all vertices
void Cage3D::buildVertexList(vector<Vector3f> objectVertices) {
    object3D.vertexList.resize(objectVertices.size());

    for (int i = 0; i < objectVertices.size(); i++) {
        ObjectVertex objectVertex;
        objectVertex.position = objectVertices.at(i);

        if (m_useGreen) {
            // If not consider boundary cases
            // Build Green Coordinates
            if (!isPointOutsideMesh(objectVertex.position.cast<double>(), heMesh)) {
                objectVertex.greenCord.constructGreenCoordinates(objectVertex.position.cast<double>(), heMesh);
            } else {
                objectVertex.greenCord.constructGreenCoordinatesExterior(objectVertex.position.cast<double>(), heMesh);
            }
        } else {
            // If consider boundary cases
            if (isPointOnBoundary(objectVertex.position.cast<double>(), heMesh)) {
                objectVertex.greenCord.constructGreenCoordinatesBoundary(objectVertex.position.cast<double>(), heMesh);
            }
            else {
                if (!isPointOutsideMesh(objectVertex.position.cast<double>(), heMesh)) {
                    objectVertex.greenCord.constructGreenCoordinates(objectVertex.position.cast<double>(), heMesh);
                }
                else {
                    objectVertex.greenCord.constructGreenCoordinatesExterior(objectVertex.position.cast<double>(), heMesh);
                }
            }
            // Build MVC Coordinates
            objectVertex.mvcCoord.constructMVC(objectVertex.position.cast<double>(), heMesh);
        }

        object3D.vertexList.at(i) = objectVertex;
    }
}


//---- Check whether a vertex is outside of the cage
bool Cage3D::rayIntersectsTriangle(const Eigen::Vector3d& P, const Eigen::Vector3d& D, const Face& face) {
    const double EPSILON = 0.0000001;
    Eigen::Vector3d vertex0 = face.halfEdges[0]->vertex->position.cast<double>();
    Eigen::Vector3d vertex1 = face.halfEdges[1]->vertex->position.cast<double>();
    Eigen::Vector3d vertex2 = face.halfEdges[2]->vertex->position.cast<double>();
    Eigen::Vector3d edge1, edge2, h, s, q;
    double a, f, u, v;
    edge1 = vertex1 - vertex0;
    edge2 = vertex2 - vertex0;
    h = D.cross(edge2);
    a = edge1.dot(h);
    if (a > -EPSILON && a < EPSILON)
        return false;    // This ray is parallel to this triangle.
    f = 1.0 / a;
    s = P - vertex0;
    u = f * (s.dot(h));
    if (u < 0.0 || u > 1.0)
        return false;
    q = s.cross(edge1);
    v = f * D.dot(q);
    if (v < 0.0 || u + v > 1.0)
        return false;
    // At this point we know that there is a line intersection but not if it's within the segment
    double t = f * edge2.dot(q);
    if (t > EPSILON) // ray intersection
        return true;
    else // This means that there is a line intersection but not a ray intersection.
        return false;
}

bool Cage3D::isPointOutsideMesh(const Eigen::Vector3d& point, HalfEdgeMesh& mesh) {
    int intersections = 0;
    Eigen::Vector3d rayDir(1.0, 0.0, 0.0); // Arbitrary direction

    for (const auto& face : mesh.faces) {
        if (rayIntersectsTriangle(point, rayDir, face)) {
            intersections++;
        }
    }

    return intersections % 2 == 0;
}

bool Cage3D::isPointOnBoundary(const Eigen::Vector3d& point, HalfEdgeMesh& mesh) {
    double minDistance = DBL_MAX;
    Vertex* closestVertex = nullptr;

    // Step 1: Find the closest vertex
    for (auto& vertex : mesh.vertices) {
        double dist = (vertex.position - point).squaredNorm();
        if (dist < minDistance) {
            minDistance = dist;
            closestVertex = &vertex;
        }
    }

    // Step 2: Check if the closest vertex is on a boundary
    if (closestVertex) {
        HalfEdge* startEdge = closestVertex->halfEdge;
        HalfEdge* edge = startEdge;

        do {
            if (!edge->twin) {
                // If twin is nullptr, then it's a boundary edge
                return true;
            }
            edge = edge->twin->next;
        } while (edge != startEdge);
    }

    return false;
}

void Cage3D::setObjectFilePath(const QString &path)
{
    m_objectFilePath = path.toStdString();
}

bool Cage3D::isObjectFilePathSet()
{
    if (m_objectFilePath.empty()) {
        return false;
    }
    return true;
}

void Cage3D::setCageFilePath(const QString &path)
{
    m_cageFilePath = path.toStdString();
}

bool Cage3D::isCageFilePathSet()
{
    if (m_cageFilePath.empty()) {
        return false;
    }
    return true;
}
