//#ifndef HALFEDGE_H
//#define HALFEDGE_H

//#include <list>
//#include <Eigen/Core>
//#include <unordered_map>
//#include <unordered_set>
//#include <iostream>
//#include <Eigen/SVD>

//using namespace std;
//using namespace Eigen;

//struct HalfEdge;
//struct Vertex;
//struct Face;

//struct HalfEdge {
//    HalfEdge* twin;
//    HalfEdge* next;
//    Vertex* vertex;

//    // Constructor
//    HalfEdge() : twin(nullptr), next(nullptr), vertex(nullptr) {}
//};

//struct Vertex {
//    int vertexIdx;

//    Eigen::Vector3f position; // Actual position
//    Eigen::Vector3f initialPosition;

//    HalfEdge* halfEdge;

//    // Variables used for calculating Green Coordinates (For Cages)
//    Eigen::Vector3f calculatePosition;
//    float s;
//    float I;
//    float II;
//    Vector3f q;
//    Vector3f N;

//    // Variables used for calculating MVC
//    float mvc_d;
//    Vector3f mvc_u;
//    float mvc_l;
//    float mvc_theta;
//    float mvc_c;
//    float mvc_s;

//    // Constructor
//    Vertex(const Eigen::Vector3f& pos) : position(pos), halfEdge(nullptr) {}
//};

//struct Face {
//    HalfEdge* halfEdges[3];  // Array to store pointers to the face's half-edges

//    // Constructor
//    Face() : halfEdges{nullptr, nullptr, nullptr} {}

//    Eigen::Vector3f calculateNormal() const {
//        if (halfEdges[0] == nullptr || halfEdges[1] == nullptr || halfEdges[2] == nullptr) {
//            // Invalid face, return zero vector as placeholder
//            return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
//        }
//        // Obtain the position of the vertices
//        Eigen::Vector3f p0 = halfEdges[0]->vertex->position;
//        Eigen::Vector3f p1 = halfEdges[1]->vertex->position;
//        Eigen::Vector3f p2 = halfEdges[2]->vertex->position;

//        // Compute two vectors on the face
//        Eigen::Vector3f v1 = p1 - p0;
//        Eigen::Vector3f v2 = p2 - p0;

//        // Compute the cross product to get the face normal
//        Eigen::Vector3f normal = v1.cross(v2);

//        // Normalize the normal to ensure it's a unit vector
//        normal.normalize();

//        return normal;
//    }

//    float calculateArea() const {
//        if (halfEdges[0] == nullptr || halfEdges[1] == nullptr || halfEdges[2] == nullptr) {
//            // Invalid face, return 0 as a placeholder
//            return 0.0f;
//        }
//        // Obtain the position of the vertices
//        Eigen::Vector3f p0 = halfEdges[0]->vertex->position;
//        Eigen::Vector3f p1 = halfEdges[1]->vertex->position;
//        Eigen::Vector3f p2 = halfEdges[2]->vertex->position;

//        // Compute two vectors on the face
//        Eigen::Vector3f v1 = p1 - p0;
//        Eigen::Vector3f v2 = p2 - p0;

//        // Compute the cross product to get the area of the parallelogram
//        Eigen::Vector3f crossProduct = v1.cross(v2);

//        // The area of the triangle is half the magnitude of the cross product
//        float area = 0.5f * crossProduct.norm();

//        return area;
//    }
//};

//struct pair_hash {
//    template <class T1, class T2>
//    std::size_t operator () (const std::pair<T1, T2>& pair) const {
//        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
//    }
//};

//class HalfEdgeMesh {
//public:
//    HalfEdgeMesh();

//    std::vector<Vertex> vertices;
//    std::list<HalfEdge> halfEdges;
//    std::vector<Face> faces;

//    void buildHalfEdgeStructure(const std::vector<Eigen::Vector3f>& _vertices,
//                                const std::vector<Eigen::Vector3i>& _faces);

//    void updateVertexPos(std::vector<Eigen::Vector3f>& outVertices);

//    int vertexDegree(Vertex* vertex);

//private:

//};

//#endif // HALFEDGE_H

#ifndef HALFEDGE_H
#define HALFEDGE_H

#include <list>
#include <Eigen/Core>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <Eigen/SVD>

using namespace std;
using namespace Eigen;

struct HalfEdge;
struct Vertex;
struct Face;

struct HalfEdge {
    HalfEdge* twin;
    HalfEdge* next;
    Vertex* vertex;

    // Constructor
    HalfEdge() : twin(nullptr), next(nullptr), vertex(nullptr) {}
};

struct Vertex {
    int vertexIdx;

    Eigen::Vector3d position; // Actual position using double instead of float
    Eigen::Vector3d initialPosition;

    HalfEdge* halfEdge;

    // Variables used for calculating Green Coordinates (For Cages)
    Eigen::Vector3d calculatePosition;
    double s;
    double I;
    double II;
    Vector3d q;
    Vector3d N;

    // Variables used for calculating MVC
    double mvc_d;
    Vector3d mvc_u;
    double mvc_l;
    double mvc_theta;
    double mvc_c;
    double mvc_s;

    // Constructor
    Vertex(const Eigen::Vector3d& pos) : position(pos), halfEdge(nullptr) {}
};

struct Face {
    HalfEdge* halfEdges[3];  // Array to store pointers to the face's half-edges

    // Constructor
    Face() : halfEdges{nullptr, nullptr, nullptr} {}

    Eigen::Vector3d calculateNormal() const {
        if (halfEdges[0] == nullptr || halfEdges[1] == nullptr || halfEdges[2] == nullptr) {
            // Invalid face, return zero vector as placeholder
            return Eigen::Vector3d(0.0, 0.0, 0.0);
        }
        // Obtain the position of the vertices
        Eigen::Vector3d p0 = halfEdges[0]->vertex->position;
        Eigen::Vector3d p1 = halfEdges[1]->vertex->position;
        Eigen::Vector3d p2 = halfEdges[2]->vertex->position;

        // Compute two vectors on the face
        Eigen::Vector3d v1 = p1 - p0;
        Eigen::Vector3d v2 = p2 - p0;

        // Compute the cross product to get the face normal
        Eigen::Vector3d normal = v1.cross(v2);

        // Normalize the normal to ensure it's a unit vector
        normal.normalize();

        return normal;
    }

    double calculateArea() const {
        if (halfEdges[0] == nullptr || halfEdges[1] == nullptr || halfEdges[2] == nullptr) {
            // Invalid face, return 0 as a placeholder
            return 0.0;
        }
        // Obtain the position of the vertices
        Eigen::Vector3d p0 = halfEdges[0]->vertex->position;
        Eigen::Vector3d p1 = halfEdges[1]->vertex->position;
        Eigen::Vector3d p2 = halfEdges[2]->vertex->position;

        // Compute two vectors on the face
        Eigen::Vector3d v1 = p1 - p0;
        Eigen::Vector3d v2 = p2 - p0;

        // Compute the cross product to get the area of the parallelogram
        Eigen::Vector3d crossProduct = v1.cross(v2);

        // The area of the triangle is half the magnitude of the cross product
        double area = 0.5 * crossProduct.norm();

        return area;
    }
};

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

class HalfEdgeMesh {
public:
    HalfEdgeMesh();

    std::vector<Vertex> vertices;
    std::list<HalfEdge> halfEdges;
    std::vector<Face> faces;

    void buildHalfEdgeStructure(const std::vector<Eigen::Vector3d>& _vertices,
                                const std::vector<Eigen::Vector3i>& _faces);

    void updateVertexPos(std::vector<Eigen::Vector3f>& outVertices);

    int vertexDegree(Vertex* vertex);

private:

};

#endif // HALFEDGE_H
