#ifndef HALFEDGE_H
#define HALFEDGE_H

#include <list>
#include <Eigen/Core>
#include <unordered_map>
#include <unordered_set>
#include <Eigen/SVD>

using namespace std;
using namespace Eigen;

struct HalfEdge;
struct Vertex;

struct HalfEdge {
    HalfEdge* twin;
    HalfEdge* next;
    Vertex* vertex;

    float weight = 0.0f;

    // Adding a constructor for convenience
    HalfEdge() : twin(nullptr), next(nullptr), vertex(nullptr) {}
};

struct Vertex {
    int vertexIdx;

    Eigen::Vector3f position;
    Eigen::Vector3f nextPosition;
    HalfEdge* halfEdge;

    Matrix3f R = Matrix3f::Identity();

    // Adding a constructor for convenience
    Vertex(const Eigen::Vector3f& pos) : position(pos), halfEdge(nullptr) {}
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

    void buildHalfEdgeStructure(const std::vector<Eigen::Vector3f>& _vertices,
                                const std::vector<Eigen::Vector3i>& _faces);

    void updateVertexPos(std::vector<Eigen::Vector3f>& outVertices);

    int vertexDegree(Vertex* vertex);

private:

};

#endif // HALFEDGE_H
