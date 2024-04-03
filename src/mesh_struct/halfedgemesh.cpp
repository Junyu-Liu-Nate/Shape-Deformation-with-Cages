#include "halfedgemesh.h"

HalfEdgeMesh::HalfEdgeMesh() {

}

void HalfEdgeMesh::buildHalfEdgeStructure(const std::vector<Eigen::Vector3f>& _vertices,
                                          const std::vector<Eigen::Vector3i>& _faces) {
    // Create vertices and store the index within them
    vertices.clear();
    vertices.reserve(_vertices.size());
    int idx = 0;
    for (const auto& pos : _vertices) {
        vertices.emplace_back(pos);
        vertices.back().vertexIdx = idx++;  // Store the index
    }

    // Create a map for edge lookup
    std::unordered_map<std::pair<int, int>, HalfEdge*, pair_hash> edgeMap;

    for (const auto& face : _faces) {
        std::array<HalfEdge*, 3> faceHalfEdges;

        // Create halfEdges for each edge of the face
        for (int i = 0; i < 3; ++i) {
            int vStart = face[i];
            int vEnd = face[(i + 1) % 3];

            halfEdges.emplace_back();
            HalfEdge* newHalfEdge = &halfEdges.back();

            // Link the halfEdge to the start vertex
            newHalfEdge->vertex = &vertices[vStart];

            // Set the vertex's halfEdge if it's not set yet
            if (vertices[vStart].halfEdge == nullptr) {
                vertices[vStart].halfEdge = newHalfEdge;
            }

            // Store the halfEdge in the array
            faceHalfEdges[i] = newHalfEdge;

            // Add to the edgeMap for easy lookup of the twin halfEdge
            edgeMap[{vStart, vEnd}] = newHalfEdge;
        }

        // Link the next halfEdges to form the face loop
        for (int i = 0; i < 3; ++i) {
            faceHalfEdges[i]->next = faceHalfEdges[(i + 1) % 3];
        }
    }

    // Set up twin halfEdges
    for (auto& [edgeKey, halfEdge] : edgeMap) {
        int vStart = edgeKey.first;
        int vEnd = edgeKey.second;

        auto it = edgeMap.find({vEnd, vStart});
        if (it != edgeMap.end()) {
            HalfEdge* twinHalfEdge = it->second;
            halfEdge->twin = twinHalfEdge;
            twinHalfEdge->twin = halfEdge;
        }
    }
}

void HalfEdgeMesh::updateVertexPos(std::vector<Eigen::Vector3f>& outVertices) {
    for (Vertex& vertex : vertices) {
        outVertices[vertex.vertexIdx] = vertex.nextPosition;
        vertex.position = vertex.nextPosition;
    }
}

int HalfEdgeMesh::vertexDegree(Vertex* vertex) {
    HalfEdge* startEdge = vertex->halfEdge;
    HalfEdge* edge = startEdge;
    int degree = 0;
    do {
        degree++;
        edge = edge->twin->next;
    } while (edge && edge != startEdge);

    return degree;
}
