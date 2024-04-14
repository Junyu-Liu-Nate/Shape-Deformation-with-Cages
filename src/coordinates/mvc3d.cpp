#include "mvc3d.h"

MVC3D::MVC3D()
{

}

void MVC3D::constructMVC(const Vector3f& vertexPos, HalfEdgeMesh& cage) {
    //--- Initialize all coords as 0
    wCoords.resize(cage.vertices.size());
    std::fill(wCoords.begin(), wCoords.end(), 0.0f);

    float epsilon = 0.0f; // TODO: Check how to set this!!!!!!!!!

    //--- Setup u for all vertices
    for (Vertex& vertex : cage.vertices) {
        vertex.mvc_d = (vertex.initialPosition - vertexPos).norm();
        // TODO: if dj < ε return f j !!!!!!!!!
        vertex.mvc_u = (vertex.initialPosition - vertexPos) / vertex.mvc_d;
    }

    //--- Calculate wCoords by iterating faces
    for (Face& face : cage.faces) {
        float h = 0.0f;
        // Calculate l, theta, h
        for (HalfEdge* halfEdge : face.halfEdges) {
            halfEdge->vertex->mvc_l = (halfEdge->next->vertex->mvc_u - halfEdge->next->next->vertex->mvc_u).norm();
            halfEdge->vertex->mvc_theta = 2 * asin(halfEdge->vertex->mvc_l / 2);
            h += halfEdge->vertex->mvc_theta;
        }
        h = h / 2;

        // Deal with the situaton of vertex lying on a face
        if (M_PI - h < epsilon) {
            for (HalfEdge* halfEdge : face.halfEdges) {
                wCoords.at(halfEdge->vertex->vertexIdx) = sin(halfEdge->vertex->mvc_theta) * halfEdge->next->vertex->mvc_l * halfEdge->next->next->vertex->mvc_l;
            }
        }

        // Assemble u matrix
        Matrix3f uMatrix;
        for (int matrix_i = 0; matrix_i < 3; matrix_i++) {
            uMatrix.col(matrix_i) = face.halfEdges[matrix_i]->vertex->mvc_u;
        }

        // Calculate c, s
        for (HalfEdge* halfEdge : face.halfEdges) {
            float nominator = 2 * sin(h) * sin(h - halfEdge->vertex->mvc_theta);
            float denominator = sin(halfEdge->next->vertex->mvc_theta) * sin(halfEdge->next->next->vertex->mvc_theta);
            halfEdge->vertex->mvc_c = nominator / denominator - 1;

            halfEdge->vertex->mvc_s = copysign(1.0, uMatrix.determinant()) * sqrt(1 - pow(halfEdge->vertex->mvc_c, 2));
        }

        for (HalfEdge* halfEdge : face.halfEdges) {
            if (abs(halfEdge->vertex->mvc_s) < epsilon) {
                continue;
            }
            float nominator = halfEdge->vertex->mvc_theta - halfEdge->next->vertex->mvc_c * halfEdge->next->next->vertex->mvc_theta - halfEdge->next->next->vertex->mvc_c * halfEdge->next->vertex->mvc_theta;
            float denominator = halfEdge->vertex->mvc_d * sin(halfEdge->next->vertex->mvc_theta) * halfEdge->next->next->vertex->mvc_s;
            wCoords.at(halfEdge->vertex->vertexIdx) += nominator / denominator;
        }
    }
}
