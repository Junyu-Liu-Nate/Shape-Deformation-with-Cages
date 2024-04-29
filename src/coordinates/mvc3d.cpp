#include "mvc3d.h"

MVC3D::MVC3D()
{

}

void MVC3D::constructMVC(const Vector3d& vertexPos, HalfEdgeMesh& cage) {
    //--- Initialize all coords as 0
    wCoords.resize(cage.vertices.size());
    std::fill(wCoords.begin(), wCoords.end(), 0.0);

    double epsilon = 1e-8; // Adjusted for double precision

    //--- Setup u for all vertices
    for (Vertex& vertex : cage.vertices) {
        vertex.mvc_d = (vertex.initialPosition - vertexPos).norm();
        // Check for proximity to epsilon to handle edge cases
        vertex.mvc_u = (vertex.initialPosition - vertexPos) / vertex.mvc_d;
    }

    //--- Calculate wCoords by iterating faces
    for (Face& face : cage.faces) {
        double h = 0.0;
        // Calculate l, theta, h
        for (HalfEdge* halfEdge : face.halfEdges) {
            halfEdge->vertex->mvc_l = (halfEdge->next->vertex->mvc_u - halfEdge->next->next->vertex->mvc_u).norm();
            halfEdge->vertex->mvc_theta = 2 * asin(halfEdge->vertex->mvc_l / 2);
            h += halfEdge->vertex->mvc_theta;
        }
        h = h / 2;

        // Deal with the situation of vertex lying on a face
        if (M_PI - h < epsilon) {
            for (HalfEdge* halfEdge : face.halfEdges) {
                wCoords.at(halfEdge->vertex->vertexIdx) = sin(halfEdge->vertex->mvc_theta) * halfEdge->next->vertex->mvc_l * halfEdge->next->next->vertex->mvc_l;
            }
        }

        // Assemble u matrix
        Matrix3d uMatrix; // Changed from Matrix3f to Matrix3d
        for (int matrix_i = 0; matrix_i < 3; matrix_i++) {
            uMatrix.col(matrix_i) = face.halfEdges[matrix_i]->vertex->mvc_u.cast<double>(); // Ensure casting to double
        }

        // Calculate c, s
        for (HalfEdge* halfEdge : face.halfEdges) {
            double nominator = 2 * sin(h) * sin(h - halfEdge->vertex->mvc_theta);
            double denominator = sin(halfEdge->next->vertex->mvc_theta) * sin(halfEdge->next->next->vertex->mvc_theta);
            halfEdge->vertex->mvc_c = nominator / denominator - 1;

            double clampedValue = max(0.0, 1 - pow(halfEdge->vertex->mvc_c, 2));
            halfEdge->vertex->mvc_s = copysign(1.0, uMatrix.determinant()) * sqrt(clampedValue);
        }

        for (HalfEdge* halfEdge : face.halfEdges) {
            if (abs(halfEdge->vertex->mvc_s) < epsilon) {
                continue;
            }
            double nominator = halfEdge->vertex->mvc_theta - halfEdge->next->vertex->mvc_c * halfEdge->next->next->vertex->mvc_theta - halfEdge->next->next->vertex->mvc_c * halfEdge->next->vertex->mvc_theta;
            double denominator = halfEdge->vertex->mvc_d * sin(halfEdge->next->vertex->mvc_theta) * halfEdge->next->next->vertex->mvc_s;
            if (denominator == 0) {
                continue;
            }
            wCoords.at(halfEdge->vertex->vertexIdx) += nominator / denominator;
        }
    }
}
