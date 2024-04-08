#include "greencoordinates3d.h"

GreenCoordinates3D::GreenCoordinates3D()
{

}

void GreenCoordinates3D::constructGreenCoordinates(const Vector3f& vertexPos, HalfEdgeMesh& cage) {
    //--- Initialize all coords as 0
    phiCoords.resize(cage.vertices.size());
    std::fill(phiCoords.begin(), phiCoords.end(), 0.0f);
    psiCoords.resize(cage.faces.size());
    std::fill(psiCoords.begin(), psiCoords.end(), 0.0f);

    //--- Calculate coords by iterating all faces
    int j = 0;
//    std::cout << cage.faces.size() << std::endl;
    for (Face& face : cage.faces) {
        for (HalfEdge* halfEdge : face.halfEdges) {
            halfEdge->vertex->calculatePosition = halfEdge->vertex->position - vertexPos;
//            std::cout << halfEdge->vertex->vertexIdx << "(original): " << halfEdge->vertex->position.x() << ", " << halfEdge->vertex->position.y() << ", " << halfEdge->vertex->position.z() << std::endl;
//            std::cout << halfEdge->vertex->vertexIdx << "(calculate): " << halfEdge->vertex->calculatePosition.x() << ", " << halfEdge->vertex->calculatePosition.y() << ", " << halfEdge->vertex->calculatePosition.z() << std::endl;
        }
//        std::cout << std::endl;
        Vector3f faceNormal = calculateFaceNormal(face);
        Vector3f p = (face.halfEdges[0]->vertex->calculatePosition.dot(faceNormal)) * faceNormal;

        for (HalfEdge* halfEdge : face.halfEdges) {
            Vertex* thisVertex = halfEdge->vertex;
            Vertex* nextVertex = halfEdge->next->vertex;
            thisVertex->s = copysign(1.0, (thisVertex->calculatePosition - p).cross(nextVertex->calculatePosition - p).dot(faceNormal));
            thisVertex->I = gcTriInt(p, thisVertex->calculatePosition, nextVertex->calculatePosition, Vector3f(0,0,0));
            thisVertex->II = gcTriInt(Vector3f(0,0,0), nextVertex->calculatePosition, thisVertex->calculatePosition, Vector3f(0,0,0));
            thisVertex->q = nextVertex->calculatePosition.cross(thisVertex->calculatePosition);
            thisVertex->N = thisVertex->q / thisVertex->q.norm();
        }

        float ISum = 0;
        for (HalfEdge* halfEdge : face.halfEdges) {
            ISum += halfEdge->vertex->s * halfEdge->vertex->I;
        }
        float I = -abs(ISum);
        psiCoords.at(j) = -I;
//        if (isnan(psiCoords.at(j))) {
//            std::cout << "nan for psi" << std::endl;
//        }

        Vector3f omega = faceNormal * I;
        for (HalfEdge* halfEdge : face.halfEdges) {
            omega += halfEdge->vertex->N * I * halfEdge->vertex->I;
        }
        float epsilon = 0.0; // TODO: Check how to set this
        if (omega.norm() > epsilon) {
            for (HalfEdge* halfEdge : face.halfEdges) {
                int vertexIdx = halfEdge->vertex->vertexIdx;
                phiCoords.at(vertexIdx) += (halfEdge->next->vertex->N.dot(omega)) / (halfEdge->next->vertex->N.dot(halfEdge->vertex->calculatePosition));
            }
        }

        j++;
    }
}

//float GreenCoordinates3D::gcTriInt(Vector3f p, Vector3f v1, Vector3f v2, Vector3f eta) {
//    //--- Calculate alpha
//    float alphaNominator = (v2 - v1).dot(p - v1);
//    float alphaDenominator = (v2 - v1).norm() * (p - v1).norm();
//    float alpha = acos(alphaNominator / alphaDenominator);

//    //--- Calculate beta
//    float betaNominator = (v1 - p).dot(v2 - p);
//    float betaDenominator = (v1 - p).norm() * (v2 - p).norm();
//    float beta = acos(betaNominator / betaDenominator);

//    //--- Calculate lambda
//    float lambda = (p - v1).norm() * (p - v1).norm() * sin(alpha) * sin(alpha);

//    //--- Calculate c
//    float c = (p - eta).norm() * (p - eta).norm();

//    std::vector<float> thetaList(2, 0.0f);
//    thetaList.at(0) = M_PI - alpha;
//    thetaList.at(1) = M_PI - alpha - beta;
//    std::vector<float> IList(2, 0.0f);
//    for (int i = 0; i < 2; i++) {
//        float theta = thetaList.at(i);
//        float S = sin(theta);
//        float C = cos(theta);

//        float term1 = -copysign(1.0, S) * 0.5;
//        float term2 = 2 * sqrt(c) * atan((sqrt(c) * C) / sqrt(lambda + S*S*c));
//        float term3a = (2 * sqrt(lambda) * S * S) / ((1 - C) * (1 - C));
//        float term3b = 1 - 2*c*C / (c * (1+C) + lambda + sqrt(lambda*lambda + lambda*c*S*S));
//        float term3 = sqrt(lambda) * log(term3a * term3b);

//        IList.at(i) = term1 * (term2 + term3);
//    }

//    float result = -1 / (4*M_PI) * abs(IList.at(0) - IList.at(1) - sqrt(c) * beta);

//    return result;
//}

float GreenCoordinates3D::gcTriInt(Vector3f p, Vector3f v1, Vector3f v2, Vector3f eta) {
    // Calculate alpha
    float alphaNominator = (v2 - v1).dot(p - v1);
    float alphaDenominator = (v2 - v1).norm() * (p - v1).norm();
    if (std::abs(alphaDenominator) < 1e-8) {  // Prevent division by zero
        std::cout << "alphaDenominator is zero" << std::endl;
        return NAN; // or handle more gracefully
//        std::exit(EXIT_FAILURE);
    }
    float alpha = acos(alphaNominator / alphaDenominator);
    if (std::isnan(alpha)) {
        std::cout << "alpha is NaN" << std::endl;
    }

    // Calculate beta
    float betaNominator = (v1 - p).dot(v2 - p);
    float betaDenominator = (v1 - p).norm() * (v2 - p).norm();
    if (std::abs(betaDenominator) < 1e-8) {  // Prevent division by zero
        std::cout << "betaDenominator is zero" << std::endl;
        return NAN; // or handle more gracefully
    }
    float beta = acos(betaNominator / betaDenominator);
    if (std::isnan(beta)) {
        std::cout << "beta is NaN" << std::endl;
    }

    // Calculate lambda
    float lambda = (p - v1).norm() * (p - v1).norm() * sin(alpha) * sin(alpha);
    if (lambda < 0) {
        std::cout << "lambda is negative" << std::endl;
    }

    // Calculate c
    float c = (p - eta).norm() * (p - eta).norm();
    if (c < 0) {
        std::cout << "c is negative" << std::endl;
    }

    std::vector<float> thetaList(2, 0.0f);
    thetaList.at(0) = M_PI - alpha;
    thetaList.at(1) = M_PI - alpha - beta;
    std::vector<float> IList(2, 0.0f);
    for (int i = 0; i < 2; i++) {
        float theta = thetaList.at(i);
        float S = sin(theta);
        float C = cos(theta);

        float term1 = -copysign(1.0, S) * 0.5;

        if (std::abs(lambda + S*S*c) < 1e-8) {
            std::cout << "Division by zero in term2 calculation" << std::endl;
            return NAN; // or handle more gracefully
        }
        float term2 = 2 * sqrt(c) * atan((sqrt(c) * C) / sqrt(lambda + S*S*c));
        if (std::isnan(term2)) {
            std::cout << "term2 is NaN" << std::endl;
        }

        float term3a = (2 * sqrt(lambda) * S * S) / ((1 - C) * (1 - C));
        float term3b = 1 - 2*c*C / (c * (1+C) + lambda + sqrt(lambda*lambda + lambda*c*S*S));
        if (term3a <= 0 || term3b <= 0) {
            std::cout << "Non-positive argument for log in term3 calculation" << std::endl;
            return NAN; // or handle more gracefully
        }
        float term3 = sqrt(lambda) * log(term3a * term3b);
        if (std::isnan(term3)) {
            std::cout << "term3 is NaN" << std::endl;
        }

        IList.at(i) = term1 * (term2 + term3);
        if (std::isnan(IList.at(i))) {
            std::cout << "IList[" << i << "] is NaN" << std::endl;
        }
    }

    float result = -1 / (4*M_PI) * abs(IList.at(0) - IList.at(1) - sqrt(c) * beta);
    if (std::isnan(result)) {
        std::cout << "Result is NaN" << std::endl;
    }

    return result;
}


Eigen::Vector3f GreenCoordinates3D::calculateFaceNormal(const Face& face) {
    // Ensure valid half-edge pointers
    if (!face.halfEdges[0] || !face.halfEdges[1] || !face.halfEdges[2]) {
        throw std::runtime_error("Face has invalid half-edges.");
    }

    Eigen::Vector3f edgeVec1 = face.halfEdges[1]->vertex->position - face.halfEdges[0]->vertex->position;
    Eigen::Vector3f edgeVec2 = face.halfEdges[2]->vertex->position - face.halfEdges[0]->vertex->position;

    Eigen::Vector3f normal = edgeVec1.cross(edgeVec2);

    normal.normalize();

    return normal;
}
