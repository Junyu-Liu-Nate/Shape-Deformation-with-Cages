#include "greencoordinates2d.h"

GreenCoordinates2D::GreenCoordinates2D()
{

}

// all z values are set to zero

void GreenCoordinates2D::constructGreenCoordinates(const Vector2f& vertexPos, vector<TwoDVertex> cagePoints, vector<TwoDEdge> cageEdges) {
    //--- Initialize all coords as 0
    phiCoords.resize(cagePoints.size());
    std::fill(phiCoords.begin(), phiCoords.end(), 0.0f);
    psiCoords.resize(cageEdges.size());
    std::fill(psiCoords.begin(), psiCoords.end(), 0.0f);

    //--- Calculate coords by iterating all edges
    for(int r = 0; r < cageEdges.size(); r++){
        auto currEdge = cageEdges[r];
        Vector2f a = currEdge.edge.second->position - currEdge.edge.first->position;
        Vector2f b = currEdge.edge.first->position - vertexPos;
        float Q = a.dot(a);
        float S = b.dot(b);
        float R = (2 * a).dot(b);
        float BA = b.dot(a.norm() * currEdge.calculateNormal()); // NORMAL DIRECTION - correct
        float SRT = std::sqrt(4 * S * Q - R * R);
        float L0 = std::log(S);
        float L1 = std::log(S + Q + R);
        float A0 = atan(R / SRT) / SRT; // atan or atan2 - atan should be fine
        float A1 = atan((2 * Q + R) / SRT) / SRT;
        float A10 = A1 - A0;
        float L10 = L1 - L0;

        psiCoords.at(r) = -((a.norm()) / (4 * M_PI)) * ((4 * S - (R * R) / Q) * A10 + (R / (2 * Q)) * L10 + L1 - 2);

        phiCoords.at(currEdge.edge.second->idx) -= (BA / (2 * M_PI)) * ((L10 / (2 * Q)) - A10 * R / Q);
        phiCoords.at(currEdge.edge.first->idx) += (BA / (2 * M_PI)) * ((L10 / (2 * Q)) - A10 * (2 + R / Q));
    }

}

void GreenCoordinates2D::constructGreenCoordinatesExterior(const Vector2f& vertexPos, vector<TwoDVertex> cagePoints, vector<TwoDEdge> cageEdges) {
    //--- Construct the coordinates the same as internal at first
    constructGreenCoordinates(vertexPos, cagePoints, cageEdges);

    //--- Add alpha and betas
    for (int r = 0; r < cageEdges.size(); r++) {
        auto currEdge = cageEdges[r];

        // TODO: Need to figure out a way to define the EXIT EDGE !!!!!!
        // Currently hardcoded edge idx 1 as the exit edge
        if (r != 0) {
            continue;
        }

        vector<Vector2f> vList(2);
        std::fill(vList.begin(), vList.end(), Vector2f(0,0));
        vList.at(0) = currEdge.edge.first->position;
        vList.at(1) = currEdge.edge.second->position;

        Vector2f edgeNormal = currEdge.calculateNormal();

        MatrixXf A(3, 3);
        A << vList.at(0).x(), vList.at(1).x(), edgeNormal.x(),
             vList.at(0).y(), vList.at(1).y(), edgeNormal.y(),
             1,               1,               0;
        Vector3f b;
        b << vertexPos.x(), vertexPos.y(), 1;

        Vector3f solution = A.colPivHouseholderQr().solve(b);

        phiCoords.at(currEdge.edge.first->idx) += solution[0];
        phiCoords.at(currEdge.edge.second->idx) += solution[1];
        psiCoords.at(r) -= solution[2];
    }
}

void GreenCoordinates2D::constructGreenCoordinatesBoundary(const Vector2f& vertexPos, vector<TwoDVertex> cagePoints, vector<TwoDEdge> cageEdges) {
    //--- Construct the coordinates the same as internal at first
    constructGreenCoordinates(vertexPos, cagePoints, cageEdges);

    //--- Add alpha and betas
    for (int r = 0; r < cageEdges.size(); r++) {
        auto currEdge = cageEdges[r];

        // TODO: Need to figure out a way to define the EXIT EDGE !!!!!!
        // Currently hardcoded edge idx 1 as the exit edge
        if (r != 0) {
            continue;
        }

        vector<Vector2f> vList(2);
        std::fill(vList.begin(), vList.end(), Vector2f(0,0));
        vList.at(0) = currEdge.edge.first->position;
        vList.at(1) = currEdge.edge.second->position;

        Vector2f edgeNormal = currEdge.calculateNormal();

//        MatrixXf A(3, 3);
//        A << vList.at(0).x(), vList.at(1).x(), edgeNormal.x(),
//            vList.at(0).y(), vList.at(1).y(), edgeNormal.y(),
//            1,               1,               0;
//        Vector3f b;
//        b << 0.5 * vertexPos.x(), 0.5 * vertexPos.y(), 0.5;
        Eigen::MatrixXd A(3, 3);
        A << vList.at(0).x(), vList.at(1).x(), edgeNormal.x(),
            vList.at(0).y(), vList.at(1).y(), edgeNormal.y(),
            1,               1,               0;
        Eigen::Vector3d b;
        b << 0.5 * vertexPos.x(), 0.5 * vertexPos.y(), 0.5;

//        Vector3f solution = A.colPivHouseholderQr().solve(b);
        Vector3d solution = A.colPivHouseholderQr().solve(b);
//        cout << solution << endl;

        phiCoords.at(currEdge.edge.first->idx) += solution[0];
        phiCoords.at(currEdge.edge.second->idx) += solution[1];
        psiCoords.at(r) -= solution[2];
    }
}
