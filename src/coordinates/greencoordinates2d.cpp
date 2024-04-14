#include "greencoordinates2d.h"

GreenCoordinates2D::GreenCoordinates2D()
{

}

// z should be zero
void GreenCoordinates2D::constructGreenCoordinates(const Vector2f& vertexPos) {
    //--- Initialize all coords as 0
    phiCoords.resize(4);
    std::fill(phiCoords.begin(), phiCoords.end(), 0.0f);
    psiCoords.resize(4);
    std::fill(psiCoords.begin(), psiCoords.end(), 0.0f);

    //--- Hardcode cage
    // cage points
    vector<Vector2f> cagePoints;
    cagePoints.push_back(Vector2f(1, -1));
    cagePoints.push_back(Vector2f(1, 1));
    cagePoints.push_back(Vector2f(-1, 1));
    cagePoints.push_back(Vector2f(-1, -1));

    // cage edges
    vector<std::pair<Vector2f, Vector2f>> cageEdges;
    for(int i = 0; i < cagePoints.size(); ++i) {
        std::pair<Vector2f, Vector2f> edge;

        // Last position, do the first minus the last
        if (i == cagePoints.size() - 1) {
            edge.first = cagePoints[i];
            edge.second = cagePoints[0];
        }

        // Store the value of the next element minus the current element
        else {
            edge.first = cagePoints[i];
            edge.second = cagePoints[i + 1];
        }
        cageEdges.push_back(edge);
    }

    //--- Calculate coords by iterating all edges
    int j = 0;
    for(int r = 0; r < cageEdges.size(); r++){
        auto currEdge = cageEdges[r];
        Vector2f a = currEdge.second - currEdge.first;
        Vector2f b = currEdge.first - vertexPos;
        float Q = a.dot(a);
        float S = b.dot(b);
        float R = (2 * a).dot(b);
        float BA = b.dot(a.norm() * Vector2f(-a.y(), a.x()).normalized()); // NORMAL DIRECTION ???????????????
        float SRT = std::sqrt(4 * S * Q - R * R);
        float L0 = std::log(S);
        float L1 = std::log(S + Q + R);
        float A0 = atan(R / SRT) / SRT; // atan or atan2 ????????????????????
        float A1 = atan((2 * Q + R) / SRT) / SRT;
        float A10 = A1 - A0;
        float L10 = L1 - L0;

        //            psi += -((a.norm()) / (4 * M_PI)) * ((4 * S - (R * R) / Q) * A10 + (R / (2 * Q)) * L10 + L1 - 2); // INCREMENT OR NOT ??????
        //            phi2 = phi2 - (BA / (2 * M_PI)) * ((L10 / (2 * Q)) - A10 * R / Q);
        //            phi1 = phi1 + (BA / (2 * M_PI)) * ((L10 / (2 * Q)) - A10 * (2 + R / Q));

        psiCoords.at(j) = -((a.norm()) / (4 * M_PI)) * ((4 * S - (R * R) / Q) * A10 + (R / (2 * Q)) * L10 + L1 - 2);
        phiCoords.at(j+1) -= (BA / (2 * M_PI)) * ((L10 / (2 * Q)) - A10 * R / Q);
        phiCoords.at(j) += (BA / (2 * M_PI)) * ((L10 / (2 * Q)) - A10 * (2 + R / Q));

        j++;
    }

}
