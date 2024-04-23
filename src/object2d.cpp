#include "object2d.h"

Object2D::Object2D()
{

}

void Object2D::updateVertices(vector<TwoDVertex> cagePoints, vector<TwoDEdge> cageEdges) {
    //----- Green Coordinates 2D
    for (ObjectVertex2D& objectVertex : vertexList) {
        Vector2f newPos = Vector2f(0,0);

        // Sum of weighted vertices
        Vector2f term1 = Vector2f(0,0);
        for (int i = 0; i < cagePoints.size(); i++) {
            term1 += objectVertex.greenCord.phiCoords.at(i) * cagePoints.at(i).position;
        }

        // Sum of weighted edges
        Vector2f term2 = Vector2f(0,0);
        for (int i = 0; i < cageEdges.size(); i++) {
            Vector2f a = cageEdges.at(i).edge.second->position - cageEdges.at(i).edge.first->position;
            Vector2f edgeNormal = cageEdges.at(i).calculateNormal();
            float s = a.norm() / cageEdges.at(i).originalLength;
            term2 += objectVertex.greenCord.psiCoords.at(i) * -edgeNormal * s; // TODO: This fix is not consistent with the paper
        }

        objectVertex.position = (term1 + term2);
    }

    //----- Green Coordinates Higher Order 2D
//    for (ObjectVertex2D& objectVertex : vertexList) {
//        Vector2f newPos = Vector2f(0,0);

//        for (int i = 0; i < cageEdges.size(); i++) {
//            Vector2f v0 = cageEdges.at(i).edge.first->position;
//            int phiIdx = cageEdges.at(i).edge.first->idx;
//            Vector2f v1 = cageEdges.at(i).edge.second->position;
////            Vector2f vIntermidiate = (v1 + v0) / 2;

//            vector<float> phi = objectVertex.gcHigherOrder.phiCoords.at(phiIdx);
//            vector<float> psi = objectVertex.gcHigherOrder.psiCoords.at(i);

////            Vector2f term1 = phi.at(0) * v0 + phi.at(1) * vIntermidiate + phi.at(2) * v1;
////            Vector2f term2 = psi.at(0) * Vector2f(v0.y(), -v0.x()) + psi.at(1) * Vector2f(vIntermidiate.y(), -vIntermidiate.x()) + psi.at(2) * Vector2f(v1.y(), -v1.x());

//            Vector2f term1 = phi.at(0) * v0 + phi.at(1) * v1;
//            Vector2f term2 = psi.at(0) * Vector2f(v0.y(), -v0.x()) + psi.at(1) * Vector2f(v1.y(), -v1.x());

//            newPos += term1 + term2;
//        }

//        objectVertex.position = newPos;
//    }

    //----- MVC 2D
//    for (ObjectVertex2D& objectVertex : vertexList) {
//        Vector2f newPos = Vector2f(0,0);

//        // Sum of weighted vertices
//        Vector2f term1 = Vector2f(0,0);
//        for (int i = 0; i < cagePoints.size(); i++) {
//            term1 += objectVertex.mvcCoord.MVCoord.at(i) * cagePoints.at(i).position;
//        }

//        objectVertex.position = term1;
//    }
}

vector<Vector3f> Object2D::getVertices() {
    vector<Vector3f> returnVertices;
    returnVertices.resize(vertexList.size());

    for (int i = 0; i < vertexList.size(); i++) {
        returnVertices.at(i) = Vector3f(vertexList.at(i).position.x(), vertexList.at(i).position.y(), 0.0);
    }

    return returnVertices;
}

