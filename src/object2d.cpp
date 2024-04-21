#include "object2d.h"

Object2D::Object2D()
{

}

//void Object2D::updateVertices(vector<Vector2f> cagePoints, vector<std::pair<Vector2f, Vector2f>> cageEdges, vector<Vector2f> cageOriginalLengths) {
//    //----- Green Coordinates 2D
//    for (ObjectVertex2D& objectVertex : vertexList) {
//        Vector2f newPos = Vector2f(0,0);

//        // Sum of weighted vertices
//        Vector2f term1 = Vector2f(0,0);
//        for (int i = 0; i < 4; i++) {
//            term1 += objectVertex.greenCord.phiCoords.at(i) * cagePoints.at(i);
//        }

//        // Sum of weighted edges
//        Vector2f term2 = Vector2f(0,0);
//        for (int i = 0; i < 4; i++) {
//            Vector2f a = cageEdges.at(i).second - cageEdges.at(i).first;
//            Vector2f edgeNormal = Vector2f(-a.y(), a.x()).normalized();
//            float s = a.norm() / cageOriginalLengths.at(i).norm();
//            term2 += objectVertex.greenCord.psiCoords.at(i) * -edgeNormal * s; // TODO: This fix is not consistent with the paper
//        }

//        objectVertex.position = (term1 + term2);
//    }
//}

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
}

vector<Vector3f> Object2D::getVertices() {
    vector<Vector3f> returnVertices;
    returnVertices.resize(vertexList.size());

    for (int i = 0; i < vertexList.size(); i++) {
        returnVertices.at(i) = Vector3f(vertexList.at(i).position.x(), vertexList.at(i).position.y(), 0.0);
    }

    return returnVertices;
}

