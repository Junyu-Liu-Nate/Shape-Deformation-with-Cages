#include "object2d.h"

Object2D::Object2D()
{

}

void Object2D::updateVertices(vector<TwoDVertex> cagePoints, vector<TwoDEdge> cageEdges, unordered_map<std::tuple<int, int, int>, ControlPoint, tuple_hash> controlPoints) {
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

//            vector<float> phi = objectVertex.gcHigherOrder.phiCoords.at(phiIdx);
//            vector<float> psi = objectVertex.gcHigherOrder.psiCoords.at(i);

//            //----- Linear case: degree = 1
////            Vector2f c0 = v0;
////            Vector2f c1 = v1 - v0;
////            Vector2f term1 = phi.at(0) * c0 + phi.at(1) * c1;
////            Vector2f term2 = psi.at(0) * Vector2f(c0.y(), -c0.x()) + psi.at(1) * Vector2f(c1.y(), -c1.x());

//            //----- Quadratic case: degree = 2
////            Vector2f vIntermidiate = (v1 + v0) / 2;
////            Vector2f c0 = v0;
////            Vector2f c1 = -2 * (v0 - vIntermidiate);
////            Vector2f c2 = v1 + v0 - 2 * vIntermidiate;
////            Vector2f term1 = phi.at(0) * c0 + phi.at(1) * c1 + phi.at(2) * c2;
////            Vector2f term2 = psi.at(0) * Vector2f(c0.y(), -c0.x()) + psi.at(1) * Vector2f(c1.y(), -c1.x()) + psi.at(2) * Vector2f(c2.y(), -c2.x());

//            //----- Cubic case: degree = 3
//            Vector2f p0 = v0;
////            Vector2f p1 = v0 + (v1 - v0) / 3;
////            Vector2f p2 = v0 + 2 * (v1 - v0) / 3;
//            Vector2f p1 = controlPoints[make_tuple(cageEdges.at(i).edge.first->idx, cageEdges.at(i).edge.second->idx, 1)].position;
//            Vector2f p2 = controlPoints[make_tuple(cageEdges.at(i).edge.first->idx, cageEdges.at(i).edge.second->idx, 2)].position;
//            Vector2f p3 = v1;
//            Vector2f c0 = p0;
//            Vector2f c1 = -3 * p0 + 3 * p1;
//            Vector2f c2 = 3 * p0 - 6 * p1 + 3 * p2;
//            Vector2f c3 = -p0 + 3 * p1 - 3 * p2 + p3;
//            Vector2f term1 = phi.at(0) * c0 + phi.at(1) * c1 + phi.at(2) * c2 + phi.at(3) * c3;
//            Vector2f term2 = psi.at(0) * Vector2f(c0.y(), -c0.x()) + psi.at(1) * Vector2f(c1.y(), -c1.x()) + psi.at(2) * Vector2f(c2.y(), -c2.x()) + psi.at(3) * Vector2f(c3.y(), -c3.x());

//            newPos += term1 + term2;
//        }

//        objectVertex.position = newPos;
//    }

    //----- MVC 2D
//    int printCounter = 0;
//    for (ObjectVertex2D& objectVertex : vertexList) {
//        Vector2f newPos = Vector2f(0,0);

//        // Sum of weighted vertices
//        Vector2f term1 = Vector2f(0,0);
//        for (int i = 0; i < cagePoints.size(); i++) {
//            if (cagePoints.at(i).isMargin) {
//                term1 += objectVertex.mvcCoord.MVCoord.at(i) * cagePoints.at(i).position;
//            }
//        }

//        objectVertex.position = term1;
//        printCounter ++;
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

