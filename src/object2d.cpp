#include "object2d.h"

Object2D::Object2D()
{

}

void Object2D::updateVertices() {
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

    //----- Green Coordinates 2D
    for (ObjectVertex2D& objectVertex : vertexList) {
        Vector2f newPos = Vector2f(0,0);

        // Sum of weighted vertices
        Vector2f term1 = Vector2f(0,0);
        for (int i = 0; i < 4; i++) {
            term1 += objectVertex.greenCord.phiCoords.at(i) * cagePoints.at(i);
        }

        // Sum of weighted edges
        Vector2f term2 = Vector2f(0,0);
        for (int i = 0; i < 4; i++) {
            Vector2f a = cageEdges.at(i).second - cageEdges.at(i).first;
            Vector2f edgeNormal = Vector2f(-a.y(), a.x());
            term2 += objectVertex.greenCord.psiCoords.at(i) * edgeNormal * 1.0;
        }

        objectVertex.position = term1 + term2;
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
