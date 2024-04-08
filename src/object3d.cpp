#include "object3d.h"

Object3D::Object3D()
{

}

void Object3D::updateVertices(const HalfEdgeMesh& heMesh) {
    int counter = 0;
    for (ObjectVertex& objectVertex : vertexList) {
        Vector3f term1 = Vector3f(0,0,0);
        for (int i = 0; i < objectVertex.greenCord.phiCoords.size(); i++) {
            term1 += objectVertex.greenCord.phiCoords.at(i) * heMesh.vertices.at(i).position;
        }

        Vector3f term2 = Vector3f(0,0,0);
        for (int i = 0; i < objectVertex.greenCord.psiCoords.size(); i++) {
            // TODO: calculate s_j
            term2 += objectVertex.greenCord.psiCoords.at(i) * heMesh.faces.at(i).getNormal() * 0.5;
            // TODO: psiCoords contain nan
            // TODO: Normal calculation is problematic
//            std::cout << heMesh.faces.at(i).getNormal() << std::endl << std::endl;
        }

        objectVertex.position = term1 + term2;

        if (counter == 0) {
//            std::cout << term1 << std::endl;
//            std::cout << term2 << std::endl << std::endl;
        }
        counter++;
    }
//    std::cout << vertexList.at(0).position << std::endl;
}

vector<Vector3f> Object3D::getVertices() {
    vector<Vector3f> returnVertices;
    returnVertices.resize(vertexList.size());

    for (int i = 0; i < vertexList.size(); i++) {
        returnVertices.at(i) = vertexList.at(i).position;
    }

    return returnVertices;
}
