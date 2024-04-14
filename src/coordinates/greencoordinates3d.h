#ifndef GREENCOORDINATES3D_H
#define GREENCOORDINATES3D_H

#include "mesh_struct/halfedgemesh.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;

class GreenCoordinates3D
{
public:
    GreenCoordinates3D();

    vector<float> phiCoords;
    vector<float> psiCoords;

    void constructGreenCoordinates(const Vector3f& vertexPos, HalfEdgeMesh& cage);
    void constructGreenCoordinatesExterior(const Vector3f& vertexPos, HalfEdgeMesh& cage);

private:
    float gcTriInt(Vector3f p, Vector3f v1, Vector3f v2, Vector3f eta);
};

#endif // GREENCOORDINATES3D_H
