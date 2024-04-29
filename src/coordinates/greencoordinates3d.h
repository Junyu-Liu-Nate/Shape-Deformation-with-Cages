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

    vector<double> phiCoords;  // Changed from vector<float> to vector<double>
    vector<double> psiCoords;  // Changed from vector<float> to vector<double>

    double numericalEpsilon = 1e-8;  // Changed from float to double

    void constructGreenCoordinates(const Vector3d& vertexPos, HalfEdgeMesh& cage);
    void constructGreenCoordinatesExterior(const Vector3d& vertexPos, HalfEdgeMesh& cage);

private:
    double gcTriInt(Vector3d p, Vector3d v1, Vector3d v2, Vector3d eta);  // All Vector3f changed to Vector3d and float to double
};

#endif // GREENCOORDINATES3D_H
