#ifndef MVC3D_H
#define MVC3D_H

#include "mesh_struct/halfedgemesh.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <iostream>

class MVC3D
{
public:
    MVC3D();

    vector<float> wCoords;
    void constructMVC(const Vector3f& vertexPos, HalfEdgeMesh& cage);

};

#endif // MVC3D_H
