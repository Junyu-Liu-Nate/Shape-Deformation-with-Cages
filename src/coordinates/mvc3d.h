//#ifndef MVC3D_H
//#define MVC3D_H

//#include "mesh_struct/halfedgemesh.h"
//#include <Eigen/Geometry>
//#include <Eigen/Dense>
//#include <iostream>

//class MVC3D
//{
//public:
//    MVC3D();

//    vector<float> wCoords;
//    void constructMVC(const Vector3f& vertexPos, HalfEdgeMesh& cage);

//};

//#endif // MVC3D_H

#ifndef MVC3D_H
#define MVC3D_H

#include "mesh_struct/halfedgemesh.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <iostream>
#include <vector>  // Ensure to include this for std::vector

using namespace std;
using namespace Eigen;

class MVC3D
{
public:
    MVC3D();

    vector<double> wCoords;  // Changed from vector<float> to vector<double>
    void constructMVC(const Vector3d& vertexPos, HalfEdgeMesh& cage);  // Changed Vector3f to Vector3d

};

#endif // MVC3D_H
