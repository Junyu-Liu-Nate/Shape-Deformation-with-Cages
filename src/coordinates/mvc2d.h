#ifndef MVC2D_H
#define MVC2D_H

<<<<<<< HEAD
#include <Eigen/Dense>
=======

#include <Eigen/Dense>
#include "mesh_struct/margincage2d.h"
#include <iostream>
>>>>>>> 3D-Green-Coord

using namespace Eigen;
class MVC2D
{
public:
    MVC2D();
    std::vector<float> MVCoord;
<<<<<<< HEAD
    void constructMVC(const std::vector<Vector3f> cage, Vector3f imagePos);
};

=======
//    void constructMVC(const std::vector<Vector3f> cage, Vector3f imagePos);
    void constructMVC(Vector2f imagePos, vector<TwoDVertex> cagePoints);
};


>>>>>>> 3D-Green-Coord
#endif // MVC2D_H
