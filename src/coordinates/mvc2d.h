#ifndef MVC2D_H
#define MVC2D_H


#include <Eigen/Dense>
#include "mesh_struct/margincage2d.h"

using namespace Eigen;
class MVC2D
{
public:
    MVC2D();
    std::vector<float> MVCoord;
//    void constructMVC(const std::vector<Vector3f> cage, Vector3f imagePos);
    void constructMVC(Vector2f imagePos, vector<TwoDVertex> cagePoints);
};


#endif // MVC2D_H
