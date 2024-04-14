#ifndef GREENCOORDINATES2D_H
#define GREENCOORDINATES2D_H

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;

class GreenCoordinates2D
{
public:
    GreenCoordinates2D();

    vector<float> phiCoords;
    vector<float> psiCoords;

    void constructGreenCoordinates(const Vector2f& vertexPos);
};

#endif // GREENCOORDINATES2D_H
