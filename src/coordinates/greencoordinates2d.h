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

    void constructGreenCoordinates(const Vector2f& vertexPos, vector<Vector2f> cagePoints, vector<std::pair<Vector2f, Vector2f>> cageEdges);
};

#endif // GREENCOORDINATES2D_H
