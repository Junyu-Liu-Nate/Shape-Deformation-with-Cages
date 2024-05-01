#ifndef GREENCOORDINATES2D_H
#define GREENCOORDINATES2D_H

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <iostream>
#include "mesh_struct/margincage2d.h"

using namespace std;
using namespace Eigen;

class GreenCoordinates2D
{
public:
    GreenCoordinates2D();

    vector<float> phiCoords;
    vector<float> psiCoords;

    void constructGreenCoordinates(const Vector2f& vertexPos, vector<TwoDVertex> cagePoints, vector<TwoDEdge> cageEdges);
    void constructGreenCoordinatesExterior(const Vector2f& vertexPos, vector<TwoDVertex> cagePoints, vector<TwoDEdge> cageEdges);
    void constructGreenCoordinatesBoundary(const Vector2f& vertexPos, vector<TwoDVertex> cagePoints, vector<TwoDEdge> cageEdges);
};

#endif // GREENCOORDINATES2D_H
