#ifndef GCHIGHERORDER2D_H
#define GCHIGHERORDER2D_H

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <iostream>
#include <complex>
#include "mesh_struct/margincage2d.h"

using namespace std;
using namespace Eigen;

class GCHigherOrder2D
{
public:
    GCHigherOrder2D();

    vector<vector<float>> phiCoords;
    vector<vector<float>> psiCoords;

    int degree = 2;

    void constructGCHigherOrder(const Vector2f& vertexPos, vector<TwoDVertex> cagePoints, vector<TwoDEdge> cageEdges);
    void gcHigherOrderEdge(const Vector2f& eta, const Vector2f& v0, const Vector2f& v1, vector<float>& phi, vector<float>& psi);
};

#endif // GCHIGHERORDER2D_H
