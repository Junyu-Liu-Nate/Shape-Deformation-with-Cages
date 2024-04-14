#ifndef OBJECT2D_H
#define OBJECT2D_H

#include <Eigen/Core>
#include <Eigen/SVD>
#include "coordinates/greencoordinates2d.h"

using namespace std;
using namespace Eigen;

struct ObjectVertex2D {
    Vector2f position;

    GreenCoordinates2D greenCord;
};

class Object2D
{
public:
    Object2D();

    vector<ObjectVertex2D> vertexList;

    void updateVertices();
    vector<Vector3f> getVertices();
};

#endif // OBJECT2D_H
