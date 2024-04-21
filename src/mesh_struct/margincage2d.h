#ifndef MARGINCAGE2D_H
#define MARGINCAGE2D_H

#include <list>
#include <Eigen/Core>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <Eigen/SVD>

using namespace std;
using namespace Eigen;

struct TwoDVertex {
    int idx;
    bool isMargin;
    Vector2f position;
};

struct TwoDEdge {
    bool isMargin;
    pair<TwoDVertex*, TwoDVertex*> edge;
    float originalLength;

    Vector2f calculateNormal() {
        Vector2f a = edge.second->position - edge.first->position;

        return Vector2f(-a.y(), a.x()).normalized();
    }
};

class MarginCage2D
{
public:
    MarginCage2D();
};

#endif // MARGINCAGE2D_H
