#ifndef OBJECT2D_H
#define OBJECT2D_H

#include <Eigen/Core>
#include <Eigen/SVD>
#include "coordinates/greencoordinates2d.h"
#include "mesh_struct/margincage2d.h"
#include "coordinates/mvc2d.h"
#include "coordinates/gchigherorder2d.h"
#include "common.h"

using namespace std;
using namespace Eigen;

enum class Mode2D {
    MVC,
    Green,
    HigherOrderGreen
};

struct ObjectVertex2D {
    Vector2f position;

    GreenCoordinates2D greenCord;
    GCHigherOrder2D gcHigherOrder;
    MVC2D mvcCoord;
};

class Object2D
{
public:
    Object2D(Mode2D mode);

    vector<ObjectVertex2D> vertexList;

//    void updateVertices(vector<TwoDVertex> cagePoints, vector<TwoDEdge> cageEdges);

//    struct tuple_hash {
//        template <class T1, class T2, class T3>
//        std::size_t operator()(const std::tuple<T1, T2, T3>& tpl) const {
//            auto& [first, second, third] = tpl;
//            auto hash1 = std::hash<T1>{}(first);
//            auto hash2 = std::hash<T2>{}(second);
//            auto hash3 = std::hash<T3>{}(third);
//            return ((hash1 ^ (hash2 << 1)) >> 1) ^ (hash3 << 1); // Combine hashes
//        }
//    };
    void updateVertices(vector<TwoDVertex> cagePoints, vector<TwoDEdge> cageEdges, unordered_map<std::tuple<int, int, int>, ControlPoint, tuple_hash> controlPoints);

    vector<Vector3f> getVertices();

private:
    Mode2D m_mode;
};

#endif // OBJECT2D_H
