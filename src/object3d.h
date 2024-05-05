#ifndef OBJECT3D_H
#define OBJECT3D_H

#include <Eigen/Core>
#include <Eigen/SVD>
#include "coordinates/greencoordinates3d.h"
#include "coordinates/mvc3d.h"
#include "coordinates/greencoordinates2d.h"
#include "mesh_struct/halfedgemesh.h"
#include <QtConcurrent/QtConcurrentMap>

using namespace std;
using namespace Eigen;

struct ObjectVertex {
    Vector3f position;

    GreenCoordinates3D greenCord;
    MVC3D mvcCoord;
};

class Object3D
{
public:
    Object3D(bool useGreen);

    vector<ObjectVertex> vertexList;

    void updateVertices(const HalfEdgeMesh& heMesh);
    vector<Vector3f> getVertices();

    double calculateS(const Face face);

private:
    bool m_useGreen = false;
};

#endif // OBJECT3D_H
