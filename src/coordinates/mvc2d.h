#ifndef MVC2D_H
#define MVC2D_H

#include <Eigen/Dense>

using namespace Eigen;
class MVC2D
{
public:
    MVC2D();
    std::vector<float> MVCoord;
    void constructMVC(const std::vector<Vector3f> cage, Vector3f imagePos);
};

#endif // MVC2D_H
