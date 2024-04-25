#include "mvc2d.h"
#include "Eigen/Dense"

MVC2D::MVC2D()
{

}

void MVC2D::constructMVC(const std::vector<Vector3f> cage, Vector3f vert){
    int nSize = cage.size();
    float dx, dy;
    std::vector<Vector3f> s(nSize);
    for(int i = 0; i < nSize; i++) {
<<<<<<< Updated upstream
        dx = cage[i][0] - vert[0];
        dy = cage[i][1] - vert[1];
=======
        dx = cagePoints[i].position.x() - vert[0];
        dy = cagePoints[i].position.y() - vert[0];
>>>>>>> Stashed changes
        s[i][0] = dx;
        s[i][1] = dy;
        s[i][2] = 0;
    }

    MVCoord.resize(nSize);
    std::fill(MVCoord.begin(), MVCoord.end(), 0.f);
    int ip, im; // i+ and i-
    float ri, rp, Ai, Di, dl, mu;  // Distance

    // --------------------------------------- NEW ---------------------------------------
    // If the vertex is on the edge or extremely close to it
    for(int i = 0; i < nSize; i++) {
        ip = (i+1) % nSize;
        ri = sqrt(s[i][0] * s[i][0] + s[i][1] * s[i][1]);
        Ai = 0.5f * (s[i][0] * s[ip][1] - s[ip][0] * s[i][1]);
        Di = s[ip][0] * s[i][0] + s[ip][1] * s[i][1];
        float eps = 10.0f * std::numeric_limits<float>::min();
        if( ri <= eps) {
            MVCoord[i] = 1.0;
        }
        else if(fabs(Ai) <= 0 && Di < 0.0){
            dx = cagePoints[ip].position.x() - cagePoints[i].position.x();
            dy = cagePoints[ip].position.y() - cagePoints[i].position.y();
            dl = sqrt(dx * dx + dy * dy);
            assert(dl > eps);
            dx = vert[0] - cagePoints[i].position.x();
            dy = vert[1] - cagePoints[i].position.y();
            mu = sqrt(dx * dx + dy * dy) / dl;
            assert(mu >= 0.0 && mu <= 1.0);
            MVCoord[i] = 1.0 - mu;
            MVCoord[ip] = mu;
        }
    }

    // --------------------------------------- END ---------------------------------------
    std::vector<float> tanalpha(nSize); // this stores the value of tan(alpha / 2)
    for(int i = 0; i < nSize; i++) {
        ip = (i + 1) % nSize;
        im = (nSize - 1 + i) % nSize;
        ri = sqrt(s[i][0] * s[i][0] + s[i][1] * s[i][1]);
        rp = sqrt(s[ip][0] * s[ip][0] + s[ip][1] * s[ip][1]);
        Ai = 0.5 * (s[i][0] * s[ip][1] - s[ip][0] * s[i][1]);
        Di = s[ip][0] * s[i][0] + s[ip][1] * s[i][1];
        tanalpha[i] = (ri * rp - Di) / (2.0 * Ai);
    }

    // the following is Equation 11
    float wi = 0.f, wsum = 0.f;
    for(int i = 0; i < nSize; i++) {
        im = (nSize - 1 + i) % nSize;
        ri = sqrt(s[i][0] * s[i][0] + s[i][1] * s[i][1]);
        wi = 2.0 * (tanalpha[i] + tanalpha[im]) / ri;
        wsum += wi;
        MVCoord[i] = wi;
    }

    if(fabs(wsum) > 0.0) {
        for(int i = 0; i < nSize; i++){
            MVCoord[i] /= wsum;
        }
    }
}
