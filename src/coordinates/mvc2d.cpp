#include "mvc2d.h"
#include "Eigen/Dense"

MVC2D::MVC2D()
{

}

//----- NOTE: NEED to ensure that the cage points are connect by each other !

//void MVC2D::constructMVC(Vector2f vert, vector<TwoDVertex> cagePoints){
//    int nSize = cagePoints.size();
//    float dx, dy;
//    std::vector<Vector3f> s(nSize);
//    for(int i = 0; i < nSize; i++) {
//        dx = cagePoints.at(i).position[0] - vert[0];
//        dy = cagePoints.at(i).position[1] - vert[1];
//        s[i][0] = dx;
//        s[i][1] = dy;
//        s[i][2] = 0;
//    }

//    MVCoord.resize(nSize);
//    std::fill(MVCoord.begin(), MVCoord.end(), 1.0f); // What should the initial value be???
//    int ip, im; // i+ and i-
//    float ri, rp, Ai, Di, dl, mu;  // Distance

//    vector<float> AList;
//    vector<float> rList;
//    vector<float> DList;
//    AList.resize(nSize);
//    rList.resize(nSize);
//    DList.resize(nSize);

//    // If the vertex is on the edge or extremely close to it
//    vector<int> specialCaseIndices;

//    for(int i = 0; i < nSize; i++) {
//        ip = (i+1) % nSize;
//        ri = s[i].norm();
//        rList.at(i) = ri;
//        Ai = 0.5f * (s[i][0] * s[ip][1] - s[ip][0] * s[i][1]); // correct
//        AList.at(i) = Ai;
//        Di = s[ip].dot(s[i]);
//        DList.at(i) = Di;
//        float eps = 10.0f * std::numeric_limits<float>::min();
//        if( ri <= eps) {
//            cout << "Check ri <= eps" << endl;
//        }
//        if(fabs(Ai) <= 0 && Di < 0.0){
//            cout << "Check fabs(Ai) <= 0 && Di < 0.0" << endl;
//            rp = s[ip].norm();
//            MVCoord[i] = (ri + MVCoord[i] + ri * MVCoord[ip]) / (ri + rp);
//            specialCaseIndices.push_back(i);
//        }
//    }

//    float wsum = 0;
//    for (int i = 0; i < nSize; i++) {
//        if (find(specialCaseIndices.begin(), specialCaseIndices.end(), i) != specialCaseIndices.end()) {
//            continue;
//        }
//        ip = (i + 1) % nSize;
//        im = (nSize - 1 + i) % nSize;
//        float w = 0;
//        if (AList.at(im) != 0) {
//            w += (rList.at(im) - DList.at(im) / rList.at(i)) / AList.at(im);
//        }
//        if (AList.at(i) != 0) {
//            w += (rList.at(ip) - DList.at(i) / rList.at(i)) / AList.at(i);
//        }
//        MVCoord[i] += w * MVCoord[i];
//        wsum += w;
//    }

//    if(fabs(wsum) > 0.0) {
//        for(int i = 0; i < nSize; i++){
//            if (find(specialCaseIndices.begin(), specialCaseIndices.end(), i) != specialCaseIndices.end()) {
//                continue;
//            }
//            MVCoord[i] /= wsum;
//        }
//    }
//}

void MVC2D::constructMVC(Vector2f vert, vector<TwoDVertex> cagePoints){
    int nSize = cagePoints.size();
    std::vector<Vector3f> s(nSize);
    for(int i = 0; i < nSize; i++) {
        float dx = cagePoints.at(i).position[0] - vert[0];
        float dy = cagePoints.at(i).position[1] - vert[1];
        s[i] = Vector3f(dx, dy, 0);
    }

    MVCoord.resize(nSize, 0.0f); // Initialize MVCs to zero
    vector<float> AList(nSize), rList(nSize), DList(nSize);
    vector<int> specialCaseIndices;

    for(int i = 0; i < nSize; i++) {
        int ip = (i + 1) % nSize;
        rList[i] = s[i].norm();
        AList[i] = 0.5f * (s[i][0] * s[ip][1] - s[ip][0] * s[i][1]);
        DList[i] = s[ip].dot(s[i]);

        // Check for special cases
        if(rList[i] <= std::numeric_limits<float>::epsilon() * 10) {
            std::cout << "Vertex is very close to or on the cage vertex" << std::endl;
        }
    }

    float wsum = 0.0f;
    for(int i = 0; i < nSize; i++) {
        if(std::find(specialCaseIndices.begin(), specialCaseIndices.end(), i) != specialCaseIndices.end())
            continue;

        int ip = (i + 1) % nSize;
        int im = (i + nSize - 1) % nSize;
        float w = 0.0f;

        if(AList[im] != 0.0f)
            w += (rList[im] - DList[im] / rList[i]) / AList[im];

        if(AList[i] != 0.0f)
            w += (rList[ip] - DList[i] / rList[i]) / AList[i];

        MVCoord[i] += w;
        wsum += w;
    }

    if(std::fabs(wsum) > std::numeric_limits<float>::epsilon()) {
        for(int i = 0; i < nSize; i++) {
            if(std::find(specialCaseIndices.begin(), specialCaseIndices.end(), i) != specialCaseIndices.end())
                continue;

            MVCoord[i] /= wsum;
        }
    }
}
