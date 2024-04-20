#include "gchigherorder2d.h"

GCHigherOrder2D::GCHigherOrder2D()
{

}

void GCHigherOrder2D::gcHigherOrderEdge(const Vector2f& eta, const Vector2f& v0, const Vector2f& v1, vector<float>& phi, vector<float>& psi) {
    float x = ((eta - v0) / (eta - v0).norm()).dot((v1 - v0) / (v1 - v0).norm());

    float y = sqrt(1 - pow(x, 2));

    complex<float> complexNumber(x, y);
    float magnitudeNumerator = (eta - v0).norm();
    float magnitudeDenominator = (v1 - v0).norm();
    complex<float> omega = (magnitudeNumerator / magnitudeDenominator) * complexNumber;

    float B = 1 / (2 * M_PI * pow((v1 - v0).norm(), 2));

    float Dreal = 0.5 * log(1 + (1 - 2 * omega.real() / pow(norm(omega), 2)));
    float Dimag = atan2(omega.imag(), pow(norm(omega), 2) - omega.real());
    complex<float> W(Dreal, Dimag);

    float F;
    float epsilon = 0.0;
    if (abs(omega.imag()) < epsilon) {
        F = B / (pow(norm(omega), 2) - omega.real());
    }
    else {
        F = (B * W.imag()) / omega.imag();
    }

    vector<float> F_list;
    F_list.resize(degree);
    for (int i = 0; i < degree; i++) {
        F_list.at(i) = F;
        F = omega.real() * F + B * W.real();
        W = omega * W + complex<float>(1 / (i + 1), 0);
    }

    float alpha = - (eta - v0).dot((v1 - v0).transpose());
    float beta = - (eta - v0).dot((v1 - v0));
    float gamma = pow((v1 - v0).norm(), 2);
    float delta = (-1 / (2 * M_PI)) * log((v1 - eta).norm());

    for (int i = 0; i < degree; i++) {
        phi.at(i) = alpha * F_list.at(i);
        if (i == degree - 1) {
            psi.at(i) = beta * F_list.at(i) + gamma * F_list.at(0) + delta;
        }
        else {
            psi.at(i) = beta * F_list.at(i) + gamma * F_list.at(i + 1) + delta;
        }
    }

    psi.at(0) = 0;
}
