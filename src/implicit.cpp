#include "implicit.h"
#include <cmath>

// Convention: f < 0 = inside, f > 0 = outside. Zero level-set is the surface.

float implicitSphere(float x, float y, float z) {
    float r = 0.75f;
    return std::sqrt(x*x + y*y + z*z) - r;
}

float implicitBox(float x, float y, float z) {
    const Eigen::Vector3f b(0.6f, 0.45f, 0.5f);
    const Eigen::Vector3f p(std::abs(x), std::abs(y), std::abs(z));
    const Eigen::Vector3f q = p - b;
    const Eigen::Vector3f qMax = q.cwiseMax(Eigen::Vector3f::Zero());
    const float outside = qMax.norm();
    const float inside = std::min(std::max(q.x(), std::max(q.y(), q.z())), 0.0f);
    return outside + inside;
}

float implicitTorus(float x, float y, float z) {
    float R = 0.6f, r = 0.25f;
    // Torus in XZ-plane (major radius R, minor radius r)
    float qx = std::sqrt(x*x + z*z) - R;
    return std::sqrt(qx*qx + y*y) - r;
}

Eigen::Vector3f gradient(ScalarField f, float x, float y, float z, float eps) {
    float fx = f(x + eps, y, z) - f(x - eps, y, z);
    float fy = f(x, y + eps, z) - f(x, y - eps, z);
    float fz = f(x, y, z + eps) - f(x, y, z - eps);
    Eigen::Vector3f g(fx, fy, fz);
    return g / (2.0f * eps);
}

