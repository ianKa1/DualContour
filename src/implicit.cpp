#include "implicit.h"
#include <cmath>

// Convention: f < 0 = inside, f > 0 = outside. Zero level-set is the surface.

float implicitSphere(float x, float y, float z) {
    float r = 0.75f;
    return std::sqrt(x*x + y*y + z*z) - r;
}

float implicitBox(float x, float y, float z) {
    float hx = 0.6f, hy = 0.45f, hz = 0.5f;
    float dx = std::max(std::abs(x) - hx, 0.0f);
    float dy = std::max(std::abs(y) - hy, 0.0f);
    float dz = std::max(std::abs(z) - hz, 0.0f);
    return std::sqrt(dx*dx + dy*dy + dz*dz);
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

