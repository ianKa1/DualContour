#pragma once
#include <Eigen/Core>

using ScalarField = float(*)(float x, float y, float z);

float implicitSphere(float x, float y, float z);  // r=0.75
float implicitBox   (float x, float y, float z);  // half-extents (0.6,0.45,0.5)
float implicitTorus (float x, float y, float z);  // R=0.6, r=0.25 in XZ-plane
Eigen::Vector3f gradient(ScalarField f, float x, float y, float z, float eps=1e-4f);

