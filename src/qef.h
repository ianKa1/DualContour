#pragma once
#include <Eigen/Core>
#include <vector>

struct HermiteSample {
    Eigen::Vector3f point;
    Eigen::Vector3f normal;
};

Eigen::Vector3f solveQEF(const std::vector<HermiteSample>& samples,
                         const Eigen::Vector3f& cellMin,
                         const Eigen::Vector3f& cellMax,
                         float svdThreshold = 1e-3f);


