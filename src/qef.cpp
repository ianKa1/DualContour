#include "qef.h"
#include <Eigen/SVD>
#include <algorithm>

Eigen::Vector3f solveQEF(const std::vector<HermiteSample>& samples,
                         const Eigen::Vector3f& cellMin,
                         const Eigen::Vector3f& cellMax,
                         float svdThreshold) {
    if (samples.empty()) {
        // Fallback: return mass-point clamped to cell
        Eigen::Vector3f massPoint = (cellMin + cellMax) * 0.5f;
        return massPoint.cwiseMax(cellMin).cwiseMin(cellMax);
    }

    // Compute mass-point (average of intersection points)
    Eigen::Vector3f massPoint = Eigen::Vector3f::Zero();
    for (const auto& sample : samples) {
        massPoint += sample.point;
    }
    massPoint /= static_cast<float>(samples.size());

    // Translate system to mass-point for better conditioning
    int n = static_cast<int>(samples.size());
    Eigen::MatrixXf A(n, 3);
    Eigen::VectorXf b(n);

    for (int i = 0; i < n; ++i) {
        Eigen::Vector3f p = samples[i].point - massPoint;
        Eigen::Vector3f n = samples[i].normal;
        A.row(i) = n.transpose();
        b(i) = n.dot(p);
    }

    // Solve with SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    // Set threshold to zero near-degenerate singular values
    float maxSV = svd.singularValues()(0);
    svd.setThreshold(svdThreshold * maxSV);
    
    Eigen::Vector3f x = svd.solve(b);
    
    // Translate result back
    x += massPoint;
    
    // Clamp to cell bounds
    x = x.cwiseMax(cellMin).cwiseMin(cellMax);
    
    return x;
}

