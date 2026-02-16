#include "qef.h"
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>

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
    Eigen::Vector3d massPoint = Eigen::Vector3d::Zero();
    for (const auto& sample : samples) {
        massPoint += sample.point.cast<double>();
    }
    massPoint /= static_cast<double>(samples.size());

    // Translate system to mass-point for better conditioning
    const int sampleCount = static_cast<int>(samples.size());
    Eigen::MatrixXd A(sampleCount, 3);
    Eigen::VectorXd b(sampleCount);

    for (int i = 0; i < sampleCount; ++i) {
        const Eigen::Vector3d p = samples[i].point.cast<double>() - massPoint;
        Eigen::Vector3d normal = samples[i].normal.cast<double>();
        const double nNorm = normal.norm();
        if (nNorm > 1e-12) {
            normal /= nNorm;
        } else {
            normal = Eigen::Vector3d::UnitX();
        }
        A.row(i) = normal.transpose();
        b(i) = normal.dot(p);
    }

    // Solve with SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // Set threshold to suppress near-degenerate singular values (rank deficiency)
    const auto& svals = svd.singularValues();
    const double maxSV = svals.size() > 0 ? svals(0) : 1.0;
    svd.setThreshold(static_cast<double>(svdThreshold) * std::max(1.0, maxSV));

    // If rank < 3 the system is underdetermined; minimum-norm SVD solve still gives
    // a reasonable result in constrained directions, with zero displacement in null-space
    // directions (i.e., biased toward massPoint). This handles edge/corner features.
    Eigen::Vector3d x = svd.solve(b);

    // Translate back to world space
    x += massPoint;
    if (!std::isfinite(x.x()) || !std::isfinite(x.y()) || !std::isfinite(x.z())) {
        x = massPoint;
    }

    // If the QEF solution lands outside the cell (rank deficiency or ill-conditioning),
    // fall back to the mass point rather than clamping to the cell boundary.
    // The mass point is the average of intersection points and lies on/near the surface,
    // so it produces far less geometric distortion than a boundary-clamped position.
    Eigen::Vector3f xf = x.cast<float>();
    const bool inCell = (xf.array() >= cellMin.array()).all() &&
                        (xf.array() <= cellMax.array()).all();
    if (!inCell) {
        xf = massPoint.cast<float>();
        xf = xf.cwiseMax(cellMin).cwiseMin(cellMax);  // safety clamp for massPoint too
    }

    return xf;
}

