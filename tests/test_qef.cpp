#include "qef.h"
#include <cmath>
#include <iostream>
#include <cassert>

static int g_pass = 0, g_fail = 0;

static void check(const char* name, bool cond) {
    if (cond) {
        std::cout << "  PASS: " << name << "\n";
        ++g_pass;
    } else {
        std::cout << "  FAIL: " << name << "\n";
        ++g_fail;
    }
}

static bool inCell(const Eigen::Vector3f& v,
                   const Eigen::Vector3f& lo,
                   const Eigen::Vector3f& hi,
                   float tol = 1e-4f) {
    return v.x() >= lo.x()-tol && v.x() <= hi.x()+tol &&
           v.y() >= lo.y()-tol && v.y() <= hi.y()+tol &&
           v.z() >= lo.z()-tol && v.z() <= hi.z()+tol;
}

// ---- Test 1: Empty samples → cell center ----------------------------------
static void testEmpty() {
    std::cout << "Test 1: Empty samples\n";
    Eigen::Vector3f lo(0,0,0), hi(1,1,1);
    auto v = solveQEF({}, lo, hi);
    check("result inside cell", inCell(v, lo, hi));
    check("result near center", (v - Eigen::Vector3f(0.5f,0.5f,0.5f)).norm() < 1e-3f);
}

// ---- Test 2: Single plane z=0.5, normal (0,0,1) ---------------------------
static void testSinglePlane() {
    std::cout << "Test 2: Single plane (z=0.5, n=(0,0,1))\n";
    Eigen::Vector3f lo(0,0,0), hi(1,1,1);
    std::vector<HermiteSample> samples;
    // Four points on the plane z=0.5 at corners of [0.25,0.75]²
    for (float x : {0.25f, 0.75f})
        for (float y : {0.25f, 0.75f})
            samples.push_back({{x, y, 0.5f}, {0,0,1}});
    auto v = solveQEF(samples, lo, hi);
    check("result inside cell", inCell(v, lo, hi));
    check("z coordinate ≈ 0.5", std::abs(v.z() - 0.5f) < 0.05f);
}

// ---- Test 3: Edge feature — two orthogonal planes meeting at x=0.3,y=0.6 --
static void testEdgeFeature() {
    std::cout << "Test 3: Edge feature (x=0.3, y=0.6)\n";
    Eigen::Vector3f lo(0,0,0), hi(1,1,1);
    std::vector<HermiteSample> samples;
    // Plane 1: x=0.3, normal (1,0,0) — four samples at different z
    for (float z : {0.2f, 0.4f, 0.6f, 0.8f})
        samples.push_back({{0.3f, 0.6f, z}, {1,0,0}});
    // Plane 2: y=0.6, normal (0,1,0) — four samples at different z
    for (float z : {0.2f, 0.4f, 0.6f, 0.8f})
        samples.push_back({{0.3f, 0.6f, z}, {0,1,0}});
    auto v = solveQEF(samples, lo, hi);
    check("result inside cell", inCell(v, lo, hi));
    check("x ≈ 0.3", std::abs(v.x() - 0.3f) < 0.05f);
    check("y ≈ 0.6", std::abs(v.y() - 0.6f) < 0.05f);
}

// ---- Test 4: Corner feature — three orthogonal planes at (0.4,0.5,0.6) ----
// Each sample gives one plane equation: n_i·(x - p_i)=0
// With one sample per axis at the same point, the system is exactly determined.
static void testCornerFeature() {
    std::cout << "Test 4: Corner feature (0.4, 0.5, 0.6)\n";
    Eigen::Vector3f lo(0,0,0), hi(1,1,1);
    std::vector<HermiteSample> samples = {
        {{0.4f, 0.5f, 0.6f}, {1,0,0}},  // plane x=0.4
        {{0.4f, 0.5f, 0.6f}, {0,1,0}},  // plane y=0.5
        {{0.4f, 0.5f, 0.6f}, {0,0,1}},  // plane z=0.6
    };
    auto v = solveQEF(samples, lo, hi);
    check("result inside cell", inCell(v, lo, hi));
    check("x ≈ 0.4", std::abs(v.x() - 0.4f) < 0.05f);
    check("y ≈ 0.5", std::abs(v.y() - 0.5f) < 0.05f);
    check("z ≈ 0.6", std::abs(v.z() - 0.6f) < 0.05f);
}

// ---- Test 5: Degenerate — all normals (0,0,1), rank-1 system ---------------
static void testDegenerate() {
    std::cout << "Test 5: Degenerate (all normals parallel)\n";
    Eigen::Vector3f lo(0,0,0), hi(1,1,1);
    std::vector<HermiteSample> samples;
    for (float x : {0.2f, 0.5f, 0.8f})
        samples.push_back({{x, 0.5f, 0.5f}, {0,0,1}});
    auto v = solveQEF(samples, lo, hi);
    check("result inside cell", inCell(v, lo, hi));
    // Should not produce NaN or inf
    check("finite result", std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z()));
}

// ---- Test 6: Clamping — solution outside cell should be clamped ------------
static void testClamping() {
    std::cout << "Test 6: Clamping to cell bounds\n";
    // Tight cell [0.4, 0.6]^3; plane at z=0.9 (outside cell)
    Eigen::Vector3f lo(0.4f,0.4f,0.4f), hi(0.6f,0.6f,0.6f);
    std::vector<HermiteSample> samples;
    for (float x : {0.45f, 0.55f})
        for (float y : {0.45f, 0.55f})
            samples.push_back({{x, y, 0.9f}, {0,0,1}});
    auto v = solveQEF(samples, lo, hi);
    check("result inside cell", inCell(v, lo, hi));
    check("z clamped to hi.z", std::abs(v.z() - 0.6f) < 1e-4f);
}

int main() {
    testEmpty();
    testSinglePlane();
    testEdgeFeature();
    testCornerFeature();
    testDegenerate();
    testClamping();

    std::cout << "\nResults: " << g_pass << " passed, " << g_fail << " failed\n";
    return g_fail > 0 ? 1 : 0;
}
