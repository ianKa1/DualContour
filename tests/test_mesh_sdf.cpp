#include "mesh_sdf.h"
#include "implicit.h"
#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <string>

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

int main() {
    // --- Test 1: Load ---
    std::cout << "Test 1: Load teapot OBJ\n";
    std::string path = DATA_DIR "/teapot.obj";
    bool loaded = loadMeshSDF(path);
    check("loadMeshSDF returns true", loaded);
    if (!loaded) {
        std::cerr << "Cannot load " << path << " — aborting remaining tests\n";
        return 1;
    }

    // --- Test 2: Interior point ---
    std::cout << "Test 2: Interior/exterior sign\n";
    float sdIn = implicitMeshSDF(0.f, 0.f, 0.f);
    std::cout << "  SDF at (0,0,0) = " << sdIn << "\n";
    check("center is inside (SDF < 0)", sdIn < 0.f);

    float sdOut = implicitMeshSDF(0.f, 5.f, 0.f);
    std::cout << "  SDF at (0,5,0) = " << sdOut << "\n";
    check("far point is outside (SDF > 0)", sdOut > 0.f);

    // --- Test 3: Normalization — all 8 corners of [-0.9,0.9]^3 are outside ---
    std::cout << "Test 3: All bounding-box corners are outside\n";
    bool allOutside = true;
    for (float sx : {-0.9f, 0.9f})
        for (float sy : {-0.9f, 0.9f})
            for (float sz : {-0.9f, 0.9f}) {
                float sd = implicitMeshSDF(sx, sy, sz);
                if (sd <= 0.f) {
                    std::cout << "  Corner (" << sx << "," << sy << "," << sz
                              << ") has SDF=" << sd << " (not outside!)\n";
                    allOutside = false;
                }
            }
    check("all [-0.9,0.9]^3 corners outside", allOutside);

    // --- Test 4: SDF gradient magnitude ≈ 1 near the surface ---
    std::cout << "Test 4: Gradient magnitude ≈ 1 near surface\n";
    // Sample a grid; collect points where |SDF| < 0.05 and check grad norm
    int gradSamples = 0, gradGood = 0;
    const int M = 20;
    const float step = 1.8f / M;
    for (int ix = 0; ix <= M; ++ix)
        for (int iy = 0; iy <= M; ++iy)
            for (int iz = 0; iz <= M; ++iz) {
                float x = -0.9f + ix * step;
                float y = -0.9f + iy * step;
                float z = -0.9f + iz * step;
                float sd = implicitMeshSDF(x, y, z);
                if (std::abs(sd) < 0.05f) {
                    ++gradSamples;
                    Eigen::Vector3f g = gradient(implicitMeshSDF, x, y, z);
                    float gn = g.norm();
                    // SDF gradient magnitude should be close to 1
                    if (gn > 0.5f && gn < 2.0f) ++gradGood;
                }
            }
    std::cout << "  Near-surface samples: " << gradSamples
              << "  with |grad|∈(0.5,2): " << gradGood << "\n";
    check("found near-surface samples", gradSamples > 0);
    if (gradSamples > 0)
        check("≥80% of near-surface gradients have |grad| ∈ (0.5, 2.0)",
              gradGood >= gradSamples * 8 / 10);

    // --- Test 5: Finite values everywhere in [-0.9,0.9]^3 ---
    std::cout << "Test 5: No NaN/inf in SDF over grid\n";
    bool allFinite = true;
    const int K = 15;
    const float kstep = 1.8f / K;
    for (int ix = 0; ix <= K && allFinite; ++ix)
        for (int iy = 0; iy <= K && allFinite; ++iy)
            for (int iz = 0; iz <= K && allFinite; ++iz) {
                float x = -0.9f + ix * kstep;
                float y = -0.9f + iy * kstep;
                float z = -0.9f + iz * kstep;
                float sd = implicitMeshSDF(x, y, z);
                if (!std::isfinite(sd)) allFinite = false;
            }
    check("all SDF values finite", allFinite);

    std::cout << "\nResults: " << g_pass << " passed, " << g_fail << " failed\n";
    return g_fail > 0 ? 1 : 0;
}
