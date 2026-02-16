#include "dual_contour.h"
#include "implicit.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
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

// Signed volume contribution of a triangle (divergence theorem)
static double triSignedVolume(const std::array<float,3>& a,
                               const std::array<float,3>& b,
                               const std::array<float,3>& c) {
    Eigen::Vector3d va(a[0],a[1],a[2]);
    Eigen::Vector3d vb(b[0],b[1],b[2]);
    Eigen::Vector3d vc(c[0],c[1],c[2]);
    return va.dot(vb.cross(vc)) / 6.0;
}

static void runTests(int N) {
    std::cout << "\n=== N=" << N << " ===\n";

    DCGrid grid = buildGrid(implicitSphere, N);
    DCMesh mesh = dualContour(implicitSphere, grid);

    const int nVerts = static_cast<int>(mesh.vertices.size());
    const int nTris  = static_cast<int>(mesh.triangles.size());
    std::cout << "  Vertices: " << nVerts << "  Triangles: " << nTris << "\n";

    check("non-empty mesh", nVerts > 0 && nTris > 0);

    // 1. Vertex global bounds: inside [-1,1]^3
    {
        bool allInBounds = true;
        for (const auto& v : mesh.vertices) {
            if (std::abs(v[0]) > 1.01f || std::abs(v[1]) > 1.01f || std::abs(v[2]) > 1.01f) {
                allInBounds = false;
                break;
            }
        }
        check("all vertices within [-1,1]^3", allInBounds);
    }

    // 2. Cell containment: each assigned vertex lies inside its cell
    {
        float minB = grid.minBound;
        float cs   = grid.cellSize;
        int violations = 0;
        for (int ck = 0; ck < N; ++ck) {
            for (int cj = 0; cj < N; ++cj) {
                for (int ci = 0; ci < N; ++ci) {
                    int idx = grid.vertexIndex[ci + N*cj + N*N*ck];
                    if (idx < 0) continue;
                    const auto& v = mesh.vertices[idx];
                    float xlo = minB + ci * cs, xhi = xlo + cs;
                    float ylo = minB + cj * cs, yhi = ylo + cs;
                    float zlo = minB + ck * cs, zhi = zlo + cs;
                    float tol = cs * 1e-3f;
                    if (v[0] < xlo-tol || v[0] > xhi+tol ||
                        v[1] < ylo-tol || v[1] > yhi+tol ||
                        v[2] < zlo-tol || v[2] > zhi+tol) {
                        ++violations;
                    }
                }
            }
        }
        check("all vertices inside their cell", violations == 0);
        if (violations > 0)
            std::cout << "    (" << violations << " violation(s))\n";
    }

    // 3. Triangle index validity
    {
        bool valid = true;
        for (const auto& tri : mesh.triangles) {
            if (tri[0] < 0 || tri[0] >= nVerts ||
                tri[1] < 0 || tri[1] >= nVerts ||
                tri[2] < 0 || tri[2] >= nVerts) {
                valid = false;
                break;
            }
        }
        check("all triangle indices valid", valid);
    }

    // 4. Winding consistency within consecutive pairs (each quad = 2 adjacent tris)
    //    For every pair (tri[2i], tri[2i+1]) emitted by the same quad,
    //    their face normals must agree (dot > -0.5).
    {
        int flipCount = 0;
        for (int t = 0; t + 1 < nTris; t += 2) {
            const auto& t0 = mesh.triangles[t];
            const auto& t1 = mesh.triangles[t+1];
            auto& v0 = mesh.vertices[t0[0]]; auto& v1 = mesh.vertices[t0[1]]; auto& v2 = mesh.vertices[t0[2]];
            auto& w0 = mesh.vertices[t1[0]]; auto& w1 = mesh.vertices[t1[1]]; auto& w2 = mesh.vertices[t1[2]];
            Eigen::Vector3f a0(v0[0],v0[1],v0[2]), a1(v1[0],v1[1],v1[2]), a2(v2[0],v2[1],v2[2]);
            Eigen::Vector3f b0(w0[0],w0[1],w0[2]), b1(w1[0],w1[1],w1[2]), b2(w2[0],w2[1],w2[2]);
            Eigen::Vector3f n0 = (a1-a0).cross(a2-a0);
            Eigen::Vector3f n1 = (b1-b0).cross(b2-b0);
            float sqn0 = n0.squaredNorm(), sqn1 = n1.squaredNorm();
            if (sqn0 < 1e-16f || sqn1 < 1e-16f) continue; // degenerate, skip
            if (n0.dot(n1) < 0.0f) ++flipCount;
        }
        check("no intra-quad winding flips", flipCount == 0);
        if (flipCount > 0)
            std::cout << "    (" << flipCount << " flip(s) detected)\n";
    }

    // 5. Approximate volume check (only meaningful at higher res)
    if (N >= 32) {
        double vol = 0.0;
        for (const auto& tri : mesh.triangles) {
            vol += triSignedVolume(mesh.vertices[tri[0]],
                                   mesh.vertices[tri[1]],
                                   mesh.vertices[tri[2]]);
        }
        vol = std::abs(vol);
        const double sphereVol = 4.0/3.0 * M_PI * 0.75 * 0.75 * 0.75; // ≈ 1.767
        double err = std::abs(vol - sphereVol) / sphereVol;
        std::cout << "  Mesh volume: " << vol << "  (sphere: " << sphereVol << ", err: " << err*100 << "%)\n";
        check("volume within 10% of sphere", err < 0.10);
    }
}

int main() {
    runTests(16);
    runTests(32);

    std::cout << "\nResults: " << g_pass << " passed, " << g_fail << " failed\n";
    return g_fail > 0 ? 1 : 0;
}
