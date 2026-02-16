#include "dual_contour.h"
#include "qef.h"
#include "implicit.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>

// Flat index helpers
static inline int cornerIdx(int i, int j, int k, int N) {
    return i + (N+1)*j + (N+1)*(N+1)*k;
}

static inline int cellIdx(int ci, int cj, int ck, int N) {
    return ci + N*cj + N*N*ck;
}

// 12 edges per cell: EDGE_CORNERS[edge][0] and EDGE_CORNERS[edge][1] are corner indices
static const int EDGE_CORNERS[12][2] = {
    {0,1}, {2,3}, {4,5}, {6,7},   // X-axis edges
    {0,2}, {1,3}, {4,6}, {5,7},   // Y-axis edges
    {0,4}, {1,5}, {2,6}, {3,7}    // Z-axis edges
};

// Get corner position from corner index (0-7) within a cell
static Eigen::Vector3f getCornerPos(int corner, int ci, int cj, int ck, float minBound, float cellSize) {
    int i = ci + (corner & 1);
    int j = cj + ((corner >> 1) & 1);
    int k = ck + ((corner >> 2) & 1);
    return Eigen::Vector3f(
        minBound + i * cellSize,
        minBound + j * cellSize,
        minBound + k * cellSize
    );
}

DCGrid buildGrid(ScalarField f, int N, float minBound, float maxBound) {
    DCGrid grid;
    grid.N = N;
    grid.minBound = minBound;
    grid.maxBound = maxBound;
    grid.cellSize = (maxBound - minBound) / N;
    
    int numCorners = (N+1) * (N+1) * (N+1);
    grid.values.resize(numCorners);
    grid.vertexIndex.resize(N * N * N, -1);
    
    // Sample the scalar field at all corners
    for (int k = 0; k <= N; ++k) {
        for (int j = 0; j <= N; ++j) {
            for (int i = 0; i <= N; ++i) {
                float x = minBound + i * grid.cellSize;
                float y = minBound + j * grid.cellSize;
                float z = minBound + k * grid.cellSize;
                int idx = cornerIdx(i, j, k, N);
                grid.values[idx] = f(x, y, z);
            }
        }
    }
    
    return grid;
}

DCMesh dualContour(ScalarField f, DCGrid& grid) {
    DCMesh mesh;
    int N = grid.N;
    float minBound = grid.minBound;
    float cellSize = grid.cellSize;
    
    // Pass 1: One vertex per cell
    for (int ck = 0; ck < N; ++ck) {
        for (int cj = 0; cj < N; ++cj) {
            for (int ci = 0; ci < N; ++ci) {
                // Get corner values for this cell
                float cornerVals[8];
                for (int c = 0; c < 8; ++c) {
                    int i = ci + (c & 1);
                    int j = cj + ((c >> 1) & 1);
                    int k = ck + ((c >> 2) & 1);
                    int idx = cornerIdx(i, j, k, N);
                    cornerVals[c] = grid.values[idx];
                }
                
                // Check all 12 edges for sign changes
                std::vector<HermiteSample> samples;
                for (int e = 0; e < 12; ++e) {
                    int c0 = EDGE_CORNERS[e][0];
                    int c1 = EDGE_CORNERS[e][1];
                    float f0 = cornerVals[c0];
                    float f1 = cornerVals[c1];
                    
                    if ((f0 < 0) != (f1 < 0)) {  // Sign change
                        // Compute intersection point
                        float t = -f0 / (f1 - f0);
                        Eigen::Vector3f p0 = getCornerPos(c0, ci, cj, ck, minBound, cellSize);
                        Eigen::Vector3f p1 = getCornerPos(c1, ci, cj, ck, minBound, cellSize);
                        Eigen::Vector3f p = p0 + t * (p1 - p0);
                        
                        // Compute normal via gradient
                        Eigen::Vector3f n = gradient(f, p.x(), p.y(), p.z());
                        float len = n.norm();
                        if (len > 1e-6f) {
                            n /= len;
                        } else {
                            n = Eigen::Vector3f(1, 0, 0);  // Fallback
                        }
                        
                        samples.push_back({p, n});
                    }
                }
                
                // If we have samples, solve QEF and add vertex
                if (!samples.empty()) {
                    Eigen::Vector3f cellMin(minBound + ci * cellSize,
                                           minBound + cj * cellSize,
                                           minBound + ck * cellSize);
                    Eigen::Vector3f cellMax(minBound + (ci+1) * cellSize,
                                           minBound + (cj+1) * cellSize,
                                           minBound + (ck+1) * cellSize);
                    
                    Eigen::Vector3f vertex = solveQEF(samples, cellMin, cellMax);
                    int vertexIdx = static_cast<int>(mesh.vertices.size());
                    mesh.vertices.push_back({vertex.x(), vertex.y(), vertex.z()});
                    
                    int cellIdx_val = cellIdx(ci, cj, ck, N);
                    grid.vertexIndex[cellIdx_val] = vertexIdx;
                }
            }
        }
    }
    
    auto signChange = [](float a, float b) {
        return (a < 0.0f) != (b < 0.0f);
    };

    auto fetchCellVertex = [&](int ci, int cj, int ck, int& outV) -> bool {
        if (ci < 0 || cj < 0 || ck < 0 || ci >= N || cj >= N || ck >= N) return false;
        outV = grid.vertexIndex[cellIdx(ci, cj, ck, N)];
        return outV >= 0;
    };

    auto emitQuad = [&](const int cells[4][3], const Eigen::Vector3f& edgeMid) {
        int v[4];
        for (int t = 0; t < 4; ++t) {
            if (!fetchCellVertex(cells[t][0], cells[t][1], cells[t][2], v[t])) {
                return;
            }
        }

        const Eigen::Vector3f p0(mesh.vertices[v[0]][0], mesh.vertices[v[0]][1], mesh.vertices[v[0]][2]);
        const Eigen::Vector3f p1(mesh.vertices[v[1]][0], mesh.vertices[v[1]][1], mesh.vertices[v[1]][2]);
        const Eigen::Vector3f p2(mesh.vertices[v[2]][0], mesh.vertices[v[2]][1], mesh.vertices[v[2]][2]);
        Eigen::Vector3f n = (p1 - p0).cross(p2 - p0);

        Eigen::Vector3f g = gradient(f, edgeMid.x(), edgeMid.y(), edgeMid.z());
        if (g.norm() > 1e-8f && n.norm() > 1e-8f) {
            g.normalize();
            // Keep winding so face normal points with the SDF gradient (outward).
            if (n.dot(g) < 0.0f) {
                std::swap(v[1], v[3]);
            }
        }

        mesh.triangles.push_back({v[0], v[1], v[2]});
        mesh.triangles.push_back({v[0], v[2], v[3]});
    };

    // Pass 2: emit one quad for each sign-changing grid edge.
    // X-edges (i->i+1), shared by 4 cells around Y/Z.
    for (int k = 0; k <= N; ++k) {
        for (int j = 0; j <= N; ++j) {
            for (int i = 0; i < N; ++i) {
                float f0 = grid.values[cornerIdx(i, j, k, N)];
                float f1 = grid.values[cornerIdx(i + 1, j, k, N)];
                if (!signChange(f0, f1) || j == 0 || k == 0) continue;

                const int cells[4][3] = {
                    {i, j - 1, k - 1},
                    {i, j,     k - 1},
                    {i, j,     k},
                    {i, j - 1, k}
                };
                const Eigen::Vector3f edgeMid(
                    minBound + (static_cast<float>(i) + 0.5f) * cellSize,
                    minBound + static_cast<float>(j) * cellSize,
                    minBound + static_cast<float>(k) * cellSize);
                emitQuad(cells, edgeMid);
            }
        }
    }

    // Y-edges (j->j+1), shared by 4 cells around X/Z.
    for (int k = 0; k <= N; ++k) {
        for (int j = 0; j < N; ++j) {
            for (int i = 0; i <= N; ++i) {
                float f0 = grid.values[cornerIdx(i, j, k, N)];
                float f1 = grid.values[cornerIdx(i, j + 1, k, N)];
                if (!signChange(f0, f1) || i == 0 || k == 0) continue;

                const int cells[4][3] = {
                    {i - 1, j, k - 1},
                    {i,     j, k - 1},
                    {i,     j, k},
                    {i - 1, j, k}
                };
                const Eigen::Vector3f edgeMid(
                    minBound + static_cast<float>(i) * cellSize,
                    minBound + (static_cast<float>(j) + 0.5f) * cellSize,
                    minBound + static_cast<float>(k) * cellSize);
                emitQuad(cells, edgeMid);
            }
        }
    }

    // Z-edges (k->k+1), shared by 4 cells around X/Y.
    for (int k = 0; k < N; ++k) {
        for (int j = 0; j <= N; ++j) {
            for (int i = 0; i <= N; ++i) {
                float f0 = grid.values[cornerIdx(i, j, k, N)];
                float f1 = grid.values[cornerIdx(i, j, k + 1, N)];
                if (!signChange(f0, f1) || i == 0 || j == 0) continue;

                const int cells[4][3] = {
                    {i - 1, j - 1, k},
                    {i,     j - 1, k},
                    {i,     j,     k},
                    {i - 1, j,     k}
                };
                const Eigen::Vector3f edgeMid(
                    minBound + static_cast<float>(i) * cellSize,
                    minBound + static_cast<float>(j) * cellSize,
                    minBound + (static_cast<float>(k) + 0.5f) * cellSize);
                emitQuad(cells, edgeMid);
            }
        }
    }

    // Final pass: remove degenerate triangles and enforce consistent outward winding.
    std::vector<std::array<int, 3>> cleanTriangles;
    cleanTriangles.reserve(mesh.triangles.size());
    for (const auto& tri : mesh.triangles) {
        const Eigen::Vector3f p0(mesh.vertices[tri[0]][0], mesh.vertices[tri[0]][1], mesh.vertices[tri[0]][2]);
        const Eigen::Vector3f p1(mesh.vertices[tri[1]][0], mesh.vertices[tri[1]][1], mesh.vertices[tri[1]][2]);
        const Eigen::Vector3f p2(mesh.vertices[tri[2]][0], mesh.vertices[tri[2]][1], mesh.vertices[tri[2]][2]);
        Eigen::Vector3f n = (p1 - p0).cross(p2 - p0);
        if (n.squaredNorm() < 1e-12f) {
            continue;
        }

        cleanTriangles.push_back(tri);
    }
    mesh.triangles.swap(cleanTriangles);
    
    return mesh;
}

