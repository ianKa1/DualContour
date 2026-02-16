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
    
    // Pass 2: Quad emission
    for (int ck = 0; ck < N; ++ck) {
        for (int cj = 0; cj < N; ++cj) {
            for (int ci = 0; ci < N; ++ci) {
                int cellIdx_val = cellIdx(ci, cj, ck, N);
                if (grid.vertexIndex[cellIdx_val] == -1) continue;
                
                // Check +X edge (requires cj>=1, ck>=1)
                if (cj >= 1 && ck >= 1) {
                    int i = ci;
                    int j = cj;
                    int k = ck;
                    int idx0 = cornerIdx(i, j, k, N);
                    int idx1 = cornerIdx(i+1, j, k, N);
                    float f0 = grid.values[idx0];
                    float f1 = grid.values[idx1];
                    
                    if ((f0 < 0) != (f1 < 0)) {
                        // Get the 4 sharing cells
                        int cells[4][3] = {
                            {ci, cj, ck},
                            {ci, cj-1, ck},
                            {ci, cj, ck-1},
                            {ci, cj-1, ck-1}
                        };
                        int vIndices[4];
                        bool valid = true;
                        for (int c = 0; c < 4; ++c) {
                            int cci = cells[c][0], ccj = cells[c][1], cck = cells[c][2];
                            if (cci < 0 || ccj < 0 || cck < 0 || cci >= N || ccj >= N || cck >= N) {
                                valid = false;
                                break;
                            }
                            int cidx = cellIdx(cci, ccj, cck, N);
                            vIndices[c] = grid.vertexIndex[cidx];
                            if (vIndices[c] == -1) {
                                valid = false;
                                break;
                            }
                        }
                        
                        if (valid) {
                            // Create quad as two triangles: (0,1,2) and (1,3,2)
                            int va = vIndices[0], vb = vIndices[1], vc = vIndices[2], vd = vIndices[3];
                            
                            // Compute face normal for winding check
                            Eigen::Vector3f a(mesh.vertices[va][0], mesh.vertices[va][1], mesh.vertices[va][2]);
                            Eigen::Vector3f b(mesh.vertices[vb][0], mesh.vertices[vb][1], mesh.vertices[vb][2]);
                            Eigen::Vector3f d(mesh.vertices[vd][0], mesh.vertices[vd][1], mesh.vertices[vd][2]);
                            Eigen::Vector3f faceNormal = (b - a).cross(d - a);
                            
                            // Edge midpoint for gradient check
                            float midX = minBound + (i + 0.5f) * cellSize;
                            float midY = minBound + j * cellSize;
                            float midZ = minBound + k * cellSize;
                            Eigen::Vector3f grad = gradient(f, midX, midY, midZ);
                            float gradLen = grad.norm();
                            if (gradLen > 1e-6f) grad /= gradLen;
                            
                            // Check if we need to flip (f_low < 0 means inside, outward opposes gradient)
                            float f_low = std::min(f0, f1);
                            bool flip = (f_low < 0) ? (faceNormal.dot(grad) > 0) : (faceNormal.dot(grad) < 0);
                            
                            if (flip) {
                                mesh.triangles.push_back({va, vc, vb});
                                mesh.triangles.push_back({vb, vc, vd});
                            } else {
                                mesh.triangles.push_back({va, vb, vc});
                                mesh.triangles.push_back({vb, vd, vc});
                            }
                        }
                    }
                }
                
                // Check +Y edge (requires ci>=1, ck>=1)
                if (ci >= 1 && ck >= 1) {
                    int i = ci;
                    int j = cj;
                    int k = ck;
                    int idx0 = cornerIdx(i, j, k, N);
                    int idx1 = cornerIdx(i, j+1, k, N);
                    float f0 = grid.values[idx0];
                    float f1 = grid.values[idx1];
                    
                    if ((f0 < 0) != (f1 < 0)) {
                        int cells[4][3] = {
                            {ci, cj, ck},
                            {ci-1, cj, ck},
                            {ci, cj, ck-1},
                            {ci-1, cj, ck-1}
                        };
                        int vIndices[4];
                        bool valid = true;
                        for (int c = 0; c < 4; ++c) {
                            int cci = cells[c][0], ccj = cells[c][1], cck = cells[c][2];
                            if (cci < 0 || ccj < 0 || cck < 0 || cci >= N || ccj >= N || cck >= N) {
                                valid = false;
                                break;
                            }
                            int cidx = cellIdx(cci, ccj, cck, N);
                            vIndices[c] = grid.vertexIndex[cidx];
                            if (vIndices[c] == -1) {
                                valid = false;
                                break;
                            }
                        }
                        
                        if (valid) {
                            int va = vIndices[0], vb = vIndices[1], vc = vIndices[2], vd = vIndices[3];
                            Eigen::Vector3f a(mesh.vertices[va][0], mesh.vertices[va][1], mesh.vertices[va][2]);
                            Eigen::Vector3f b(mesh.vertices[vb][0], mesh.vertices[vb][1], mesh.vertices[vb][2]);
                            Eigen::Vector3f d(mesh.vertices[vd][0], mesh.vertices[vd][1], mesh.vertices[vd][2]);
                            Eigen::Vector3f faceNormal = (b - a).cross(d - a);
                            
                            float midX = minBound + i * cellSize;
                            float midY = minBound + (j + 0.5f) * cellSize;
                            float midZ = minBound + k * cellSize;
                            Eigen::Vector3f grad = gradient(f, midX, midY, midZ);
                            float gradLen = grad.norm();
                            if (gradLen > 1e-6f) grad /= gradLen;
                            
                            float f_low = std::min(f0, f1);
                            bool flip = (f_low < 0) ? (faceNormal.dot(grad) > 0) : (faceNormal.dot(grad) < 0);
                            
                            if (flip) {
                                mesh.triangles.push_back({va, vc, vb});
                                mesh.triangles.push_back({vb, vc, vd});
                            } else {
                                mesh.triangles.push_back({va, vb, vc});
                                mesh.triangles.push_back({vb, vd, vc});
                            }
                        }
                    }
                }
                
                // Check +Z edge (requires ci>=1, cj>=1)
                if (ci >= 1 && cj >= 1) {
                    int i = ci;
                    int j = cj;
                    int k = ck;
                    int idx0 = cornerIdx(i, j, k, N);
                    int idx1 = cornerIdx(i, j, k+1, N);
                    float f0 = grid.values[idx0];
                    float f1 = grid.values[idx1];
                    
                    if ((f0 < 0) != (f1 < 0)) {
                        int cells[4][3] = {
                            {ci, cj, ck},
                            {ci-1, cj, ck},
                            {ci, cj-1, ck},
                            {ci-1, cj-1, ck}
                        };
                        int vIndices[4];
                        bool valid = true;
                        for (int c = 0; c < 4; ++c) {
                            int cci = cells[c][0], ccj = cells[c][1], cck = cells[c][2];
                            if (cci < 0 || ccj < 0 || cck < 0 || cci >= N || ccj >= N || cck >= N) {
                                valid = false;
                                break;
                            }
                            int cidx = cellIdx(cci, ccj, cck, N);
                            vIndices[c] = grid.vertexIndex[cidx];
                            if (vIndices[c] == -1) {
                                valid = false;
                                break;
                            }
                        }
                        
                        if (valid) {
                            int va = vIndices[0], vb = vIndices[1], vc = vIndices[2], vd = vIndices[3];
                            Eigen::Vector3f a(mesh.vertices[va][0], mesh.vertices[va][1], mesh.vertices[va][2]);
                            Eigen::Vector3f b(mesh.vertices[vb][0], mesh.vertices[vb][1], mesh.vertices[vb][2]);
                            Eigen::Vector3f d(mesh.vertices[vd][0], mesh.vertices[vd][1], mesh.vertices[vd][2]);
                            Eigen::Vector3f faceNormal = (b - a).cross(d - a);
                            
                            float midX = minBound + i * cellSize;
                            float midY = minBound + j * cellSize;
                            float midZ = minBound + (k + 0.5f) * cellSize;
                            Eigen::Vector3f grad = gradient(f, midX, midY, midZ);
                            float gradLen = grad.norm();
                            if (gradLen > 1e-6f) grad /= gradLen;
                            
                            float f_low = std::min(f0, f1);
                            bool flip = (f_low < 0) ? (faceNormal.dot(grad) > 0) : (faceNormal.dot(grad) < 0);
                            
                            if (flip) {
                                mesh.triangles.push_back({va, vc, vb});
                                mesh.triangles.push_back({vb, vc, vd});
                            } else {
                                mesh.triangles.push_back({va, vb, vc});
                                mesh.triangles.push_back({vb, vd, vc});
                            }
                        }
                    }
                }
            }
        }
    }
    
    return mesh;
}

