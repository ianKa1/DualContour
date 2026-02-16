# Debug Plan: Fix Visual Artifacts + Add Unit Tests

## Context

The teapot reconstruction shows spiky geometry — triangles pointing outward from the surface. Analysis of the three components reveals a primary bug in `dual_contour.cpp`: **double winding correction**. Both `emitQuad` and the final pass independently try to reorient each triangle via gradient comparison; when the gradient direction differs between the edge midpoint and a triangle's centroid, two triangles of the same quad end up with opposite winding. Polyscope renders both faces, causing the "spiky" visual.

Separately, we need test executables for: mesh SDF loading, dual contouring correctness, and QEF solving.

---

## Bug Root Cause Analysis

### Bug 1 (PRIMARY — causes spiky triangles): Double winding correction in `dual_contour.cpp`

**In `emitQuad` (lines 155–161):** computes gradient at `edgeMid`, swaps `v[1]↔v[3]` if `n·g < 0`.

**In the final pass (lines 250–255):** for every triangle independently, computes gradient at the triangle's centroid, swaps `outTri[1]↔outTri[2]` if `n·g < 0`.

**Why this causes artifacts:** For a non-planar quad, the gradient at the edge midpoint and the gradient at each triangle's centroid can disagree. This causes `emitQuad` to correctly orient the quad, then the final pass to flip one of the two triangles back, making the two triangles of the same quad face opposite directions.

**Fix:** Remove the gradient-based winding flip in the final pass (lines 252–255). Keep only the degenerate triangle filter (line 246–248). The `emitQuad` gradient check remains as the single source of winding truth.

---

## Files to Modify

| File | Change |
|------|--------|
| `src/dual_contour.cpp` | **Lines 250–255**: Remove the `g = gradient(...)` and conditional swap block; keep only degenerate triangle skip |
| `tests/test_qef.cpp` | New file: QEF unit tests |
| `tests/test_dual_contour.cpp` | New file: DC correctness tests |
| `tests/test_mesh_sdf.cpp` | New file: SDF sign and value tests |
| `CMakeLists.txt` | Add 3 test executables |

---

## Fix: `src/dual_contour.cpp` final pass (lines 238–258)

**Remove these lines from the final pass:**
```cpp
    const Eigen::Vector3f c = (p0 + p1 + p2) / 3.0f;
    const Eigen::Vector3f g = gradient(f, c.x(), c.y(), c.z());
    if (g.squaredNorm() > 1e-12f && n.dot(g) < 0.0f) {
        std::swap(outTri[1], outTri[2]);
    }
```

**Keep:** the degenerate-normal `continue` (line 246–248) and the `cleanTriangles.push_back(outTri)`.

---

## Test Programs

### 1. `tests/test_qef.cpp`

Test `solveQEF` with hand-crafted inputs:

| Test | Input | Expected |
|------|-------|----------|
| Single plane z=0.5, normal (0,0,1) | 4 samples at z=0.5 | vertex z ≈ 0.5, clamped to cell |
| Edge feature: two orthogonal planes | normals (1,0,0) and (0,1,0) | vertex on the edge line |
| Corner feature: three orthogonal planes | normals (1,0,0),(0,1,0),(0,0,1) | vertex at exact corner |
| Degenerate: all normals (0,0,1) | 3 parallel-plane samples | falls back toward mass point |
| Empty samples | empty vector | center of cell `(cellMin+cellMax)/2` |

All results must lie inside `[cellMin, cellMax]`.

### 2. `tests/test_dual_contour.cpp`

Use `implicitSphere` (r=0.75, known ground truth):

- **Vertex bounds**: every vertex within `[-1, 1]³`
- **Cell containment**: for every assigned cell vertex, verify it lies in `[cellMin, cellMax]`
- **Index validity**: every triangle index in `[0, mesh.vertices.size())`
- **Winding consistency**: for every quad's two triangles, verify normals are within 90° of each other (no intra-quad flip)
- **Approximate volume**: compute signed mesh volume, compare to sphere volume 4/3 π(0.75)³ ≈ 1.767 — within 10% at N=32

### 3. `tests/test_mesh_sdf.cpp`

Use `loadMeshSDF(DATA_DIR "/teapot.obj")`:

- **Load succeeds**: returns `true`
- **Interior point**: `implicitMeshSDF(0, 0, 0) < 0` (teapot body center is inside)
- **Exterior point**: `implicitMeshSDF(0, 5, 0) > 0` (far above teapot)
- **Gradient magnitude near surface**: for query points where |SDF| < 0.05, `gradient(implicitMeshSDF, x, y, z).norm()` ≈ 1.0 (SDF property)
- **Normalization**: all 8 corners of `[-0.9,0.9]³` have SDF > 0 (all outside)

---

## CMakeLists.txt additions

```cmake
# Tests (no Polyscope dependency)
foreach(test_name test_qef test_dual_contour test_mesh_sdf)
  add_executable(${test_name} tests/${test_name}.cpp
    src/qef.cpp src/implicit.cpp src/dual_contour.cpp src/mesh_sdf.cpp)
  target_include_directories(${test_name} PRIVATE src)
  target_link_libraries(${test_name} PRIVATE Eigen3::Eigen igl::core)
  target_compile_definitions(${test_name} PRIVATE DATA_DIR="${CMAKE_SOURCE_DIR}/data")
  target_compile_options(${test_name} PRIVATE -Wall -Wextra -O2)
endforeach()
```

---

## Verification

1. Build tests: `cmake --build build -j$(nproc)`
2. Run each test:
   - `./build/test_qef` — all assertions pass
   - `./build/test_dual_contour` — no cell-containment violations, consistent quad winding
   - `./build/test_mesh_sdf` — correct SDF signs at known interior/exterior points
3. Run main app `./build/dual_contour` — teapot no longer shows spiky triangles at N=32
4. Switch shapes (Sphere, Box, Torus) to confirm no regressions
