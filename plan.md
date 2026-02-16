# Dual Contouring C++ Demo — Implementation Plan

## Context
Bootstrap a complete, working C++ Dual Contouring demo from an empty repo. The algorithm extracts a triangle mesh from an implicit (signed-distance) function by placing one vertex per grid cell via QEF minimization and connecting cells sharing a sign-change edge. The result is rendered interactively with Polyscope. This is a self-contained educational demo with three built-in shapes and a resolution slider.

---

## Files to Create

```
DualContour/
├── CMakeLists.txt
└── src/
    ├── main.cpp
    ├── implicit.h
    ├── implicit.cpp
    ├── qef.h
    ├── qef.cpp
    ├── dual_contour.h
    └── dual_contour.cpp
```

---

## Step 1 — CMakeLists.txt

Use CMake FetchContent (no vcpkg, no system installs required):

- **Eigen 3.4.0** from `https://gitlab.com/libeigen/eigen.git` — provides `Eigen3::Eigen` INTERFACE target
- **Polyscope v2.3.0** from `https://github.com/nmwsharp/polyscope.git` — provides `polyscope` target (bundles glfw, imgui, glm)

```cmake
cmake_minimum_required(VERSION 3.20)
project(DualContour LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
# Eigen
FetchContent_Declare(eigen GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git GIT_TAG 3.4.0 GIT_SHALLOW TRUE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen)
# Polyscope
FetchContent_Declare(polyscope GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git GIT_TAG v2.3.0 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(polyscope)

add_executable(dual_contour
  src/main.cpp src/implicit.cpp src/qef.cpp src/dual_contour.cpp)
target_include_directories(dual_contour PRIVATE src)
target_link_libraries(dual_contour PRIVATE polyscope Eigen3::Eigen)
target_compile_options(dual_contour PRIVATE -Wall -Wextra -O2)
```

---

## Step 2 — `implicit.h` / `implicit.cpp`

**Convention**: `f < 0` = inside, `f > 0` = outside. Zero level-set is the surface.

```cpp
// implicit.h
using ScalarField = float(*)(float x, float y, float z);
float implicitSphere(float x, float y, float z);  // r=0.75
float implicitBox   (float x, float y, float z);  // half-extents (0.6,0.45,0.5)
float implicitTorus (float x, float y, float z);  // R=0.6, r=0.25 in XZ-plane
Eigen::Vector3f gradient(ScalarField f, float x, float y, float z, float eps=1e-4f);
```

All three shapes fit in `[-1,1]^3`. Gradient uses central differences.

---

## Step 3 — `qef.h` / `qef.cpp`

Minimise `E(x) = Σ (n_i · (x − p_i))²` → `||Ax − b||²` where row i of A is `n_i^T`, `b_i = n_i · p_i`.

Key implementation details:
- Translate system to mass-point (average of intersection points) before building A,b — improves conditioning
- Solve with `Eigen::JacobiSVD` (ComputeThinU | ComputeThinV), `setThreshold(1e-3 * maxSV)` to zero near-degenerate singular values
- Translate result back, then clamp `x` to `[cellMin, cellMax]`
- Fallback: if samples list is empty, return mass-point clamped to cell (never actually called, but defensive)

```cpp
struct HermiteSample { Eigen::Vector3f point, normal; };
Eigen::Vector3f solveQEF(const std::vector<HermiteSample>& samples,
                         const Eigen::Vector3f& cellMin,
                         const Eigen::Vector3f& cellMax,
                         float svdThreshold = 1e-3f);
```

---

## Step 4 — `dual_contour.h` — Data Structures

```cpp
struct DCGrid {
    int N;  float minBound, maxBound, cellSize;
    std::vector<float> values;       // (N+1)^3 scalar samples
    std::vector<int>   vertexIndex;  // N^3, -1 if no vertex in cell
};
struct DCMesh {
    std::vector<std::array<float,3>> vertices;
    std::vector<std::array<int,3>>   triangles;
};
DCGrid buildGrid(ScalarField f, int N, float minBound=-1.f, float maxBound=1.f);
DCMesh dualContour(ScalarField f, const DCGrid& grid);
```

**Flat index helpers:**
- Corner: `idx(i,j,k) = i + (N+1)*j + (N+1)^2*k`
- Cell:   `idx(ci,cj,ck) = ci + N*cj + N^2*ck`

---

## Step 5 — `dual_contour.cpp` — Core Algorithm

### 12-Edge Table Per Cell

```
Corners: c = (c&1, (c>>1)&1, (c>>2)&1)  for c in 0..7
EDGE_CORNERS[12][2] = {
  {0,1},{2,3},{4,5},{6,7},   // X-axis
  {0,2},{1,3},{4,6},{5,7},   // Y-axis
  {0,4},{1,5},{2,6},{3,7}    // Z-axis
}
```

### Pass 1: One Vertex Per Cell

For each cell `(ci,cj,ck)`:
1. Check all 12 edges; for each sign-change edge compute `t = -f0/(f1-f0)`, intersection point, and `gradient(f,...)` as normal. Push `HermiteSample`.
2. If non-empty: call `solveQEF(samples, cellMin, cellMax)`, push to `mesh.vertices`, store index in `grid.vertexIndex`.

### Pass 2: Quad Emission

Iterate cells, check the 3 canonical edges per cell (+X, +Y, +Z from min-corner). For each sign-change edge, the 4 sharing cells are:

```
+X edge at (ci,cj,ck) [requires cj>=1, ck>=1]:
  cells: (ci,cj,ck), (ci,cj-1,ck), (ci,cj,ck-1), (ci,cj-1,ck-1)

+Y edge at (ci,cj,ck) [requires ci>=1, ck>=1]:
  cells: (ci,cj,ck), (ci-1,cj,ck), (ci,cj,ck-1), (ci-1,cj,ck-1)

+Z edge at (ci,cj,ck) [requires ci>=1, cj>=1]:
  cells: (ci,cj,ck), (ci-1,cj,ck), (ci,cj-1,ck), (ci-1,cj-1,ck)
```

Skip quads where any of the 4 cells has `vertexIndex == -1` or is out-of-bounds.

**Winding**: Compute face normal of first candidate triangle (va,vb,vd). Compare dot with `gradient(f, edgeMidpoint)`. If `f_low < 0` (inside), outward opposes gradient — flip accordingly.

---

## Step 6 — `main.cpp` — Polyscope UI

```cpp
// Global state
static int g_resolution = 32;
static int g_shapeIdx   = 0;
static ScalarField g_shapes[] = { implicitSphere, implicitBox, implicitTorus };

void rebuildMesh() {
    // buildGrid + dualContour + polyscope::removeSurfaceMesh + registerSurfaceMesh
}
void myCallback() {
    // ImGui::SliderInt("Resolution", 8-64), ImGui::Combo("Shape", ...), stats
    // Call rebuildMesh() on any change
}
int main() {
    polyscope::init();
    polyscope::state::userCallback = myCallback;
    rebuildMesh();
    polyscope::show();
}
```

Key Polyscope API:
- `polyscope::init()` before any calls
- `polyscope::hasSurfaceMesh(name)` + `removeSurfaceMesh(name)` before re-registering
- `polyscope::registerSurfaceMesh(name, vertices, triangles)` accepts `vector<array<float,3>>` and `vector<array<int,3>>`
- `polyscope::show()` blocks until window close

---

## Implementation Order

| Step | File | Notes |
|------|------|-------|
| 1 | `CMakeLists.txt` | FetchContent; verify `cmake -B build` succeeds |
| 2 | `implicit.h/cpp` | Three SDFs + gradient |
| 3 | `qef.h/cpp` | JacobiSVD solve; test with hand-crafted samples |
| 4 | `dual_contour.h` | Structs only |
| 5 | `dual_contour.cpp` | buildGrid, Pass 1, Pass 2 |
| 6 | `main.cpp` | Polyscope init + ImGui callback |

---

## Verification

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
./build/dual_contour
```

Expected: Polyscope window opens showing a smooth sphere mesh (default 32³ grid). Changing shape to "Torus" produces a torus with a visible hole. Dragging the resolution slider to 64 increases mesh density visibly. No degenerate / flipped faces on sphere or box.
