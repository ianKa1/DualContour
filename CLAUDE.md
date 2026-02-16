# DualContour

A C++ implementation of the Dual Contouring algorithm with interactive 3D visualization.

## Project Overview

Dual Contouring is a mesh extraction method that, unlike Marching Cubes, preserves sharp features by using Hermite data (intersection points + normals) at grid edges. This demo extracts a mesh from an implicit function over a uniform grid and visualizes the result.

## Tech Stack

- **Language**: C++17
- **Build system**: CMake (≥ 3.20)
- **Visualization**: [Polyscope](https://polyscope.run/) — lightweight, header-friendly viewer for geometry processing research
- **Linear algebra**: [Eigen3](https://eigen.tuxfamily.org/) — used for QEF (Quadric Error Function) solving and vector math
- **Package management**: vcpkg or system package manager

## Repository Structure

```
DualContour/
├── CLAUDE.md
├── README.md
├── CMakeLists.txt
├── src/
│   ├── main.cpp          # Entry point: build scene, run Polyscope loop
│   ├── implicit.h/cpp    # Implicit function definitions (sphere, torus, CSG, etc.)
│   ├── dual_contour.h/cpp# Core DC algorithm (grid, QEF solve, mesh extraction)
│   └── qef.h/cpp         # Quadric Error Function solver (SVD-based)
└── third_party/          # Vendored or CMake FetchContent dependencies
```

## Algorithm Overview

1. **Grid setup** — define a uniform 3D grid over a bounding box
2. **Sign detection** — evaluate the implicit function at each grid corner
3. **Edge intersection** — for each edge where sign changes, compute intersection point and surface normal
4. **QEF solve** — per cube: collect Hermite data and minimize `sum ||n_i · (x - p_i)||^2` via SVD to find the optimal vertex position inside the cell
5. **Mesh connectivity** — for each edge with a sign change, emit a quad connecting the 4 cells sharing that edge; split quads into tris for rendering

## Build

```bash
# Install dependencies (macOS example)
brew install cmake eigen polyscope   # or use vcpkg

cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
./build/dual_contour
```

## Key Implementation Notes

- QEF solving: use `Eigen::JacobiSVD` with a tolerance threshold to avoid degenerate cells placing vertices far outside the cell — clamp to cell bounds
- Normal estimation: use the gradient of the implicit function (analytic or central-differences)
- Grid resolution: default 32³; controllable via CLI arg or Polyscope UI slider
- Polyscope registration: register the extracted mesh with `polyscope::registerSurfaceMesh` and call `polyscope::show()`

## Development Guidelines

- Keep implicit functions as simple lambdas/functors in `implicit.h` so new shapes are easy to add
- Prefer `Eigen::Vector3d` for geometry math; avoid raw arrays
- No dynamic memory in the hot loop — pre-allocate grid arrays
- Run `cmake --build build && ./build/dual_contour` to iterate quickly
