# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

Dependencies are fetched automatically via CMake FetchContent (Eigen 3.4.0, libigl v2.5.0, Polyscope v2.3.0) — no manual install needed.

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
./build/dual_contour
```

Incremental rebuild: `cmake --build build && ./build/dual_contour`

## Architecture

### Data flow

1. `main.cpp` — registers shapes and wires up the Polyscope ImGui callback (`myCallback`). The callback drives `rebuildMesh()` on resolution/shape change.
2. `buildGrid()` in `dual_contour.cpp` — samples the scalar field at `(N+1)³` grid corners, stores results in `DCGrid::values`.
3. `dualContour()` in `dual_contour.cpp` — iterates edges for sign changes, calls `gradient()` to get normals, accumulates `HermiteSample`s per cell, calls `solveQEF()`, then emits quads (split into triangles) for sign-changing edges.
4. `solveQEF()` in `qef.cpp` — builds the least-squares system from Hermite samples and solves via `Eigen::JacobiSVD`; clamps the result to `[cellMin, cellMax]`.

### Key types

- `ScalarField = float(*)(float x, float y, float z)` — function pointer type; all implicit shapes must match this signature.
- `DCGrid` — flat `(N+1)³` array of scalar values + `N³` array mapping cell index to output vertex index (-1 if empty).
- `DCMesh` — output vertex and triangle lists using `std::array<float,3>` / `std::array<int,3>`.
- `HermiteSample` — `{point, normal}` as `Eigen::Vector3f` pairs, collected per cell before QEF solve.

### Implicit functions

`implicit.h/cpp` defines `ScalarField`-compatible free functions: `implicitSphere`, `implicitBox`, `implicitTorus`, and the central-differences `gradient()` helper.

`mesh_sdf.h/cpp` adds `implicitMeshSDF` backed by libigl's AABB + pseudonormal signed distance. Call `loadMeshSDF(path)` once before use. Mesh data (e.g. `teapot.obj`) lives in `data/` and the path is baked in at compile time via the `DATA_DIR` preprocessor define.

## Adding a new shape

1. Add a `float myShape(float x, float y, float z)` declaration to `implicit.h` and implement it in `implicit.cpp`. Convention: `f < 0` = inside, `f > 0` = outside.
2. Add it to the `g_shapes[]` and `g_shapeNames[]` arrays in `main.cpp`.

## Key implementation details

- `ScalarField` is a raw function pointer — lambdas with captures cannot be used without a wrapper.
- QEF SVD tolerance is `1e-3f` (passed as default arg to `solveQEF`); result is clamped to the cell bounding box to prevent outlier vertices.
- Normal estimation uses central differences with `eps=1e-4f` by default.
- Grid bounds default to `[-1, 1]³`; resolution defaults to 32 (slider range 8–64 in UI).
