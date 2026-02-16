#pragma once
#include "implicit.h"
#include <vector>
#include <array>

struct DCGrid {
    int N;
    float minBound, maxBound, cellSize;
    std::vector<float> values;       // (N+1)^3 scalar samples
    std::vector<int>   vertexIndex;  // N^3, -1 if no vertex in cell
};

struct DCMesh {
    std::vector<std::array<float,3>> vertices;
    std::vector<std::array<int,3>>   triangles;
};

DCGrid buildGrid(ScalarField f, int N, float minBound=-1.f, float maxBound=1.f);
DCMesh dualContour(ScalarField f, DCGrid& grid);

