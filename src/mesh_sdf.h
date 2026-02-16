#pragma once
#include <string>

// Load an OBJ, build an AABB tree, normalise to [-0.9,0.9]^3.
// Returns false and prints an error if loading fails.
// Must be called once before implicitMeshSDF is used.
bool loadMeshSDF(const std::string& obj_path);

// ScalarField-compatible function: queries the pre-loaded mesh SDF.
// f < 0 = inside, f > 0 = outside (pseudonormal sign).
float implicitMeshSDF(float x, float y, float z);


