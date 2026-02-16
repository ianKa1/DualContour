#include "dual_contour.h"
#include "mesh_sdf.h"
#include "implicit.h"
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <imgui.h>
#include <iostream>

// Global state
static int g_resolution = 32;
static int g_shapeIdx = 0;
static ScalarField g_shapes[] = {
    implicitSphere, implicitBox, implicitTorus, implicitMeshSDF
};
static const char* g_shapeNames[] = { "Sphere", "Box", "Torus", "Teapot" };

static DCGrid g_grid;
static DCMesh g_mesh;

void rebuildMesh() {
    ScalarField f = g_shapes[g_shapeIdx];
    
    // Build grid
    g_grid = buildGrid(f, g_resolution);
    
    // Run dual contouring
    g_mesh = dualContour(f, g_grid);
    
    // Update Polyscope
    if (polyscope::hasSurfaceMesh("mesh")) {
        polyscope::removeSurfaceMesh("mesh");
    }
    
    if (!g_mesh.vertices.empty() && !g_mesh.triangles.empty()) {
        polyscope::registerSurfaceMesh("mesh", g_mesh.vertices, g_mesh.triangles);
    }
}

void myCallback() {
    ImGui::PushItemWidth(200);
    
    bool changed = false;
    
    // Resolution slider
    int oldRes = g_resolution;
    ImGui::SliderInt("Resolution", &g_resolution, 8, 128);
    if (g_resolution != oldRes) {
        changed = true;
    }
    
    // Shape combo
    int oldShape = g_shapeIdx;
    if (ImGui::Combo("Shape", &g_shapeIdx, g_shapeNames, 4)) {
        changed = true;
    }
    
    // Stats
    ImGui::Separator();
    ImGui::Text("Vertices: %zu", g_mesh.vertices.size());
    ImGui::Text("Triangles: %zu", g_mesh.triangles.size());
    
    if (changed) {
        rebuildMesh();
    }
}

int main() {
    // Load teapot mesh
    std::string teapotPath = DATA_DIR "/teapot.obj";
    if (!loadMeshSDF(teapotPath)) {
        std::cerr << "Warning: Failed to load teapot.obj. Teapot shape will not work." << std::endl;
    }
    
    // Initialize Polyscope
    polyscope::init();
    
    // Set callback
    polyscope::state::userCallback = myCallback;
    
    // Build initial mesh
    rebuildMesh();
    
    // Show window
    polyscope::show();
    
    return 0;
}

