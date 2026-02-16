#include "mesh_sdf.h"
#include <igl/readOBJ.h>
#include <igl/signed_distance.h>
#include <igl/AABB.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_edge_normals.h>
#include <Eigen/Core>
#include <iostream>
#include <cmath>

// Module-level state
static Eigen::MatrixXd g_V;   // Vx3 double
static Eigen::MatrixXi g_F;   // Fx3 int
static igl::AABB<Eigen::MatrixXd,3> g_tree;
static Eigen::MatrixXd g_FN, g_VN, g_EN;
static Eigen::MatrixXi g_E;
static Eigen::VectorXi g_EMAP;
static bool g_loaded = false;

bool loadMeshSDF(const std::string& obj_path) {
    // Use the polygon-aware overload so n-gon faces (quads, hexagons, etc.) are read correctly.
    std::vector<std::vector<double>> Vv, TCv, Nv;
    std::vector<std::vector<int>>    Fv, FTCv, FNv;
    if (!igl::readOBJ(obj_path, Vv, TCv, Nv, Fv, FTCv, FNv)) {
        std::cerr << "Failed to load OBJ: " << obj_path << std::endl;
        return false;
    }

    // Copy vertices into Eigen matrix
    Eigen::MatrixXd V(static_cast<int>(Vv.size()), 3);
    for (int i = 0; i < static_cast<int>(Vv.size()); ++i) {
        V(i,0) = Vv[i][0]; V(i,1) = Vv[i][1]; V(i,2) = Vv[i][2];
    }

    // Fan-triangulate all polygons (handles triangles, quads, and n-gons)
    std::vector<Eigen::Vector3i> tris;
    for (const auto& poly : Fv) {
        if (poly.size() < 3) continue;
        int v0 = poly[0];
        for (int k = 1; k + 1 < static_cast<int>(poly.size()); ++k) {
            tris.push_back({v0, poly[k], poly[k+1]});
        }
    }
    if (tris.empty()) {
        std::cerr << "No valid faces in OBJ: " << obj_path << std::endl;
        return false;
    }
    Eigen::MatrixXi F(static_cast<int>(tris.size()), 3);
    for (int i = 0; i < static_cast<int>(tris.size()); ++i) {
        F.row(i) = tris[i].transpose();
    }

    // Normalise: translate centroid to origin, scale to [-0.9,0.9]^3
    Eigen::RowVector3d lo = V.colwise().minCoeff();
    Eigen::RowVector3d hi = V.colwise().maxCoeff();
    Eigen::RowVector3d centre = 0.5*(lo + hi);
    double scale = 0.9 / (0.5*(hi - lo).maxCoeff());
    V = (V.rowwise() - centre) * scale;

    g_V = V; g_F = F;
    g_tree.init(g_V, g_F);

    // Pre-compute normals needed by pseudonormal sign
    igl::per_face_normals  (g_V, g_F, g_FN);
    igl::per_vertex_normals(g_V, g_F, igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_ANGLE, g_VN);
    igl::per_edge_normals  (g_V, g_F, igl::PER_EDGE_NORMALS_WEIGHTING_TYPE_UNIFORM, g_EN, g_E, g_EMAP);

    g_loaded = true;
    return true;
}

float implicitMeshSDF(float x, float y, float z) {
    if (!g_loaded) {
        std::cerr << "Error: loadMeshSDF() must be called before implicitMeshSDF()" << std::endl;
        return 1.0f; // Return outside by default
    }
    // Use the 8-arg overload that returns the signed distance directly
    Eigen::RowVector3d p(x, y, z);
    double sd = igl::signed_distance_pseudonormal(
        g_tree, g_V, g_F, g_FN, g_VN, g_EN, g_EMAP, p);
    return static_cast<float>(sd);
}

