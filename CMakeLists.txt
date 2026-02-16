cmake_minimum_required(VERSION 3.20)
project(DualContour LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# Eigen
FetchContent_Declare(eigen GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git GIT_TAG 3.4.0 GIT_SHALLOW TRUE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING  OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen)

# libigl (header-only core — no extra modules needed)
FetchContent_Declare(libigl
  GIT_REPOSITORY https://github.com/libigl/libigl.git
  GIT_TAG v2.5.0
  GIT_SHALLOW TRUE)
set(LIBIGL_USE_STATIC_LIBRARY OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(libigl)

# Polyscope
FetchContent_Declare(polyscope GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git GIT_TAG v2.3.0 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(polyscope)

add_executable(dual_contour
  src/main.cpp
  src/implicit.cpp
  src/mesh_sdf.cpp
  src/qef.cpp
  src/dual_contour.cpp)
target_include_directories(dual_contour PRIVATE src)
target_link_libraries(dual_contour PRIVATE polyscope Eigen3::Eigen igl::core)
target_compile_options(dual_contour PRIVATE -Wall -Wextra -O2)

# Bake the data path so mesh_sdf.cpp can find teapot.obj at runtime
target_compile_definitions(dual_contour PRIVATE
  DATA_DIR="${CMAKE_SOURCE_DIR}/data")

