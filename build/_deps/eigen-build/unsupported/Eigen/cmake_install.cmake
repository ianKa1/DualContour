# Install script for directory: /Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/AdolcForward"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/AlignedVector3"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/ArpackSupport"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/AutoDiff"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/BVH"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/EulerAngles"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/FFT"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/IterativeSolvers"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/KroneckerProduct"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/LevenbergMarquardt"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/MatrixFunctions"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/MoreVectorization"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/MPRealSupport"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/NonLinearOptimization"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/NumericalDiff"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/OpenGLSupport"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/Polynomials"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/Skyline"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/SparseExtra"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/SpecialFunctions"
    "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/Splines"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-src/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-build/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/eigen-build/unsupported/Eigen/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
