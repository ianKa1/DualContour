# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-src")
  file(MAKE_DIRECTORY "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-src")
endif()
file(MAKE_DIRECTORY
  "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-build"
  "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-subbuild/polyscope-populate-prefix"
  "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-subbuild/polyscope-populate-prefix/tmp"
  "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-subbuild/polyscope-populate-prefix/src/polyscope-populate-stamp"
  "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-subbuild/polyscope-populate-prefix/src"
  "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-subbuild/polyscope-populate-prefix/src/polyscope-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-subbuild/polyscope-populate-prefix/src/polyscope-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-subbuild/polyscope-populate-prefix/src/polyscope-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
