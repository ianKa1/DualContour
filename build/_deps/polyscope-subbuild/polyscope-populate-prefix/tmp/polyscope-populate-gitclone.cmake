# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

if(EXISTS "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-subbuild/polyscope-populate-prefix/src/polyscope-populate-stamp/polyscope-populate-gitclone-lastrun.txt" AND EXISTS "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-subbuild/polyscope-populate-prefix/src/polyscope-populate-stamp/polyscope-populate-gitinfo.txt" AND
  "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-subbuild/polyscope-populate-prefix/src/polyscope-populate-stamp/polyscope-populate-gitclone-lastrun.txt" IS_NEWER_THAN "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-subbuild/polyscope-populate-prefix/src/polyscope-populate-stamp/polyscope-populate-gitinfo.txt")
  message(VERBOSE
    "Avoiding repeated git clone, stamp file is up to date: "
    "'/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-subbuild/polyscope-populate-prefix/src/polyscope-populate-stamp/polyscope-populate-gitclone-lastrun.txt'"
  )
  return()
endif()

# Even at VERBOSE level, we don't want to see the commands executed, but
# enabling them to be shown for DEBUG may be useful to help diagnose problems.
cmake_language(GET_MESSAGE_LOG_LEVEL active_log_level)
if(active_log_level MATCHES "DEBUG|TRACE")
  set(maybe_show_command COMMAND_ECHO STDOUT)
else()
  set(maybe_show_command "")
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-src"
  RESULT_VARIABLE error_code
  ${maybe_show_command}
)
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-src'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git"
            clone --no-checkout --depth 1 --no-single-branch --config "advice.detachedHead=false" "https://github.com/nmwsharp/polyscope.git" "polyscope-src"
    WORKING_DIRECTORY "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps"
    RESULT_VARIABLE error_code
    ${maybe_show_command}
  )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(NOTICE "Had to git clone more than once: ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/nmwsharp/polyscope.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git"
          checkout "v2.3.0" --
  WORKING_DIRECTORY "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-src"
  RESULT_VARIABLE error_code
  ${maybe_show_command}
)
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'v2.3.0'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git" 
            submodule update --recursive --init 
    WORKING_DIRECTORY "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-src"
    RESULT_VARIABLE error_code
    ${maybe_show_command}
  )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-src'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-subbuild/polyscope-populate-prefix/src/polyscope-populate-stamp/polyscope-populate-gitinfo.txt" "/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-subbuild/polyscope-populate-prefix/src/polyscope-populate-stamp/polyscope-populate-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  ${maybe_show_command}
)
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/Users/kaimao/Desktop/COMS6998_Geometry_processing/DualContour/build/_deps/polyscope-subbuild/polyscope-populate-prefix/src/polyscope-populate-stamp/polyscope-populate-gitclone-lastrun.txt'")
endif()
