cmake_minimum_required(VERSION 2.6)
project(AKAZE)

# Find external libraries and dependencies
#find_package(OpenCV REQUIRED)

#if (NOT APPLE)
#  find_package(OpenMP)
#endif ()

# We don't want that AKAZE uses OpenMP, we use it on top level
REMOVE_DEFINITIONS(-D_OPENMP -DUSE_OPENMP)

# ============================================================================ #
# Compilation flags

IF(MSVC)
	ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)
	SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /MP" )
	SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP" )
	SET( CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Ox /Ob2 /Oi /Ot /GL /GS- /fp:fast /MP" )
	SET( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox /Ob2 /Oi /Ot /GL /GS- /fp:fast /MP" )
ENDIF(MSVC)
IF(CMAKE_COMPILER_IS_GNUCC)
	SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pedantic -Wextra -Wunused-variable" )
	SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra -Wunused-variable" )
	SET( CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O2 -Wall -pedantic -msse2 -mfpmath=sse -ffast-math -fsched-spec-load -fomit-frame-pointer" )
	SET( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -Wall -pedantic -msse2 -mfpmath=sse -ffast-math -fsched-spec-load -fomit-frame-pointer" )
ENDIF(CMAKE_COMPILER_IS_GNUCC)

# ============================================================================ #
# Create projects
include_directories("lib")

# AKAZE libraries
set(AKAZE_SRCS
    lib/AKAZEConfig.h
    lib/AKAZE.h                  lib/AKAZE.cpp
    lib/fed.h                    lib/fed.cpp
    lib/nldiffusion_functions.h  lib/nldiffusion_functions.cpp
    lib/utils.h                  lib/utils.cpp)

add_library(AKAZE STATIC ${AKAZE_SRCS})
