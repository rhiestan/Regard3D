cmake_minimum_required(VERSION 2.6)
project(daisy)

INCLUDE_DIRECTORIES(include)

# We don't want that daisy uses OpenMP, we use it on top level
REMOVE_DEFINITIONS(-D_OPENMP -DUSE_OPENMP)

# Use OpenCV convolution
ADD_DEFINITIONS(-DWITH_OPENCV -DWITH_OPENCV_EXTRAS)

IF(MSVC)
	ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)
	SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /MP" )
	SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP" )
	SET( CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Ox /Ob2 /Oi /Ot /GL /GS- /fp:fast /MP" )
	SET( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox /Ob2 /Oi /Ot /GL /GS- /fp:fast /MP" )
ENDIF(MSVC)
IF(CMAKE_COMPILER_IS_GNUCC)
	SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pedantic" )
	SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic" )
	SET( CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O2 -Wall -pedantic -msse2 -mfpmath=sse -ffast-math -fsched-spec-load -fomit-frame-pointer" )
	SET( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -Wall -pedantic -msse2 -mfpmath=sse -ffast-math -fsched-spec-load -fomit-frame-pointer" )
ENDIF(CMAKE_COMPILER_IS_GNUCC)

SET(ALL_SRC src/corecv.cpp src/daisy.cpp src/interaction.cpp src/progress_bar.cpp src/general.cpp)

FILE(GLOB ALL_KUTILITY_INCLUDES include/kutility/*)
SET(ALL_HEADERS include/daisy/daisy.h ${ALL_KUTILITY_INCLUDES})

# Add groups for source files in Visual Studio
SOURCE_GROUP(main FILES ${ALL_SRC})
SOURCE_GROUP(header REGULAR_EXPRESSION include/daisy/*)
SOURCE_GROUP(kutility REGULAR_EXPRESSION include/kutility/*)

ADD_LIBRARY(daisy STATIC ${ALL_SRC} ${ALL_HEADERS})
