cmake_minimum_required(VERSION 2.6)
project(liop)

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

SET(ALL_SRC vl_liop.c)

SET(ALL_HEADERS vl_liop.h vl_host.h vl_qsort-def.h)

ADD_LIBRARY(liop STATIC ${ALL_SRC} ${ALL_HEADERS})
