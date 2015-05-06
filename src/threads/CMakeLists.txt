
INCLUDE_DIRECTORIES(..)

# ${CMAKE_CURRENT_SOURCE_DIR} is necessary because we will use this variable in the parent directory
# PARENT_SCOPE: Make variables visible in parent scope
SET(THREADS_SRC ${CMAKE_CURRENT_SOURCE_DIR}/ImageInfoThread.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/PreviewGeneratorThread.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/R3DComputeMatchesThread.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/R3DFeaturesThread.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/R3DSmallTasksThread.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/R3DTriangulationThread.cpp PARENT_SCOPE)
SET(THREADS_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/ImageInfoThread.h
	${CMAKE_CURRENT_SOURCE_DIR}/PreviewGeneratorThread.h
	${CMAKE_CURRENT_SOURCE_DIR}/R3DComputeMatchesThread.h
	${CMAKE_CURRENT_SOURCE_DIR}/R3DFeaturesThread.h
	${CMAKE_CURRENT_SOURCE_DIR}/R3DSmallTasksThread.h
	${CMAKE_CURRENT_SOURCE_DIR}/R3DTriangulationThread.h PARENT_SCOPE)
