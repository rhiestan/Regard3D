
INCLUDE_DIRECTORIES(..)

# ${CMAKE_CURRENT_SOURCE_DIR} is necessary because we will use this variable in the parent directory
# PARENT_SCOPE: Make variables visible in parent scope
SET(GL_SRC ${CMAKE_CURRENT_SOURCE_DIR}/GraphicsWindowWX.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/OSGGLCanvas.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/SharedGLContext.cpp PARENT_SCOPE)
SET(GL_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/GraphicsWindowWX.h
	${CMAKE_CURRENT_SOURCE_DIR}/OSGGLCanvas.h
	${CMAKE_CURRENT_SOURCE_DIR}/SharedGLContext.h PARENT_SCOPE)
