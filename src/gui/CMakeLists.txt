
INCLUDE_DIRECTORIES(..)

# ${CMAKE_CURRENT_SOURCE_DIR} is necessary because we will use this variable in the parent directory
# PARENT_SCOPE: Make variables visible in parent scope
SET(GUI_SRC ${CMAKE_CURRENT_SOURCE_DIR}/ImagePanel.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/PreviewCanvas.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/R3DNewProjectDialog.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/R3DResultDialog.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DComputeMatchesDialog.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DConsoleOutputFrame.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DDensificationDialog.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DImagePreviewDialog.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DMatchingResultsDialog.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DPictureSetDialog.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DProgressDialog.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DPropertiesDialog.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DSurfaceDialog.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DTriangulationDialog.cpp PARENT_SCOPE)
SET(GUI_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/ImagePanel.h
	${CMAKE_CURRENT_SOURCE_DIR}/PreviewCanvas.h
	${CMAKE_CURRENT_SOURCE_DIR}/R3DNewProjectDialog.h
	${CMAKE_CURRENT_SOURCE_DIR}/R3DResultDialog.h
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DComputeMatchesDialog.h
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DConsoleOutputFrame.h
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DDensificationDialog.h
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DImagePreviewDialog.h
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DMatchingResultsDialog.h
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DPictureSetDialog.h
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DProgressDialog.h
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DPropertiesDialog.h
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DSurfaceDialog.h
	${CMAKE_CURRENT_SOURCE_DIR}/Regard3DTriangulationDialog.h
	${CMAKE_CURRENT_SOURCE_DIR}/R3DImageUpdatesInterface.h PARENT_SCOPE)
