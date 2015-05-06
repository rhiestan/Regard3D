# - Find the VLFEAT library (http://www.vlfeat.org/)
# If VLFEAT is installed in an unusual location, define VLFEAT_ROOT.
# This module defines the following variables:
#  VLFEAT_INCLUDE_DIR, where to find the VLFEAT include files
#  VLFEAT_LIBRARIES, the libraries needed to use VLFEAT.
#  VLFEAT_FOUND, If false, do not try to use VLFEAT.

#=============================================================================
# Copyright 2014 Roman Hiestand
#
# Distributed under the MIT License.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================

FIND_PATH(VLFEAT_INCLUDE_DIR vlfeat/generic.h
	PATHS
		/usr/local/include
		/usr/include
		${VLFEAT_ROOT}/include
	DOC "Location of VLFEAT Headers"
)

FIND_LIBRARY(VLFEAT_LIBRARY
	NAMES vlfeat libvlfeat
	PATHS /usr/lib /usr/local/lib ${VLFEAT_ROOT}/lib
)

# Handle the QUIETLY and REQUIRED arguments and set VLFEAT_FOUND to TRUE
INCLUDE(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VLFEAT DEFAULT_MSG
	VLFEAT_LIBRARY VLFEAT_INCLUDE_DIR)

IF(VLFEAT_FOUND)
	SET(VLFEAT_LIBRARIES ${VLFEAT_LIBRARY})
ENDIF(VLFEAT_FOUND)

MARK_AS_ADVANCED(VLFEAT_INCLUDE_DIR VLFEAT_LIBRARY VLFEAT_LIBRARIES)
