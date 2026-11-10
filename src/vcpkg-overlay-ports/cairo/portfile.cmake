# Installs nothing: the libraries come from the system, found through
# pkg-config (cairo) by the ports that use them. See ../README.md.
set(VCPKG_POLICY_EMPTY_PACKAGE enabled)

vcpkg_find_acquire_program(PKGCONFIG)
execute_process(COMMAND "${PKGCONFIG}" --exists cairo RESULT_VARIABLE pc_result)
if(NOT pc_result EQUAL 0)
    message(FATAL_ERROR "cairo was not found by pkg-config. This overlay port uses the "
        "system's library: install its development package (Ubuntu: libcairo2-dev).")
endif()
execute_process(COMMAND "${PKGCONFIG}" --modversion cairo OUTPUT_VARIABLE pc_version OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "Using the system's cairo ${pc_version}")
