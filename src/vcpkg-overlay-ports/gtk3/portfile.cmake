# Installs nothing: the libraries come from the system, found through
# pkg-config (gtk+-3.0) by the ports that use them. See ../README.md.
set(VCPKG_POLICY_EMPTY_PACKAGE enabled)

vcpkg_find_acquire_program(PKGCONFIG)
execute_process(COMMAND "${PKGCONFIG}" --exists gtk+-3.0 RESULT_VARIABLE pc_result)
if(NOT pc_result EQUAL 0)
    message(FATAL_ERROR "gtk+-3.0 was not found by pkg-config. This overlay port uses the "
        "system's library: install its development package (Ubuntu: libgtk-3-dev).")
endif()
execute_process(COMMAND "${PKGCONFIG}" --modversion gtk+-3.0 OUTPUT_VARIABLE pc_version OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "Using the system's gtk+-3.0 ${pc_version}")
