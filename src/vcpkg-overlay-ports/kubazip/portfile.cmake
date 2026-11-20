# Regard3D overlay: the port from the pinned baseline, with only the SHA512 changed.
# GitHub's archive of v0.3.14 changed on 2026-11-20 (microsoft/vcpkg#55200), so the
# baseline's e35df05d... no longer matches. The new archive has the same files as
# the v0.3.14 tag (9aa49a35).
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO kuba--/zip
    REF "v${VERSION}"
    SHA512 626d1dbc67e797b21980019931f098969761d69eea88aed881882a832d10c28e782e9c0c2445dcc8485b84744286a029fd64da25e570e47af19d74135e2919b1
    HEAD_REF master
    PATCHES
        fix-name-conflict.diff
        disable-werror.patch
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DCMAKE_DISABLE_TESTING=ON
)

vcpkg_cmake_install()
vcpkg_cmake_config_fixup(CONFIG_PATH "lib/cmake/zip" PACKAGE_NAME "zip-kuba--")

if(VCPKG_LIBRARY_LINKAGE STREQUAL "dynamic")
    vcpkg_replace_string("${CURRENT_PACKAGES_DIR}/include/kubazip/zip/zip.h" "#ifndef ZIP_SHARED" "#if 0")
endif()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")

# legacy polyfill
file(INSTALL "${CURRENT_PORT_DIR}/kubazipConfig.cmake" "${CURRENT_PORT_DIR}/usage" DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}")

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE.txt")
