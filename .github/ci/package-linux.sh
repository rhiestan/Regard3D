#!/usr/bin/env bash
# Builds dist/$PKG_NAME.AppImage from the AppDir in $STAGE_DIR with
# appimagetool, which embeds the static AppImage runtime (it needs no
# libfuse2 on the host, only fusermount).
set -euo pipefail

: "${STAGE_DIR:?}" "${PKG_NAME:?}"
mkdir -p dist
work="$RUNNER_TEMP/package"
rm -rf "$work"
mkdir -p "$work"

# What appimagetool expects at the top of the AppDir. AppRun is a symbolic
# link, so that /proc/self/exe is usr/bin/Regard3D and wxStandardPaths takes
# usr/ for the installation prefix.
ln -sf usr/bin/Regard3D "$STAGE_DIR/AppRun"
cp "$STAGE_DIR/usr/share/applications/regard3d.desktop" "$STAGE_DIR/"
cp "$STAGE_DIR/usr/share/icons/hicolor/256x256/apps/regard3d.png" "$STAGE_DIR/"
ln -sf regard3d.png "$STAGE_DIR/.DirIcon"

tool="$work/appimagetool-x86_64.AppImage"
curl -sSfL -o "$tool" \
	https://github.com/AppImage/appimagetool/releases/download/continuous/appimagetool-x86_64.AppImage
chmod +x "$tool"

# The runner has no FUSE for appimagetool's own AppImage
APPIMAGE_EXTRACT_AND_RUN=1 ARCH=x86_64 "$tool" --no-appstream \
	"$STAGE_DIR" "dist/$PKG_NAME.AppImage"

ls -l dist
