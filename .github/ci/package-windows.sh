#!/usr/bin/env bash
# Builds the Windows packages from $STAGE_DIR into dist/:
#   $PKG_NAME-setup.exe   NSIS installer (src/install/Regard3D.nsi), which
#                         also runs the Visual C++ Redistributable
#   $PKG_NAME.zip         portable: the stage directory plus the MSVC runtime
#                         DLLs, so it runs without installing anything
set -euo pipefail

: "${STAGE_DIR:?}" "${PKG_NAME:?}" "${VCToolsRedistDir:?}"
mkdir -p dist
dist="$(cygpath -m "$PWD/dist")"
work="$(cygpath -m "$RUNNER_TEMP")/package"
rm -rf "$work"
mkdir -p "$work"

# --- Installer -------------------------------------------------------------------

# The uninstaller removes exactly the files that were installed (and the
# directories that are empty afterwards), never the whole installation
# directory, which the user may have chosen to be a shared one.
uninstall_list="$work/uninstall_files.nsh"
(
	cd "$STAGE_DIR"
	find . -type f | sed 's|^\./||' | sort \
		| sed 's|/|\\|g; s|.*|Delete "$INSTDIR\\&"|'
	find . -mindepth 1 -type d | sed 's|^\./||' | sort -r \
		| sed 's|/|\\|g; s|.*|RMDir "$INSTDIR\\&"|'
) > "$uninstall_list"

vcredist="$work/vc_redist.x64.exe"
curl -sSfL -o "$vcredist" https://aka.ms/vc14/vc_redist.x64.exe

makensis="$(command -v makensis || true)"
if [[ -z "$makensis" ]]; then
	for p in "/c/Program Files (x86)/NSIS/makensis.exe" "/c/Program Files/NSIS/makensis.exe"; do
		[[ -x "$p" ]] && makensis="$p" && break
	done
fi
if [[ -z "$makensis" ]]; then
	choco install nsis -y --no-progress > /dev/null
	makensis="/c/Program Files (x86)/NSIS/makensis.exe"
fi

# -D, not /D: Git Bash would take /DNAME=... for a path and rewrite it
"$makensis" -V3 \
	-DSTAGE_DIR="$(cygpath -w "$STAGE_DIR")" \
	-DUNINSTALL_LIST="$(cygpath -w "$uninstall_list")" \
	-DVCREDIST="$(cygpath -w "$vcredist")" \
	-DOUTFILE="$(cygpath -w "$dist/$PKG_NAME-setup.exe")" \
	src/install/Regard3D.nsi

# --- Portable zip -----------------------------------------------------------------

portable="$work/$PKG_NAME"
cp -r "$STAGE_DIR" "$portable"
redist="$(cygpath -u "$VCToolsRedistDir")"
crt=("$redist"/x64/Microsoft.VC*.CRT/*.dll)
omp=("$redist"/x64/Microsoft.VC*.OpenMP/vcomp140.dll)
for f in "${crt[@]}" "${omp[@]}"; do
	[[ -f "$f" ]] || { echo "MSVC runtime DLL not found: $f" >&2; exit 1; }
	cp "$f" "$portable/"
done
# The tools import the C++ runtime too (vcomp140 is OpenCV's only)
for d in openmvg pmvs poisson mve; do
	cp "${crt[@]}" "$portable/$d/"
done
(cd "$work" && 7z a -tzip -mx=9 "$dist/$PKG_NAME.zip" "$PKG_NAME" > /dev/null)

ls -l dist
