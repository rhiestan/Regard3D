#!/usr/bin/env bash
# Lays out the AppImage's AppDir in $STAGE_DIR (stage/Regard3D.AppDir):
#   usr/bin/Regard3D                Regard3D (RUNPATH $ORIGIN/../lib)
#   usr/lib/                        the libraries it needs that the host does
#                                   not provide (LLVM's libomp)
#   usr/bin/{openmvg,pmvs,poisson,mve}/
#                                   the external tools, where R3DExternalPrograms
#                                   looks (next to the executable), each with
#                                   the libraries it needs (RUNPATH $ORIGIN)
#   usr/share/Regard3D/             sensor_database.csv, DejaVuSans.ttf: the
#                                   wxStandardPaths resources directory
#                                   (<prefix>/share/<app name>), where
#                                   CameraDBLookup and R3DFontHandler look
#   usr/share/Regard3D/licenses/    Regard3D's, the tools', the vcpkg packages'
#   usr/share/applications/, usr/share/icons/
# What may come from the host instead is listed in linux-libs.sh.
set -euo pipefail
source "$(dirname "$0")/linux-libs.sh"

: "${DEPS_DIR:?}" "${VCPKG_INSTALLED:?}" "${VCPKG_TRIPLET:?}" "${LLVM_DIR:?}"
appdir="$PWD/stage/Regard3D.AppDir"
rm -rf "$appdir"
mkdir -p "$appdir/usr/bin" "$appdir/usr/lib" "$appdir/usr/share/Regard3D"
tools="$DEPS_DIR/tools"
# Where the libraries to bundle are looked for: first the packages the
# programs come from ($pkg_path, e.g. the libomp.so.5 OpenMVG ships in lib/),
# then LLVM's (libomp) and the system's (libgomp, libatomic, libquadmath)
pkg_path=()
search_path=("$LLVM_DIR/lib/x86_64-unknown-linux-gnu" "$LLVM_DIR/lib"
	/lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu)

# bundle <elf file> <directory>: copies every library it needs that the
# host does not provide into the directory (unless one is there already),
# and so on for those
bundle() {
	local lib dir found
	for lib in $(needed "$1"); do
		if is_host "$lib" || [[ -e "$2/$lib" ]]; then
			continue
		fi
		found=""
		for dir in "${pkg_path[@]}" "${search_path[@]}"; do
			if [[ -e "$dir/$lib" ]]; then
				found="$dir/$lib"
				break
			fi
		done
		if [[ -z "$found" ]]; then
			echo "::error::$(basename "$1") needs $lib, which was not found to bundle"
			exit 1
		fi
		echo "Bundling $lib (needed by $(basename "$1")) from $(dirname "$found")"
		cp -L "$found" "$2/$lib"
		chmod 755 "$2/$lib"
		bundle "$2/$lib" "$2"
	done
}

# Regard3D
cp build/Regard3D "$appdir/usr/bin/"
strip "$appdir/usr/bin/Regard3D"
patchelf --set-rpath '$ORIGIN/../lib' "$appdir/usr/bin/Regard3D"
# wxWidgets links libOpenGL.so.0, which a default Ubuntu installation lacks
# (libopengl0). glvnd's libGL.so.1 (libgl1, always there) exports the same
# OpenGL functions, and dispatches them to the current context, EGL's too.
if needed "$appdir/usr/bin/Regard3D" | grep -qx libOpenGL.so.0; then
	libgl=/usr/lib/x86_64-linux-gnu/libGL.so.1
	missing_gl="$(comm -23 \
		<(nm -D --undefined-only "$appdir/usr/bin/Regard3D" | awk '{print $NF}' | grep -E '^gl[A-Z]' | sort -u) \
		<(nm -D --defined-only "$libgl" | awk '{print $NF}' | sort -u))"
	if [[ -n "$missing_gl" ]]; then
		echo "::error::$libgl lacks OpenGL functions Regard3D uses: $(tr '\n' ' ' <<< "$missing_gl")"
		exit 1
	fi
	if needed "$appdir/usr/bin/Regard3D" | grep -qx libGL.so.1; then
		patchelf --remove-needed libOpenGL.so.0 "$appdir/usr/bin/Regard3D"
	else
		patchelf --replace-needed libOpenGL.so.0 libGL.so.1 "$appdir/usr/bin/Regard3D"
	fi
	echo "Regard3D: libOpenGL.so.0 replaced by libGL.so.1"
fi
bundle "$appdir/usr/bin/Regard3D" "$appdir/usr/lib"
for lib in "$appdir"/usr/lib/*; do
	patchelf --set-rpath '$ORIGIN' "$lib"
done

cp src/res/sensor_database.csv "$appdir/usr/share/Regard3D/"
cp "$DEPS_DIR/dejavu/ttf/DejaVuSans.ttf" "$appdir/usr/share/Regard3D/"

# The tools. Each directory gets the libraries its programs need (OpenMP
# runtime, libatomic), found through RUNPATH $ORIGIN.
ext="$appdir/usr/bin"
mkdir -p "$ext/openmvg" "$ext/pmvs" "$ext/poisson" "$ext/mve"
cp -a "$DEPS_DIR"/openmvg/bin/. "$ext/openmvg/"
cp -a "$tools"/CMVS-PMVS/bin/. "$ext/pmvs/"
# Only the two programs Regard3D runs (and the libraries shipped with
# them), the others are large examples
cp -a "$tools"/PoissonRecon/bin/{PoissonRecon,SurfaceTrimmer} "$tools"/PoissonRecon/bin/*.so* "$ext/poisson/"
cp -a "$tools"/mve/bin/. "$ext/mve/"
cp -a "$tools"/smvs/bin/smvsrecon "$tools"/mvs-texturing/bin/texrecon "$ext/mve/"
for d in openmvg pmvs poisson mve; do
	case "$d" in
		openmvg) pkgs=("$DEPS_DIR/openmvg") ;;
		pmvs) pkgs=("$tools/CMVS-PMVS") ;;
		poisson) pkgs=("$tools/PoissonRecon") ;;
		mve) pkgs=("$tools/mve" "$tools/smvs" "$tools/mvs-texturing") ;;
	esac
	pkg_path=()
	for p in "${pkgs[@]}"; do
		pkg_path+=("$p/lib" "$p/bin")
	done
	for f in "$ext/$d"/*; do
		if is_elf "$f" && [[ "$(basename "$f")" != *.so* ]]; then
			patchelf --set-rpath '$ORIGIN' "$f"
		fi
	done
	for f in "$ext/$d"/*; do
		if is_elf "$f"; then
			bundle "$f" "$ext/$d"
		fi
	done
done
pkg_path=()

# Desktop integration
install -Dm644 src/res/linux/regard3d.desktop "$appdir/usr/share/applications/regard3d.desktop"
install -Dm644 src/res/linux/regard3d.png "$appdir/usr/share/icons/hicolor/256x256/apps/regard3d.png"

# Licenses
lic="$appdir/usr/share/Regard3D/licenses"
mkdir -p "$lic/vcpkg"
cp src/licenses/Copyright.txt "$lic/Regard3D.txt"
mkdir -p "$lic/OpenMVG"
for f in LICENSE COPYRIGHT.md AUTHORS; do
	cp "$DEPS_DIR/openmvg/$f" "$lic/OpenMVG/"
done
for t in "$tools"/*/; do
	name="$(basename "$t")"
	mkdir -p "$lic/$name"
	# Everything but the programs and libraries: license files, readme,
	# license/ directory
	find "$t" -mindepth 1 -maxdepth 1 ! -name bin ! -name lib ! -name include -exec cp -r {} "$lic/$name/" \;
done
mkdir -p "$lic/DejaVu" "$lic/LLVM"
cp "$DEPS_DIR"/dejavu/LICENSE "$lic/DejaVu/"
# libomp
cp "$LLVM_DIR"/LICENSE.TXT "$lic/LLVM/" 2>/dev/null || true
for d in "$VCPKG_INSTALLED/$VCPKG_TRIPLET"/share/*/; do
	port="$(basename "$d")"
	if [[ -f "$d/copyright" ]]; then
		cp "$d/copyright" "$lic/vcpkg/$port.txt"
	fi
done

# Where the sources of the shipped programs are (CMVS-PMVS is GPL)
{
	echo "Regard3D ${R3D_VERSION:-} includes the following programs and libraries."
	echo "Their licenses are in the directories next to this file."
	echo
	echo "OpenMVG        https://github.com/rhiestan/openMVG/releases/tag/$OPENMVG_TAG"
	echo "CMVS-PMVS      https://github.com/rhiestan/CMVS-PMVS/releases/tag/$CMVS_PMVS_TAG"
	echo "MVE            https://github.com/rhiestan/mve/releases/tag/$MVE_TAG"
	echo "SMVS           https://github.com/rhiestan/smvs/releases/tag/$SMVS_TAG"
	echo "PoissonRecon   https://github.com/rhiestan/PoissonRecon/releases/tag/$POISSONRECON_TAG"
	echo "MVS-Texturing  https://github.com/rhiestan/mvs-texturing/releases/tag/$MVS_TEXTURING_TAG"
	echo "DejaVu fonts   https://dejavu-fonts.github.io/ ($DEJAVU_VERSION)"
	echo "LLVM OpenMP    https://github.com/llvm/llvm-project ($LLVM_VERSION)"
	echo "Libraries built with vcpkg (https://github.com/microsoft/vcpkg): see vcpkg/"
} > "$lic/README.txt"

echo "STAGE_DIR=$appdir" >> "$GITHUB_ENV"
du -sh "$appdir"
find "$appdir" -maxdepth 4 ! -path "*/licenses/*" | sort
