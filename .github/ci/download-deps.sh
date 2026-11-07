#!/usr/bin/env bash
# Downloads, into $DEPS_DIR, the release packages of OpenMVG and the external
# tools for $R3D_PLATFORM (windows-x64, linux-x64, macos-arm64), and the DejaVu
# font. Each package ends up in its own directory, without its top-level
# directory:
#   openmvg/                      OpenMVG installation (bin, include, lib)
#   tools/<repo>/                 bin/ plus license files
#   dejavu/                       DejaVu fonts (ttf/, LICENSE)
# The tags come from the environment (see .github/workflows/build.yml); an
# empty tag skips that tool.
set -euo pipefail

: "${DEPS_DIR:?}" "${R3D_PLATFORM:?}"
mkdir -p "$DEPS_DIR/tools"
cd "$DEPS_DIR"

if [[ "$R3D_PLATFORM" == windows-* ]]; then ext=zip; else ext=tar.gz; fi

# extract <archive> <target directory>
extract() {
	local tmp="$DEPS_DIR/.extract"
	rm -rf "$tmp" "$2"
	mkdir -p "$tmp"
	case "$1" in
		*.zip)
			if command -v 7z > /dev/null; then 7z x -y -o"$tmp" "$1" > /dev/null
			else unzip -q "$1" -d "$tmp"; fi ;;
		*) tar -C "$tmp" -xzf "$1" ;;
	esac
	local top=("$tmp"/*)
	if [[ ${#top[@]} -ne 1 || ! -d "${top[0]}" ]]; then
		echo "$1: expected a single top-level directory" >&2
		exit 1
	fi
	mv "${top[0]}" "$2"
	rm -rf "$tmp" "$1"
}

# release <repo> <tag> <asset name prefix> <target directory>
release() {
	local repo="$1" tag="$2" prefix="$3" target="$4"
	if [[ -z "$tag" ]]; then
		echo "$repo: no release tag, skipped"
		return
	fi
	local asset="$prefix-$tag-$R3D_PLATFORM.$ext"
	echo "$repo $tag: $asset"
	gh release download "$tag" --repo "rhiestan/$repo" --pattern "$asset" --clobber
	extract "$asset" "$target"
}

release openMVG "$OPENMVG_TAG" openMVG-fast openmvg
release CMVS-PMVS "$CMVS_PMVS_TAG" cmvs-pmvs tools/CMVS-PMVS
release mve "$MVE_TAG" mve tools/mve
release smvs "$SMVS_TAG" smvs tools/smvs
release PoissonRecon "$POISSONRECON_TAG" poissonrecon tools/PoissonRecon
release mvs-texturing "$MVS_TEXTURING_TAG" texrecon tools/mvs-texturing

font="dejavu-fonts-ttf-$DEJAVU_VERSION.zip"
curl -sSfL -o "$font" \
	"https://github.com/dejavu-fonts/dejavu-fonts/releases/download/version_${DEJAVU_VERSION//./_}/$font"
extract "$font" dejavu

ls -l "$DEPS_DIR" "$DEPS_DIR/tools"
