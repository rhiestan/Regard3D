#!/usr/bin/env bash
# Linux only. OpenMVG ships its own static libjpeg 9, libpng, libtiff and zlib
# (libopenMVG_{jpeg,png,tiff,zlib}.a), with the libraries' usual symbol names.
# Regard3D links vcpkg's static libjpeg-turbo, libpng, libtiff and zlib too
# (for OpenCV, wxWidgets, OpenSceneGraph), and in a static link the first
# definition of a symbol wins: OpenMVG's image code, compiled against
# libjpeg 9, then called libjpeg-turbo ("Wrong JPEG library version: library
# is 62, caller expects 90") and crashed.
# This gives every symbol the four libraries define the prefix omvg_, in
# them and in the OpenMVG libraries that use them (libopenMVG_image.a), so
# that each side keeps its own copy. On Windows vcpkg's copies are inside its
# DLLs and do not collide.
# Usage: prefix-openmvg-image-libs.sh <OpenMVG lib directory>
set -euo pipefail

libdir="${1:?}"
bin="${LLVM_DIR:?}/bin"
bundled=(libopenMVG_jpeg.a libopenMVG_png.a libopenMVG_tiff.a libopenMVG_zlib.a)
map="$(mktemp)"
names="$(mktemp)"
undef="$(mktemp)"
trap 'rm -f "$map" "$names" "$undef"' EXIT

for a in "${bundled[@]}"; do
	"$bin/llvm-nm" --defined-only --extern-only --format=just-symbols "$libdir/$a"
done | grep -v ':$' | grep -v '^$' | sort -u | awk '{print $1, "omvg_" $1}' > "$map"
echo "$(wc -l < "$map") symbols to prefix"
if [[ ! -s "$map" ]]; then
	echo "::error::no symbols found in OpenMVG's bundled image libraries"
	exit 1
fi

cut -d' ' -f1 "$map" > "$names"

# The OpenMVG libraries that use them (and the four themselves). nm writes
# to a file first: piped into grep -q, it could be killed by SIGPIPE once
# grep has found a match, which pipefail turns into "no match".
users=()
for a in "$libdir"/lib*.a; do
	"$bin/llvm-nm" --undefined-only --format=just-symbols "$a" > "$undef" 2>/dev/null || true
	if grep -qxFf "$names" "$undef"; then
		users+=("$a")
	fi
done
echo "Prefixing in: $(for a in "${users[@]}"; do basename "$a"; done | tr '\n' ' ')"
for a in "${users[@]}"; do
	"$bin/llvm-objcopy" --redefine-syms="$map" "$a"
done

# None of the original names may be referenced any more
left="$(for a in "$libdir"/lib*.a; do
	"$bin/llvm-nm" --undefined-only --format=just-symbols "$a" 2>/dev/null
done | grep -xFf "$names" | sort -u || true)"
if [[ -n "$left" ]]; then
	echo "::error::still referenced unprefixed: $(tr '\n' ' ' <<< "$left")"
	exit 1
fi
