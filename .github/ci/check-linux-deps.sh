#!/usr/bin/env bash
# check-linux-deps.sh <AppDir>
# Checks every program and library in the AppDir:
# - each library it needs is either provided by the host (linux-libs.sh) or
#   bundled in a directory of its RUNPATH;
# - it needs no newer glibc than the build runner's (Ubuntu 22.04: 2.35),
#   nor a newer C++ runtime (GLIBCXX_3.4.30), so that it runs on every
#   distribution at least that recent;
# - a program does not load two OpenMP runtimes (GCC's libgomp and LLVM's
#   libomp), whose thread pools do not know of each other.
set -euo pipefail
source "$(dirname "$0")/linux-libs.sh"

appdir="$1"
max_glibc=2.35
max_glibcxx=3.4.30
missing=0

# version_gt <a> <b>: a > b
version_gt() {
	[[ "$1" != "$2" && "$(printf '%s\n%s\n' "$1" "$2" | sort -V | tail -1)" == "$1" ]]
}

while IFS= read -r -d '' f; do
	is_elf "$f" || continue
	rel="${f#"$appdir"/}"
	dir="$(dirname "$f")"
	rp="$(runpath "$f")"
	rp="${rp//\$ORIGIN/$dir}"
	for lib in $(needed "$f"); do
		if is_host "$lib"; then
			continue
		fi
		found=0
		IFS=: read -ra dirs <<< "$rp"
		for d in "${dirs[@]}"; do
			if [[ -n "$d" && -e "$d/$lib" ]]; then
				found=1
				break
			fi
		done
		if [[ $found -eq 0 ]]; then
			echo "::error::$rel needs $lib, which is neither bundled (RUNPATH: $(runpath "$f")) nor provided by the host"
			missing=1
		fi
	done

	versions="$(readelf -V "$f" 2>/dev/null | grep -o -E 'GLIBC(XX)?_[0-9.]+' | sort -u || true)"
	glibc="$(grep '^GLIBC_' <<< "$versions" | sed 's/GLIBC_//' | sort -V | tail -1 || true)"
	glibcxx="$(grep '^GLIBCXX_' <<< "$versions" | sed 's/GLIBCXX_//' | sort -V | tail -1 || true)"
	if [[ -n "$glibc" ]] && version_gt "$glibc" "$max_glibc"; then
		echo "::error::$rel needs glibc $glibc, newer than $max_glibc"
		missing=1
	fi
	if [[ -n "$glibcxx" ]] && version_gt "$glibcxx" "$max_glibcxx"; then
		echo "::error::$rel needs GLIBCXX_$glibcxx, newer than $max_glibcxx"
		missing=1
	fi
done < <(find "$appdir" -type f -print0)

# Both OpenMP runtimes in Regard3D (directly or through a bundled library)
omp="$(cd "$appdir/usr" && { needed bin/Regard3D; for l in lib/*; do needed "$l"; done; } | grep -E '^lib(g?omp|iomp5)' | sort -u || true)"
if [[ $(wc -l <<< "$omp") -gt 1 ]]; then
	echo "::error::Regard3D loads more than one OpenMP runtime: $(tr '\n' ' ' <<< "$omp")"
	missing=1
fi

if [[ $missing -ne 0 ]]; then
	exit 1
fi
echo "All libraries are bundled or provided by the host; glibc <= $max_glibc, GLIBCXX <= $max_glibcxx."
