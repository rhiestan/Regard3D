#!/usr/bin/env bash
# Puts everything the Windows packages contain into $STAGE_DIR (stage/Regard3D):
#   Regard3D.exe, its DLLs and the OpenSceneGraph image plugins (build/)
#   libomp.dll                      LLVM's OpenMP runtime, as OpenMVG's
#   sensor_database.csv, DejaVuSans.ttf
#                                   found next to the executable (wxStandardPaths
#                                   resources directory on Windows)
#   openmvg/ pmvs/ poisson/ mve/    the external tools, in the directories
#                                   R3DExternalPrograms looks in
#   licenses/                       Regard3D's, the tools', the vcpkg packages'
# The MSVC runtime is not included: the installer runs vc_redist, the zip
# gets the DLLs added (package-windows.sh).
set -euo pipefail

: "${DEPS_DIR:?}" "${VCPKG_INSTALLED:?}" "${VCPKG_TRIPLET:?}"
stage="$PWD/stage/Regard3D"
rm -rf "$stage"
mkdir -p "$stage"
tools="$DEPS_DIR/tools"

# Regard3D
cp build/Regard3D.exe build/*.dll "$stage/"
cp -r build/osgPlugins-* "$stage/"
cp "$DEPS_DIR/openmvg/bin/libomp.dll" "$stage/"

# vcpkg's applocal step copies only what Regard3D.exe imports, not what the
# OpenSceneGraph plugins it loads at run time import (e.g. gif.dll for
# osgdb_gif). Add those from vcpkg, repeated until nothing is missing.
vcpkg_bin="$VCPKG_INSTALLED/$VCPKG_TRIPLET/bin"
readobj="${LLVM_DIR:?}/bin/llvm-readobj.exe"
while :; do
	added=0
	for bin in "$stage"/*.dll "$stage"/osgPlugins-*/*.dll; do
		for dll in $("$readobj" --coff-imports "$bin" | sed -n 's/^ *Name: *\(.*\.[A-Za-z]*\)$/\1/p' | sort -u); do
			if [[ ! -f "$stage/$dll" && -f "$vcpkg_bin/$dll" ]]; then
				echo "Adding $dll (imported by $(basename "$bin"))"
				cp "$vcpkg_bin/$dll" "$stage/"
				added=1
			fi
		done
	done
	[[ $added -eq 0 ]] && break
done

cp src/res/sensor_database.csv "$stage/"
cp "$DEPS_DIR/dejavu/ttf/DejaVuSans.ttf" "$stage/"

# The tools. Each directory gets its own OpenMP runtime: a program loads
# the DLLs next to itself first.
mkdir -p "$stage/openmvg" "$stage/pmvs" "$stage/poisson" "$stage/mve"
cp "$DEPS_DIR"/openmvg/bin/*.exe "$DEPS_DIR/openmvg/bin/libomp.dll" "$stage/openmvg/"
cp "$tools"/CMVS-PMVS/bin/* "$stage/pmvs/"
# Only the two programs Regard3D runs, the others are large examples
cp "$tools"/PoissonRecon/bin/{PoissonRecon.exe,SurfaceTrimmer.exe,libomp.dll} "$stage/poisson/"
cp "$tools"/mve/bin/* "$stage/mve/"
cp "$tools"/smvs/bin/smvsrecon.exe "$stage/mve/"
cp "$tools"/mvs-texturing/bin/texrecon.exe "$stage/mve/"

# Licenses
lic="$stage/licenses"
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
mkdir -p "$lic/DejaVu"
cp "$DEPS_DIR"/dejavu/LICENSE "$lic/DejaVu/"
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
	echo "Libraries built with vcpkg (https://github.com/microsoft/vcpkg): see vcpkg/"
} > "$lic/README.txt"

echo "STAGE_DIR=$stage" >> "$GITHUB_ENV"
du -sh "$stage"
find "$stage" -maxdepth 2 | sort
