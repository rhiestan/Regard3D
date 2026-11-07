#!/usr/bin/env bash
# check-windows-deps.sh <stage dir>
# Checks that every DLL imported by Regard3D.exe, its DLLs and the
# OpenSceneGraph plugins is either in the stage directory, part of Windows,
# or part of the MSVC runtime (installed by vc_redist, or added to the zip).
# The runner has many more DLLs in System32 than a user's machine (Visual
# Studio, SDKs), so the Windows DLLs are an explicit list of name patterns
# rather than "whatever System32 has".
set -euo pipefail

stage="$1"
readobj="${LLVM_DIR:?}/bin/llvm-readobj.exe"

shopt -s nocasematch
is_system() {
	case "$1" in
		api-ms-win-*|ext-ms-*) return 0 ;;
		# MSVC runtime
		vcruntime140*.dll|msvcp140*.dll|vcomp140.dll|concrt140.dll) return 0 ;;
		# Windows
		kernel32.dll|kernelbase.dll|user32.dll|gdi32.dll|gdiplus.dll|advapi32.dll|shell32.dll|\
		shlwapi.dll|ole32.dll|oleaut32.dll|comctl32.dll|comdlg32.dll|winspool.drv|ws2_32.dll|\
		wsock32.dll|rpcrt4.dll|opengl32.dll|glu32.dll|imm32.dll|uxtheme.dll|version.dll|\
		winmm.dll|bcrypt.dll|crypt32.dll|secur32.dll|ntdll.dll|dbghelp.dll|psapi.dll|\
		setupapi.dll|cfgmgr32.dll|dwmapi.dll|msimg32.dll|oleacc.dll|urlmon.dll|wininet.dll|\
		winhttp.dll|iphlpapi.dll|userenv.dll|dwrite.dll|d2d1.dll|d3d11.dll|dxgi.dll|\
		windowscodecs.dll|uiautomationcore.dll|shcore.dll|mpr.dll|netapi32.dll|normaliz.dll|\
		mf*.dll|ksuser.dll|propsys.dll|wldap32.dll|dnsapi.dll|ncrypt.dll|powrprof.dll|\
		hid.dll|wtsapi32.dll|avrt.dll|mscms.dll|usp10.dll|msvcrt.dll|ucrtbase.dll) return 0 ;;
	esac
	return 1
}

declare -A present
while IFS= read -r -d '' f; do
	present["$(basename "$f" | tr '[:upper:]' '[:lower:]')"]=1
done < <(find "$stage" -maxdepth 1 -iname '*.dll' -print0)

missing=0
while IFS= read -r -d '' bin; do
	while IFS= read -r dll; do
		lower="$(tr '[:upper:]' '[:lower:]' <<< "$dll")"
		if [[ -n "${present[$lower]:-}" ]] || is_system "$dll"; then
			continue
		fi
		echo "::error::${bin#"$stage"/} imports $dll, which is neither packaged nor part of Windows"
		missing=1
	done < <("$readobj" --coff-imports "$bin" | sed -n 's/^ *Name: *\(.*\.[A-Za-z]*\)$/\1/p' | sort -u)
done < <(find "$stage" -maxdepth 2 \( -iname '*.exe' -o -iname '*.dll' \) \
	! -path "$stage/openmvg/*" ! -path "$stage/pmvs/*" ! -path "$stage/poisson/*" ! -path "$stage/mve/*" -print0)

# The tools only need their OpenMP runtime next to them
for d in openmvg pmvs poisson mve; do
	while IFS= read -r -d '' bin; do
		while IFS= read -r dll; do
			if [[ -f "$stage/$d/$dll" ]] || is_system "$dll"; then
				continue
			fi
			echo "::error::$d/$(basename "$bin") imports $dll, which is neither packaged nor part of Windows"
			missing=1
		done < <("$readobj" --coff-imports "$bin" | sed -n 's/^ *Name: *\(.*\.[A-Za-z]*\)$/\1/p' | sort -u)
	done < <(find "$stage/$d" -iname '*.exe' -print0)
done

if [[ $missing -ne 0 ]]; then
	exit 1
fi
echo "All DLL dependencies are packaged or part of Windows."
