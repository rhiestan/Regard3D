#!/usr/bin/env bash
# Sourced by stage-linux.sh and check-linux-deps.sh: which shared libraries
# the AppImage may take from the host, and how to list an ELF file's needs.
#
# The host provides glibc, the C++ runtime, X11/Wayland, OpenGL and GTK 3
# with its stack (see src/vcpkg-overlay-ports/README.md); every Linux
# desktop has them, and they have to be the host's own to work with its
# drivers, display server and theme. Everything else a program needs is
# bundled next to it.

# is_host <soname>
is_host() {
	case "$1" in
		# glibc
		ld-linux-x86-64.so.2|libc.so.6|libm.so.6|libdl.so.2|libpthread.so.0|librt.so.1|\
		libresolv.so.2|libutil.so.1) return 0 ;;
		# C++ runtime: Regard3D links it statically, the tools need what
		# Ubuntu 22.04 has (GLIBCXX_3.4.30), as does every distribution
		# with glibc 2.35 or later
		libstdc++.so.6|libgcc_s.so.1) return 0 ;;
		libz.so.1) return 0 ;;
		# Display
		libX11.so.6|libX11-xcb.so.1|libXext.so.6|libXi.so.6|libXrandr.so.2|libXinerama.so.1|\
		libXcursor.so.1|libXxf86vm.so.1|libXrender.so.1|libXfixes.so.3|libXtst.so.6|\
		libXcomposite.so.1|libXdamage.so.1|libSM.so.6|libICE.so.6|libxcb*.so.*|\
		libxkbcommon.so.0|libwayland-client.so.0|libwayland-cursor.so.0|libwayland-egl.so.1) return 0 ;;
		# OpenGL (glvnd)
		libGL.so.1|libGLX.so.0|libOpenGL.so.0|libEGL.so.1|libGLU.so.1|libGLdispatch.so.0) return 0 ;;
		# GTK 3 and its stack
		libgtk-3.so.0|libgdk-3.so.0|libgdk_pixbuf-2.0.so.0|libpango-1.0.so.0|\
		libpangocairo-1.0.so.0|libpangoft2-1.0.so.0|libcairo.so.2|libcairo-gobject.so.2|\
		libatk-1.0.so.0|libharfbuzz.so.0|libfontconfig.so.1|libfreetype.so.6|\
		libglib-2.0.so.0|libgobject-2.0.so.0|libgio-2.0.so.0|libgmodule-2.0.so.0|\
		libgthread-2.0.so.0) return 0 ;;
	esac
	return 1
}

# needed <elf file>: the sonames it needs, one per line
needed() {
	readelf -d "$1" | sed -n 's/.*(NEEDED).*\[\(.*\)\]/\1/p'
}

# runpath <elf file>: its RUNPATH (or RPATH), empty if none
runpath() {
	readelf -d "$1" | sed -n 's/.*(\(RUNPATH\|RPATH\)).*\[\(.*\)\]/\2/p' | head -1
}

# is_elf <file>
is_elf() {
	[[ -f "$1" && ! -L "$1" ]] && [[ "$(head -c 4 "$1" | od -An -c | tr -d ' ')" == '177ELF' ]]
}
