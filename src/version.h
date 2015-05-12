#ifndef VERSION_H
#define VERSION_H

#define R3D_STRINGIFY2(x) #x
#define R3D_STRINGIFY(x) R3D_STRINGIFY2(x)

#define REGARD3D_VERSION_MAJOR 0
#define REGARD3D_VERSION_MINOR 7
#define REGARD3D_VERSION_BUILD 1

#define REGARD3D_VERSION_STRING R3D_STRINGIFY(REGARD3D_VERSION_MAJOR) \
	"." R3D_STRINGIFY(REGARD3D_VERSION_MINOR) \
	"." R3D_STRINGIFY(REGARD3D_VERSION_BUILD)

#define REGARD3D_NAME "Regard 3D"
#define REGARD3D_COPYRIGHT_NAME "Roman Hiestand"
#define REGARD3D_COPYRIGHT_YEAR "2015"
#define REGARD3D_VENDOR_NAME "hiesti.ch"

// The check for __INTEL_COMPILER needs to be before _MSC_VER and __GNUG__, as
// the Intel compiler defines _MSC_VER and __GNUG__ as well
#if defined(__INTEL_COMPILER)
#	define REGARD3D_COMPILER "Intel C++"
#	define REGARD3D_COMPILER_VERSION R3D_STRINGIFY(__INTEL_COMPILER)
#elif defined(_MSC_VER)
#	define REGARD3D_COMPILER "Visual C++"
#	define REGARD3D_COMPILER_VERSION R3D_STRINGIFY(_MSC_FULL_VER)
#elif defined(__clang__)
#	if defined( __apple_build_version__)
#		define REGARD3D_COMPILER_VERSION "Apple Clang"
#	else
#		define REGARD3D_COMPILER_VERSION "Clang"
#	endif
#	define REGARD3D_COMPILER_VERSION R3D_STRINGIFY(__clang_major__) "." R3D_STRINGIFY(__clang_minor__) "." R3D_STRINGIFY(__clang_patchlevel__)
#elif defined(__GNUG__)
#	define REGARD3D_COMPILER "GCC"
#	if defined(__GNUC_PATCHLEVEL__)
#		define REGARD3D_COMPILER_VERSION R3D_STRINGIFY(__GNUC__) "." R3D_STRINGIFY(__GNUC_MINOR__) "." R3D_STRINGIFY(__GNUC_PATCHLEVEL__)
#	else
#		define REGARD3D_COMPILER_VERSION R3D_STRINGIFY(__GNUC__) "." R3D_STRINGIFY(__GNUC_MINOR__)
#	endif
#else
#	define REGARD3D_COMPILER "Unknown compiler"
#	define REGARD3D_COMPILER_VERSION "Unknown version"
#endif

// Architecture
#if defined(_M_X64) || defined(_M_AMD64) || defined(__amd64__) || defined(__amd64) || defined(__x86_64__) || defined(__x86_64)
#define REGARD3D_ARCHITECTURE_X64
#define REGARD3D_ARCHITECTURE_STRING "x64"
#elif defined(_M_ARM) || defined(__arm__) || defined(_ARM)
#define REGARD3D_ARCHITECTURE_ARM
#define REGARD3D_ARCHITECTURE_STRING "ARM"
#elif defined(_M_IX86) || defined(__i386) || defined(__i486__) || defined(__i586__) || defined(__i686__) || defined(_X86_)
#define REGARD3D_ARCHITECTURE_X86
#define REGARD3D_ARCHITECTURE_STRING "x86"
#elif defined(_M_IA64) || defined(__ia64__) || defined(_IA64) || defined(__IA64__) || defined(__ia64)
#define REGARD3D_ARCHITECTURE_ITANIUM
#define REGARD3D_ARCHITECTURE_STRING "Itanium"
#elif defined(_M_PPC) || defined(__powerpc) || defined(__powerpc__) || defined(__powerpc64__) || defined(__POWERPC__) || defined(__ppc__) || defined(__ppc64__)
#define REGARD3D_ARCHITECTURE_PPC
#define REGARD3D_ARCHITECTURE_STRING "PPC"
#elif defined(__sparc__) || defined(__sparc)
#define REGARD3D_ARCHITECTURE_SPARC
#define REGARD3D_ARCHITECTURE_STRING "SPARC"
#endif

#endif
