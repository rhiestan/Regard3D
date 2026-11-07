#!/usr/bin/env bash
# Sets R3D_VERSION (from src/version.h) and PKG_NAME for the packages:
# Regard3D-<version>-<platform> for a tag v<version>, otherwise
# Regard3D-<version>-<commit>-<platform>.
set -euo pipefail

part() { sed -n "s/^#define REGARD3D_VERSION_$1 \([0-9]*\).*/\1/p" src/version.h; }
version="$(part MAJOR).$(part MINOR).$(part BUILD)"

if [[ "${GITHUB_REF:-}" == refs/tags/v* ]]; then
	tag="${GITHUB_REF_NAME#v}"
	if [[ "$tag" != "$version" ]]; then
		echo "::warning::Tag $GITHUB_REF_NAME does not match version $version in src/version.h"
	fi
	name="Regard3D-${tag}-${R3D_PLATFORM}"
else
	name="Regard3D-${version}-${GITHUB_SHA::7}-${R3D_PLATFORM}"
fi

echo "R3D_VERSION=$version" >> "$GITHUB_ENV"
echo "PKG_NAME=$name" >> "$GITHUB_ENV"
echo "Package: $name"
