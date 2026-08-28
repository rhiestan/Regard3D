# vcpkg overlay ports

Local overrides for vcpkg ports, wired in via `../vcpkg-configuration.json`.
Each entry here should be a *minimal* delta against the port in the pinned vcpkg
baseline (see `builtin-baseline` in `../vcpkg.json`), and should say why it exists.

Re-check these whenever the baseline is bumped — an overlay that upstream has
fixed should be deleted rather than carried forward.

## liblemon

**Why:** the download hash recorded in the upstream port no longer matches what the
server actually serves.

`liblemon` is pulled in by `openmvg` (graph algorithms / linear programming). Its
portfile fetches `http://lemon.cs.elte.hu/hg/lemon/archive/ed2c21cbd6ef.zip` and
checks it against a recorded SHA512. The LEMON project's Mercurial server has since
regenerated its archives, so the bytes differ and every build fails with:

```
error: download from http://lemon.cs.elte.hu/hg/lemon/archive/ed2c21cbd6ef.zip had an unexpected hash
error: building liblemon:x64-windows failed with: BUILD_FAILED
```

This is not a transient network problem and not local: the download is deterministic
(repeated fetches produce the same bytes), and vcpkg `master` still carries the same
stale hash, so it affects everyone building `openmvg` from vcpkg.

**Content was verified before overriding**, rather than blindly trusting the new bytes:

- served over HTTPS by the official `lemon.cs.elte.hu` host, HTTP 200, 781182 bytes;
- extracts to a complete, normal LEMON source tree with the expected LICENSE;
- `.hg_archival.txt` reports `node: ed2c21cbd6ef8c3633e9edca47a61a26012dda36` —
  exactly the revision the port pins — on branch `1.3`, `latesttag: r1.3.1`.

So the archive is the intended source revision; only the zip container changed.
The overlay is the upstream port with `SHA512` updated to
`849d9974...21bfd9` and a comment recording the above.

**Remove this overlay** once upstream vcpkg corrects the hash or moves liblemon to a
stable mirror.

## gmp

**Why:** the MSYS2 package the port pins has been purged from every MSYS2 mirror.

`gmp` (pulled in via `mpfr` → `suitesparse`) builds with autotools on Windows and
fetches one MSYS2 package directly:

```
Downloading autoconf2.71-2.71-3-any.pkg.tar.zst ...
error: curl operation failed with response code 404.   (all six mirrors)
error: building gmp:x64-windows failed with: BUILD_FAILED
```

MSYS2 is a rolling distribution and purges superseded builds: it now serves
`autoconf2.71-2.71-**4**`, so the pinned `-3` is gone everywhere. (gmp deliberately
cannot use the newer autoconf 2.72 that vcpkg otherwise ships — see the port's own
comment, "dumpbin detection fails with autoconf 2.72".)

**This overlay is not a local invention — it is the upstream fix.** vcpkg `master`
already corrected it in `gmp` port-version 5, bumping the URL and SHA512 to
`autoconf2.71-2.71-4`. The pinned baseline (2026.07.29) still has port-version 4.
The overlay is `ports/gmp` copied verbatim from vcpkg `master`; no local edits.

**Remove this overlay** when `builtin-baseline` is bumped to a vcpkg release that
includes gmp port-version 5 or newer.

## openmvg

**Why:** the installed `OpenMVGConfig.cmake` cannot find OpenMVG's own headers, so
`find_package(OpenMVG)` fails outright:

```
CMake Error at .../share/openmvg/cmake/OpenMVGConfig.cmake:42 (message):
  Failed to find OPENMVG - OpenMVG install root: .../share/openmvg/cmake.
  Cannot find openMVG include files.
```

**Root cause: the port passes a config path that isn't where the config actually is.**
OpenMVG installs its CMake package into `lib/openMVG/**cmake**`, but the port calls
`vcpkg_cmake_config_fixup(CONFIG_PATH "lib/openMVG")`. So `vcpkg_cmake_config_fixup`
relocated the directory without knowing about that extra level, and every piece of
path arithmetic that depends on the config's depth ended up short by one:

- `OpenMVGConfig.cmake` hand-rolls its install root as
  `get_filename_component(... "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)`, which
  resolved to `<prefix>/share` — hence the error above.
- `OpenMVGTargets.cmake`'s generated `_IMPORT_PREFIX` walk resolved to `<prefix>/share`
  too, so the imported targets pointed at
  `<prefix>/share/debug/lib/openMVG_stlplus.lib` instead of
  `<prefix>/debug/lib/openMVG_stlplus.lib`:

  ```
  CMake Error at .../OpenMVGTargets.cmake:253 (message):
    The imported target "OpenMVG::openMVG_stlplus" references the file
       ".../x64-windows/share/debug/lib/openMVG_stlplus.lib"
    but this file does not exist.
  ```

Present in the pinned baseline *and* in vcpkg `master` (both `openmvg` port-version 4),
so it is unfixed upstream.

**The fix is to tell the fixup the truth**, `CONFIG_PATH "lib/openMVG/cmake"`, rather
than to regex-patch the two files afterwards. vcpkg then does the depth adjustment
itself, and the package lands at `<prefix>/share/openmvg` — which is exactly the layout
OpenMVG's own `../..` assumes, so both problems disappear at once and nothing
hand-written needs rewriting.

Because `config_fixup` only relocates the `cmake/` directory, the overlay additionally
moves OpenMVG's remaining payload (`sensor_width_camera_database.txt`, `webgl/`) from
`lib/openMVG` into `share/openmvg`, reproducing the layout the old `CONFIG_PATH`
produced, and drops the now-empty `lib/openMVG` and `debug/lib/openMVG`.

**Remove this overlay** once upstream vcpkg corrects the `CONFIG_PATH`.

## Checking for more of this

Both overlays are the same failure mode: the pinned baseline references an artifact
that its upstream has since deleted. Before a long dependency build, it is worth
scanning the ports still to be built for rot-prone downloads:

```sh
# ports that pin MSYS2 packages directly, or download from somewhere other than GitHub
cd <vcpkg>
for p in <ports>; do
  git show <baseline-tag>:ports/$p/portfile.cmake 2>/dev/null \
    | grep -nE 'DIRECT_PACKAGES|https?://' | grep -v github.com
done
```

GitHub release archives and GNU FTP are stable and versioned; MSYS2 mirrors and
project-run Mercurial/SVN servers are the ones that rot.
