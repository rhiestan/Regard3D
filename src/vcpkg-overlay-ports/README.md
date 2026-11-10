# vcpkg overlay ports

Local overrides for vcpkg ports, wired in via `../vcpkg-configuration.json`.
Each entry here should be a *minimal* delta against the port in the pinned vcpkg
baseline (see `builtin-baseline` in `../vcpkg.json`), and should say why it exists.

Re-check these whenever the baseline is bumped — an overlay that upstream has
fixed should be deleted rather than carried forward.

The ones that existed earlier (`gmp`, `liblemon`, `openmvg`) went away when OpenMVG stopped
coming from vcpkg; see the git history of this directory.

## `gtk3`, `cairo` — the system's libraries (Linux)

On Linux, wxWidgets (and OpenCV's HighGUI) need GTK 3, and wxWidgets cairo. vcpkg would build
GTK and its whole stack (glib, pango, cairo, harfbuzz, at-spi, ...) from source and link it
statically into Regard3D, which is a long build and leaves Regard3D with a GTK that cannot load
the host's theme, input-method and print modules. These two ports install nothing: the ports
that depend on them find the system's libraries through pkg-config (Ubuntu: `libgtk-3-dev`,
which brings `libcairo2-dev`), and the AppImage uses the host's GTK, which every Linux desktop
has. cairo has to come from the system too, as the system's GTK uses the system's cairo, and two
cairos in one process do not mix.

Their portfiles only check that pkg-config finds the library. They are not used on Windows,
where nothing depends on them. Keep their feature lists in step with what other ports request
(`gtk3[introspection,wayland]`, `cairo[fontconfig,freetype,gobject,lzo,x11]`): vcpkg rejects a
request for a feature a port does not declare.

## Checking for more of this

The usual failure mode is that the pinned baseline references an artifact that its
upstream has since deleted. Before a long dependency build, it is worth
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
