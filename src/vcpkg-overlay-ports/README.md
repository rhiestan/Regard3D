# vcpkg overlay ports

Local overrides for vcpkg ports, wired in via `../vcpkg-configuration.json`.
Each entry here should be a *minimal* delta against the port in the pinned vcpkg
baseline (see `builtin-baseline` in `../vcpkg.json`), and should say why it exists.

Re-check these whenever the baseline is bumped — an overlay that upstream has
fixed should be deleted rather than carried forward.

There are currently no overlays. The ones that existed (`gmp`, `liblemon`, `openmvg`) went
away when OpenMVG stopped coming from vcpkg; see the git history of this directory.

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
