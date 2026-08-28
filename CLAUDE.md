# CLAUDE.md

Guidance for Claude Code when working in this repository.

## What this is

Regard3D — an open-source, cross-platform **structure-from-motion** GUI application (wxWidgets desktop app).
It takes a set of photographs and produces a 3D model, driving [OpenMVG](https://github.com/openMVG/openMVG)
for the SfM part and a set of bundled command-line executables (CMVS/PMVS, MVE, SMVS, PoissonRecon) for
densification and surface reconstruction. Homepage: <http://www.regard3d.org>.

License: MIT (`src/licenses/Copyright.txt`). Author: Roman Hiestand.

## Repository vs. workspace

This git repo (`Regard3D_git/`) contains **only the Regard3D sources**. It lives inside a larger,
non-versioned build workspace:

```
C:\Projects\Regard3D\
├── Regard3D_git\            <-- this repo
│   ├── src\                 <-- everything; CMake root is src/, not the repo root
│   └── README.md, version.txt
├── openMVG\                 reference checkout of OpenMVG (at tag v2.0; v2.1 fetched).
│                           NOT what gets linked — that comes from vcpkg. Useful for
│                           diffing API changes between releases.
├── openmvg_build\           stale out-of-tree OpenMVG build (pre-vcpkg)
├── eigen-3.4.0\             stale Eigen source + local install (pre-vcpkg)
├── install\                 stale local install prefix for hand-built deps
├── build_2026\              *** the current build tree *** (vcpkg manifest mode, VS 2026)
├── build_2022\              superseded build tree (vcpkg classic mode, VS 2019) — fallback
├── build-scripts\           legacy per-library MSVC build scripts (pre-vcpkg)
└── *.bat                    configure/build helper scripts
```

Dependencies today come from **vcpkg** at `C:\Projects\vcpkg\vcpkg` (triplet `x64-windows`).
The `build-scripts/` folder is the *older* way of producing dependencies by hand and is largely
historical — treat its `libs_versions.bat` as stale (it still lists Boost 1.62, OpenCV 3.1, Eigen 3.2).

## Building (Windows, current setup)

Dependencies are declared in **`src/vcpkg.json`** (vcpkg *manifest* mode) and pinned by its
`builtin-baseline` to vcpkg release **2026.07.29** (`9e593bb1`). `src/vcpkg-configuration.json`
adds `src/vcpkg-overlay-ports/` — see that directory's README before touching it.

Configure from the workspace root:

```bat
create_VStudio_regard3d_2026.bat
```

which does, in effect:

```bat
%R3D_CMAKE% -Wno-dev -S Regard3D_git/src -B build_2026 ^
      -G "Visual Studio 18 2026" -A x64 ^
      -DCMAKE_CONFIGURATION_TYPES=Release;RelWithDebInfo;Debug ^
      -DCMAKE_TOOLCHAIN_FILE=%VCPKG_PATH%\scripts\buildsystems\vcpkg.cmake ^
      -DVCPKG_TARGET_TRIPLET=x64-windows ^
      -DVCPKG_INSTALL_OPTIONS=--clean-after-build
```

Then build `build_2026\Regard3D.sln` (or `<cmake4> --build build_2026 --config Debug`).
Debug and Release both build clean.

The first configure builds ~184 packages into `build_2026/vcpkg_installed` and takes hours.
It does **not** touch the old classic `C:\Projects\vcpkg\vcpkg\installed` tree, which
`build_2022/` still uses — that remains as a fallback.

### The toolset must match vcpkg — this is the sharp edge

vcpkg builds ports with **the newest Visual Studio it finds**, which here is **VS 2026**
(toolset `v145`, MSVC 14.51). The project must use the same toolset. Building the project with
VS 2022 (MSVC 14.44) against those libraries fails at link with unresolved MSVC STL symbols:

```
openMVG_sfm.lib(sfm_report.cpp.obj) : error LNK2019: unresolved external symbol __std_min_element_d_
```

Those `__std_{min,max}_element_*` vectorized helpers exist in the 14.51 STL but not in 14.44 —
i.e. the headers that compiled the libraries are newer than the import lib linking them.

Consequences to know about:
- The `Visual Studio 18 2026` generator requires **CMake 4.x**. The system CMake (3.31.8) only
  knows up to VS 2022, so the build script uses the CMake that **vcpkg downloaded** for its own
  port builds (`$VCPKG/downloads/tools/cmake-4.4.0-windows/...`). That is a vcpkg-managed tool and
  could be cleaned up; installing a standalone CMake 4.x and pointing `R3D_CMAKE` at it is more
  robust. The script fails with an explanatory message if it is missing.
- CMake 4 removed compatibility with `cmake_minimum_required` below 3.5, so all eight vendored
  `src/thirdparty/*/CMakeLists.txt` were bumped from `2.6`/`2.8` to `3.10`.
- If you ever want the project back on VS 2022, pin vcpkg too — a custom triplet setting
  `VCPKG_PLATFORM_TOOLSET v143` and `VCPKG_VISUAL_STUDIO_PATH`. That changes every package's ABI
  hash and rebuilds all 184 from scratch.

Notes / gotchas:
- **The CMake source dir is `src/`, not the repo root.** `vcpkg.json` lives there for that reason.
- Updating dependencies is now a one-line `builtin-baseline` bump — plus re-checking whether the
  overlay ports are still needed.
- The superseded `create_VStudio_regard3d_vcpkg.bat` / `build_2022/` used vcpkg **classic** mode and
  the VS **2019** generator (despite the 2022 name), against a Dec-2022 vcpkg checkout.
- `build_2026/vcpkg_installed` lives *inside* the build tree, so never `rm -rf build_2026` to force
  a reconfigure — that throws away hours of dependency builds. Delete only `CMakeCache.txt` and
  `CMakeFiles/`.
- Some deps are still located by hand-rolled `Find*.cmake` in `src/cmake/`
  (`FindFLANN`, `FindTBB`, `FindVLFEAT`, `FindBLAS`, `FindLAPACK`).
  Where vcpkg ships a proper config package, prefer `find_package(<x> CONFIG REQUIRED)` and drop
  the local module — done for AssImp, SuiteSparse and glog.
- `FindVLFEAT.cmake` looks for a library named `vlfeat`/`libvlfeat`, but vcpkg installs `vl.lib`,
  so `VLFEAT_FOUND` is always false. It doesn't matter today because `Regard3DFeatures.cpp` also
  has an unconditional `#undef R3D_HAVE_VLFEAT`, but fix both together if VLFeat is ever wanted.
- OpenMVG's non-installed dependency headers are pulled in with an acknowledged hack:
  `INCLUDE_DIRECTORIES("${wxWidgets_ROOT_DIR}/include/openMVG_dependencies")` — i.e. it abuses the
  wxWidgets root as a stand-in for the vcpkg prefix. Worth cleaning up.

### Current dependency versions (vcpkg x64-windows)

| Library | Version | Notes |
|---|---|---|
| OpenMVG | 2.1 | linked via `OpenMVG::openMVG_*` imported targets |
| OpenCV | 4.12.0 (`opencv4`) | core, flann, imgproc, features2d, calib3d, highgui, imgcodecs, videoio, ml |
| wxWidgets | 3.3.1 | net, gl, aui, adv, core, base |
| Boost | 1.91.0 | serialization, thread, filesystem, chrono, locale, date_time, timer, system, plus header-only **accumulators** and **geometry** |
| Eigen | 5.0.1 | |
| Ceres | 2.2.0 | bundle adjustment, via OpenMVG; `[lapack,suitesparse]` |
| glog | 0.7.1 | `find_package(glog CONFIG)` → `glog::glog` |
| SuiteSparse | 7.12.3 | `find_package(SuiteSparse_config CONFIG)` → `SuiteSparse::SuiteSparseConfig` |
| OpenBLAS | 0.3.33 | BLAS/LAPACK |
| OpenSceneGraph | 3.6.5 | 3D viewport |
| AssImp | 6.0.4 | model import/export; `find_package(assimp CONFIG)` |
| FLANN | 2022-10-28 | matching |
| VLFeat | 2020-07-10 | present but **not actually enabled** — see the note below |
| NLopt | not installed | optional; only if `NLOPT_ROOT_DIR` is set (`R3D_HAVE_NLOPT`) |
| TBB | not installed | optional; `R3D_USE_TBB` / `R3D_USE_TBB_THREADING`, both off by default |

Boost is declared as the modular `boost-*` ports. Note the CMakeLists
`FIND_PACKAGE(Boost ... COMPONENTS ...)` list covers only *linked* Boost libraries — it does not
mention header-only Boost that the sources `#include`. When adding a Boost dependency, check the
actual includes:

```sh
grep -rhoE '#include [<"]boost/[a-zA-Z0-9_/.]+' --include=*.cpp --include=*.h src --exclude-dir=thirdparty
```

Vendored third-party sources live in `src/thirdparty/`: `akaze`, `fast-akaze`, `liop`, `tinyply`,
`sqlite`, `cpuid`, header-only `efanna`, `mrpt`, `hnswlib`, `kgraph`, plus a handful of OpenMVG
headers that upstream does not install (`src/thirdparty/openMVG/**`, e.g. `InterfaceMVS.h`,
`SfMPlyHelper.hpp`, `document.h`).

### Compile-time configuration

`src/config.h.in` → generated `config.h` carries the feature flags: `R3D_WIN32/MACOSX/LINUX`,
`R3D_HAVE_OPENMP`, `R3D_HAVE_VLFEAT`, `R3D_HAVE_NLOPT`, `R3D_HAVE_TBB`, `R3D_USE_TBB_THREADING`,
`R3D_HAVE_OPENMVG_VERSION` / `OPENMVG_VERSION`. Every translation unit includes
`src/CommonIncludes.h` first, which pulls in `config.h`, the wxWidgets prologue, and undefines
X11/Windows macro landmines (`Status`, `True`, `False`, `Success`, `ERROR`).

Version numbers live in `src/version.h` and are parsed back out of it by `src/CMakeLists.txt`
(`REGARD3D_VERSION_MAJOR/MINOR/BUILD`, `REGARD3D_COPYRIGHT_YEAR`). Keep the one-macro-per-line
format intact or the CMake regex breaks. User-facing changelog: `README.md` and `version.txt`.

## Architecture

### Project model — `src/R3DProject.{h,cpp}`

A singleton (`R3DProject::getInstance()`) holding a strict five-level tree that mirrors the
reconstruction pipeline; each level is a nested `std::vector` on its parent:

```
PictureSet  →  ComputeMatches  →  Triangulation  →  Densification  →  Surface
```

Every node derives from `R3DProject::Object` (`id_`, `parentId_`, `runningId_`, `name_`) and carries
its own parameters, an `R3DObjectState` (`OSInvalid` / `OSRunning` / `OSFailed` / `OSFinished`) and
result strings. `id_` is unique project-wide; `runningId_` is unique among siblings and is what names
the on-disk directories. The wxTreeCtrl in the main window maps to these nodes through
`R3DProject::R3DTreeItem` (type + id).

Persistence is **Boost.Serialization XML** (`serialize()` per class). Changing a member means bumping
that class's serialization version and handling the old one — project files in the wild must keep
loading.

`R3DProjectPaths` is the bundle of absolute/relative paths passed to all worker code; obtain it via
`getProjectPathsCM/Tri/Dns/Srf()`. Anything needing a filename should take `R3DProjectPaths` rather
than recompute paths.

### GUI

- `src/Regard3DMainFrameBase.{h,cpp}` is **generated by wxFormBuilder** from
  `src/res/Regard3dMainFrameBase.fbp`. Do not hand-edit it — edit the `.fbp` and regenerate.
- `src/Regard3DMainFrame.{h,cpp}` derives from it and holds all the real logic and event handlers.
- Per-step dialogs are in `src/gui/` (`Regard3DComputeMatchesDialog`, `Regard3DTriangulationDialog`,
  `Regard3DDensificationDialog`, `Regard3DSurfaceDialog`, `Regard3DPictureSetDialog`, …).
- The 3D viewport is OpenSceneGraph embedded in a `wxGLCanvas`: `src/gl/` (`OSGGLCanvas`,
  `GraphicsWindowWX`, `SharedGLContext`) plus `Regard3DModelViewHelper` for scene assembly and
  `R3DModelOperations` for mesh/point-cloud manipulation.
- Settings live in `src/utils/Regard3DSettings` (wxConfig-backed).

### Work execution — two mechanisms

(Mechanism 1 is being retired in favour of mechanism 2 — see *Direction of travel* below.)

1. **In-process `wxThread`s** (`src/threads/`) for anything using OpenMVG as a library:
   `R3DFeaturesThread` (keypoints/descriptors), `R3DComputeMatchesThread`, `R3DTriangulationThread`,
   `R3DSmallTasksThread` (exports, model loading, colorization), plus `ImageInfoThread` and
   `PreviewGeneratorThread` for the UI.
2. **External processes** via `wxProcess` for the bundled executables:
   `R3DDensificationProcess` and `R3DSurfaceGenProcess`. Each builds an ordered `wxArrayString cmds_`
   and runs them one at a time, advancing in `OnTerminate()` — a small command-queue state machine.
   Executables are discovered at startup by `src/utils/R3DExternalPrograms.cpp`, which expects
   `pmvs/`, `poisson/`, `mve/` (and optionally `cmpmvs/`) subdirectories next to the Regard3D
   executable (overridable in Properties): `pmvs2`, `cmvs`, `genOption`, `PoissonRecon`,
   `SurfaceTrimmer`, `makescene`, `dmrecon`, `scene2pset`, `fssrecon`, `meshclean`, `texrecon`,
   `smvsrecon`, `smvsrecon_SSE41`. The OpenMVG executables will need to be registered here too.

Threads and processes never touch the GUI directly — they post `wxCommandEvent`s back to
`Regard3DMainFrame` (`sendComputeMatchesFinishedEvent()`, `sendTriangulationFinishedEvent()`,
`sendUpdateProgressBarEvent()`, …), handled in the `OnXxxFinished()` methods.

### SfM specifics

- `src/Regard3DFeatures.cpp` — keypoint detection and description. Detectors: AKAZE, Fast-AKAZE,
  DOG (VLFeat), MSER, ORB, BRISK, GFTT, HARRIS, SimpleBlob, TBMR. Descriptors include LIOP
  (VLFeat-derived, in `src/thirdparty/liop`). `getKpSizeFactor()` holds the per-detector scaling.
- `src/R3DComputeMatches.cpp` — the pipeline's heavyweight (~2700 lines). Putative matching plus
  geometric filtering (F / E / H). Backends behind the "Matching algorithm" choice:
  `0 FLANN`, `1–3 KGraph (Fast/Medium/Precise)`, `4 Brute Force`, `5 MRPT`, `6–8 HNSW`.
  Implementations are in `src/utils/matcher_{kgraph,hnsw,mrpt,efanna}.h`.
- `src/utils/OpenMVGHelper.cpp` — the largest single file; SfM data I/O, SVG match visualisation,
  export to PMVS/CMVS, MVE, NVM (VisualSFM), MeshLab, and track colorization.
- `src/utils/OpenMVGExportToMVS.cpp` — export to the OpenMVS `.mvs` scene format.
- `src/utils/CameraDBLookup` + `UserCameraDB` — sensor-width database (`sensor_database.csv`) and a
  user-editable SQLite override, used to derive focal length from EXIF (`src/utils/ExifParser`).

## Conventions

- C++ with a wxWidgets flavour: `wxString` in GUI/path code, `std::string` at OpenMVG boundaries.
- Member variables carry a **trailing underscore** (`pMainFrame_`, `matchingAlgorithm_`);
  pointer members are conventionally prefixed `p`.
- Tabs for indentation in Regard3D-authored files; OpenMVG-derived files (parts of
  `R3DComputeMatches.cpp`, `OpenMVGExportToMVS.cpp`) keep upstream's 2-space style — match the
  surrounding block.
- Old-style uppercase CMake commands (`FIND_PACKAGE`, `SET`, `IF/ENDIF(cond)`) dominate
  `src/CMakeLists.txt`; newer additions use lowercase. Match the local style.
- Every source file starts with the MIT copyright header — copy it into new files.
- Branching: `master` is the release branch, `develop` is where work lands and is merged into
  `master` at release time.

## Direction of travel — library OpenMVG → OpenMVG command-line tools

**This is the guiding intent for current work.** Feature detection, matching and triangulation are
to move off the linked OpenMVG library and onto the **OpenMVG command-line executables**, because
tracking a set of executables across OpenMVG releases is far less work than tracking its C++ API.

That means the in-process thread mechanism should converge on the external-process mechanism already
used for densification and surfacing (`R3DDensificationProcess` / `R3DSurfaceGenProcess`): build an
ordered command list, run it through `wxProcess`, advance in `OnTerminate()`.

The relevant OpenMVG 2.0 tools (vcpkg installs them to
`C:\Projects\vcpkg\vcpkg\installed\x64-windows\tools\openmvg`):

| Step | Executable |
|---|---|
| Image listing / SfM_Data init | `openMVG_main_SfMInit_ImageListing` |
| Features | `openMVG_main_ComputeFeatures`, `openMVG_main_ComputeFeatures_OpenCV` |
| Pair generation | `openMVG_main_PairGenerator` |
| Putative matching | `openMVG_main_ComputeMatches` |
| Geometric filtering (F/E/H) | `openMVG_main_GeometricFilter` |
| Triangulation / SfM | `openMVG_main_SfM` (incremental, incrementalv2, global) |
| Colorization | `openMVG_main_ComputeSfM_DataColor` |
| Exports | `openMVG_main_openMVG2PMVS`, `…2MVE2`, `…2openMVS`, `…2MESHLAB`, `…2NVM`, … |

`../openmvg_bla.txt` in the workspace root holds captured `--help` output for
`ComputeFeatures`, `ComputeFeatures_OpenCV`, `ComputeMatches` and `SfM` — the initial scoping notes
for this migration.

Consequences worth knowing before "fixing" anything that looks broken:

- **The console output window being dead is expected for now.** Since the CLI tools will write to
  stdout/stderr and be captured like the existing external processes already are, re-plumbing
  minilog into the linked library would be throwaway work.
- Reconstruction quality/parameter surface will change: the CLI exposes OpenMVG's own describers
  (SIFT, SIFT_ANATOMY, AKAZE_FLOAT, AKAZE_MLDB, plus AKAZE_OPENCV/SIFT_OPENCV) and its own
  nearest-neighbour methods, not Regard3D's custom detector/descriptor combinations
  (Fast-AKAZE, LIOP, TBMR, …) or its custom matchers (KGraph, MRPT, HNSW, EFANNA). Decide per
  option whether it survives the move — and remember GUI indices are persisted in project files.
- The vendored `src/thirdparty/{akaze,fast-akaze,liop,efanna,mrpt,hnswlib,kgraph}` and the matcher
  headers in `src/utils/matcher_*.h` exist only to serve the in-process path; they become removable
  as it retires.
- OpenMVG's binary/text intermediate files (`sfm_data.json/bin`, `matches.putative.*`,
  `matches.f/e/h.*`) become the interface between steps, so `R3DProjectPaths` filenames must line up
  with what the tools expect on the command line.

## Current state — in-flight build migration (uncommitted)

`git status` shows **modified but uncommitted** work moving the build from the old hand-built
dependency stack to **vcpkg + OpenMVG 2.0**. This is the enabling step for the CLI migration above:

- **OpenMVG 1.4 → 2.0.** `C_Progress_display` (`third_party/progress/progress.hpp`) was replaced
  upstream by `openMVG::system::LoggerProgress` (`openMVG/system/loggerprogress.hpp`); this is
  already converted in `OpenMVGHelper.cpp` but merely commented out in `OpenMVGExportToMVS.cpp`.
- **minilog was dropped.** OpenMVG 2.0 no longer ships it, so every `MLOG <<` call is commented out
  and `Regard3DConsoleOutputFrame`'s mutex adapter is disabled. Deliberately *not* replaced — see
  above.
- **kgraph was disabled.** `thirdparty/kgraph` is no longer added to the build, and
  `ArrayMatcher_kgraph`, `kgraph_match()` and `performParamsOptimization()` in
  `R3DComputeMatches.cpp` are commented out. Matching algorithms 1–3 in the GUI
  ("KGraph - Fast / Medium / Precise") therefore hit an empty dispatch branch and produce zero
  matches. Resolved by the CLI migration; until then it is a live trap for anyone testing matching.
- **AssImp** switched from the versioned `assimp-4.0` module path to `find_package(assimp CONFIG)`
  and the `assimp::assimp` imported target.
- Assorted MSVC warning fixes (float literal suffixes, explicit casts) plus an `#undef isnan` after
  the VLFeat headers in `Regard3DFeatures.cpp`.

A Debug build in `build_2022/` does currently link and produce `Regard3D.exe`.

## Things to watch

- Changing anything serialized in `R3DProject` breaks existing project files unless versioned.
- GUI enum indices (matching algorithm, camera model, densification/surface type) are persisted as
  integers — reordering a `wxChoice` silently changes the meaning of old projects.
- macOS/Linux code paths still exist throughout `CMakeLists.txt` and `R3DExternalPrograms.cpp` but
  are untested in the current vcpkg setup; don't delete them casually, but don't assume they work.
