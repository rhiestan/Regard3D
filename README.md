
Regard3D
========


Introduction
------------

This is the source repository of Regard3D, an open source structure-from-motion program based on OpenMVG.
Please find the documentation on the [Regard3D homepage](http://www.regard3d.org).

License
-------

The Regard3D source code is released under the [MIT license](http://opensource.org/licenses/mit-license.php). The numerous third-party libraries have other licenses, please see there.


Version history
---------------

### Version 1.1.0, 26.11.2026:
- Major overhaul, most work done by Claude Code
- OpenMVG 2.1, using external command-line tools for features, matches, triangulation
- Triangulation: Intrinsic and extrinsic refinement, the triangulation and resection methods, the camera model for unknown cameras, and which matches file to use.
- New matching options: mutual matching only, the GEMML2 matcher, and two-pass pre-emptive matching.
- New pair mode: VLAD image retrieval (openMVG_main_ComputeVLAD), which matches only the image pairs it selects.
- Regard3DMVS: a new densification method.
- External tools (OpenMVG, MVE, PoissonRecon, texrecon) are performance-optimized

### Version 1.0.0, 14.03.2019:
- Upgraded to OpenMVG 1.4
- Upgraded to OpenCV 4.0
- Added new Incremental pipeline of OpenMVG
- GPS information can be used now

### Version 0.9.4, 04.01.2018:
- Upgraded to OpenMVG 1.3
- Added SMVS as new densification tool
- Added new algorithms for pairwise feature matching: KGraph, MRPT

### Version 0.9.3, 15.09.2017:
- Added user-defined camera sensor width database, added ability to set focus length
- Added possibility to set camera model (Pinhole with various distortions, among them Fisheye)
- Removed PCL (point cloud library) as a dependency
- Fixed issue #9, JPEGs with EXIF data about camera orientation

### Version 0.9.2, 19.08.2017:
- Upgraded to OpenMVG 1.2, AssImp 4.0.1
- Added Fast-AKAZE, TBMR keypoint detectors
- Bugfix: Export of textured surfaces now works again (Issue #4)

### Version 0.9.1, 01.02.2017:
- Upgraded to OpenMVG 1.1
- Third-party programs (MVE, PoissonRecon) up to date

### Version 0.9.0, 15.10.2015:
- Upgraded to OpenMVG 0.9.0
- Third-party programs (MVE, PoissonRecon) up to date
- Mac: OpenMP-enabled build

### Version 0.8.0, 30.06.2015:
- Upgraded to OpenMVG 0.8.1
- Changed multithreading in compute matches step

### Version 0.7.1, 12.05.2015:
- Fixed bug when using CMVS
- Improved speed of Windows version
- Added multithreading in computing matches on Mac OS X (using TBB)
- Surfaces with textures (OBJ file format) are now loaded with AssImp
- "Export to CMPMVS" renamed to "Export to external MVS", MeshRecon format added

### Version 0.7.0, 28.04.2015
Initial version, based on OpenMVG 0.7


System requirements
-------------------

* A 64-bit x86 processor (x86_64)
* An OpenGL capable graphics card/chip
* Windows: Windows 10 or 11, 64-bit edition. The installer also installs the Microsoft Visual C++
  Redistributable; the portable zip contains its DLLs and runs without installing anything.
* Linux (AppImage): a distribution with glibc 2.35 or newer, e.g. Ubuntu 22.04, Linux Mint 21,
  Debian 12, Fedora 36, openSUSE Leap 16 or newer. Not RHEL/Rocky/AlmaLinux 9 (glibc 2.34).
  The system has to provide GTK 3, X11 or Wayland, OpenGL (`libGL.so.1`) and FUSE (`fusermount`)
  to mount the AppImage, which every desktop installation has. Without FUSE, run it with
  `--appimage-extract-and-run`.


Documentation, building
--------

For documentation and building instructions, see the [Regard3D homepage](http://www.regard3d.org).
