
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

###Version 0.8.0, 30.06.2015:
- Upgraded to OpenMVG 0.8.1
- Changed multithreading in compute matches step

###Version 0.7.1, 12.05.2015:
- Fixed bug when using CMVS
- Improved speed of Windows version
- Added multithreading in computing matches on Mac OS X (using TBB)
- Surfaces with textures (OBJ file format) are now loaded with AssImp
- "Export to CMPMVS" renamed to "Export to external MVS", MeshRecon format added

###Version 0.7.0, 28.04.2015
Initial version, based on OpenMVG 0.7


System requirements
-------------------

* Windows 7 or newer, 64 bit edition
* Mac OS X 10.7 or newer
* OpenGL capable graphics card/chip


Documentation, building
--------

For documentation and building instructions, see the [Regard3D homepage](http://www.regard3d.org).
