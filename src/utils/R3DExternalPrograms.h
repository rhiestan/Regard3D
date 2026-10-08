/**
 * Copyright (C) 2015 Roman Hiestand
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
 * LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#ifndef R3DEXTERNALPROGRAMS_H
#define R3DEXTERNALPROGRAMS_H

class R3DExternalPrograms
{
public:

	bool initialize();

	const wxString &getCMVSPath() { return cmvsPath_; }
	const wxString &getPMVSPath() { return pmvsPath_; }
	const wxString &getGenOptionPath() { return genOptionPath_; }
	const wxString &getPoissonReconPath() { return poissonReconPath_; }
	const wxString &getSurfaceTrimmerPath() { return surfaceTrimmerPath_; }
	const wxString &getMakescenePath() { return makescenePath_; }
	const wxString &getTexReconPath() { return texreconPath_; }
	const wxString &getDMReconPath() { return dmreconPath_; }
	const wxString &getScene2PsetPath() { return scene2psetPath_; }
	const wxString &getFSSReconPath() { return fssreconPath_; }
	const wxString &getMeshCleanPath() { return meshcleanPath_; }
	const wxString &getCMPMVSPath() { return cmpmvsPath_; }
	const wxString &getSMVSReconPath() { return smvsreconPath_; }
	const wxString &getSMVSReconSSE41Path() { return smvsreconSSE41Path; }
	// COLMAP is shipped as two separate builds (colmap_cuda/ and colmap_nocuda/);
	// empty when the corresponding directory wasn't found next to the executable.
	// Note: colmap_nocuda cannot run densification - COLMAP's dense stereo
	// (patch_match_stereo) has no CPU implementation and hard-fails at runtime
	// even in a "without GPU support" build. colmap_nocuda is only useful for
	// COLMAP subcommands that don't need dense stereo.
	const wxString &getColmapCudaPath() { return colmapCudaPath_; }
	const wxString &getColmapNoCudaPath() { return colmapNoCudaPath_; }
	// True if an NVIDIA/CUDA driver was detected on this machine. This is only
	// a signal about the driver, not about whether colmap_cuda/ is installed;
	// callers wanting a runnable path should also check getColmapCudaPath().
	bool hasCudaDriver() { return cudaDriverDetected_; }
	// Whichever COLMAP build is installed, preferring colmap_cuda; empty if
	// neither is. Unlike densification, feature extraction/matching and the
	// mapper all have a working CPU path in COLMAP, so either build is a
	// legitimate choice for those steps.
	const wxString &getBestColmapPath()
	{
		return colmapCudaPath_.IsEmpty() ? colmapNoCudaPath_ : colmapCudaPath_;
	}
	// Directory holding the Graphviz tools, empty when they are not installed
	const wxString &getGraphvizPath() { return graphvizPath_; }

	// OpenMVG command line tools, empty when they are not installed
	const wxString &getComputeFeaturesPath() { return computeFeaturesPath_; }
	const wxString &getComputeFeaturesOpenCVPath() { return computeFeaturesOpenCVPath_; }
	const wxString &getPairGeneratorPath() { return pairGeneratorPath_; }
	const wxString &getComputeMatchesPath() { return computeMatchesPath_; }
	const wxString &getGeometricFilterPath() { return geometricFilterPath_; }
	const wxString &getSfMPath() { return sfmPath_; }
	const wxString &getOpenMVG2PMVSPath() { return openMVG2PMVSPath_; }
	const wxString &getOpenMVG2MVE2Path() { return openMVG2MVE2Path_; }
	const wxString &getOpenMVG2ColmapPath() { return openMVG2ColmapPath_; }
	const wxString &getOpenMVG2AgisoftPath() { return openMVG2AgisoftPath_; }
	const wxString &getOpenMVG2WebGLPath() { return openMVG2WebGLPath_; }
	const wxString &getConvertSfMDataFormatPath() { return convertSfMDataFormatPath_; }
	const wxString &getOpenMVG2MeshLabPath() { return openMVG2MeshLabPath_; }
	const wxString &getOpenMVG2NVMPath() { return openMVG2NVMPath_; }
	const wxString &getOpenMVG2CMPMVSPath() { return openMVG2CMPMVSPath_; }
	const wxString &getOpenMVG2openMVSPath() { return openMVG2openMVSPath_; }

	// OpenMVS command line tools (openmvs/), not shipped with Regard3D: each is
	// empty when it is not installed, so every caller has to check before
	// offering the step that needs it.
	const wxString &getInterfaceCOLMAPPath() { return interfaceCOLMAPPath_; }
	const wxString &getDensifyPointCloudPath() { return densifyPointCloudPath_; }
	const wxString &getReconstructMeshPath() { return reconstructMeshPath_; }
	const wxString &getRefineMeshPath() { return refineMeshPath_; }
	const wxString &getTextureMeshPath() { return textureMeshPath_; }

	/**
	 * Whether an OpenMVS densification of a triangulation can run.
	 *
	 * The scene gets into OpenMVS in one of two ways: an OpenMVG triangulation
	 * through openMVG_main_openMVG2openMVS, a COLMAP-native one through
	 * colmap image_undistorter + InterfaceCOLMAP (either COLMAP build will do,
	 * undistortion has a CPU path). If not, reason says what is missing.
	 */
	bool isOpenMVSDensificationPossible(bool colmapTriangulation, wxString &reason);

	/**
	 * Moves the log file an OpenMVS tool wrote into targetDir, echoing it to
	 * std::cout first.
	 *
	 * OpenMVS writes nothing to a redirected stdout, everything goes into
	 * "<tool>-<unique>.log" in its working folder instead - the project
	 * directory, since that is what all project paths are relative to. Echoing
	 * it makes it show up in the console output window like every other tool's
	 * output, moving it keeps the project directory clean.
	 */
	static void collectOpenMVSLog(const wxString &workingDir, const wxString &toolName,
		const wxString &targetDir);
	static bool isOpenMVSTool(const wxString &stepName);

	/**
	 * Deletes depth maps DensifyPointCloud left in its working folder.
	 *
	 * They are named depth0000.dmap etc. without any reference to the scene,
	 * and DensifyPointCloud reuses whatever it finds there instead of
	 * recomputing it. --remove-dmaps cleans up after a successful run, this
	 * after an aborted or failed one, so the next densification cannot pick
	 * up another scene's depth maps.
	 */
	static void removeOpenMVSDepthMaps(const wxString &workingDir);

	const wxArrayString &getAllPaths() { return allPaths_; }

	static R3DExternalPrograms &getInstance() { return instance_; }

private:
	R3DExternalPrograms();
	virtual ~R3DExternalPrograms();
	bool checkExecutable(const wxString &path, const wxString &name, const wxString &extension,
		wxString &outFullPath);

	bool initialized_;
	wxString cmvsPath_, pmvsPath_, genOptionPath_;
	wxString poissonReconPath_, surfaceTrimmerPath_;
	wxString makescenePath_, texreconPath_;
	wxString dmreconPath_, scene2psetPath_, fssreconPath_, meshcleanPath_;
	wxString cmpmvsPath_;
	wxString smvsreconPath_, smvsreconSSE41Path;
	wxString colmapCudaPath_, colmapNoCudaPath_;
	bool cudaDriverDetected_;
	wxString graphvizPath_;
	wxString computeFeaturesPath_, computeFeaturesOpenCVPath_;
	wxString pairGeneratorPath_, computeMatchesPath_, geometricFilterPath_;
	wxString sfmPath_;
	wxString openMVG2PMVSPath_;
	wxString openMVG2MVE2Path_;
	wxString openMVG2ColmapPath_;
	wxString openMVG2AgisoftPath_;
	wxString openMVG2WebGLPath_;
	wxString convertSfMDataFormatPath_;
	wxString openMVG2MeshLabPath_;
	wxString openMVG2NVMPath_;
	wxString openMVG2CMPMVSPath_;
	wxString openMVG2openMVSPath_;
	wxString interfaceCOLMAPPath_, densifyPointCloudPath_;
	wxString reconstructMeshPath_, refineMeshPath_, textureMeshPath_;
	wxArrayString allPaths_;

	static R3DExternalPrograms instance_;
};

#endif
