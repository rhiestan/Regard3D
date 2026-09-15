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


#include "CommonIncludes.h"
#include "R3DDensificationProcess.h"
#include "Regard3DMainFrame.h"
#include "R3DExternalPrograms.h"

#include <wx/textfile.h>

#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
	// Project paths are user chosen and regularly contain spaces
	wxString quoted(const wxString &str)
	{
		return wxT("\"") + str + wxT("\"");
	}
}
#include "cpuinfo.hpp"

#include <iostream>


R3DDensificationProcess::R3DDensificationProcess(Regard3DMainFrame *pMainFrame)
	: wxProcess(pMainFrame), pMainFrame_(pMainFrame),
	processId_(0), checkForClusters_(false), wasCancelled_(false), isOK_(true),
	writePMVSOptions_(false),
	numberOfClusters_(0),
	fixColmapSparseModel_(false)
{
}

R3DDensificationProcess::~R3DDensificationProcess()
{
}

bool R3DDensificationProcess::runDensificationProcess(R3DProject::Densification *pDensification)
{
	R3DProject *pProject = R3DProject::getInstance();
	R3DProjectPaths paths;
	if(!pProject->getProjectPathsDns(paths, pDensification))
		return false;
	pDensification_ = pDensification;

	beginTime_ = wxDateTime::UNow();

#if wxCHECK_VERSION(2, 9, 0)
	env_.cwd = paths.absoluteProjectPath_;		// All paths are relative to project

	// Set PATH/LD_LIBRARY_PATH/DYLD_LIBRARY_PATH environment variable
#if defined(R3D_WIN32)
	wxString envVarName(wxT("PATH"));
#elif defined(R3D_MACOSX)
	wxString envVarName(wxT("DYLD_LIBRARY_PATH"));
#else
	wxString envVarName(wxT("LD_LIBRARY_PATH"));
#endif

	wxEnvVariableHashMap envVars;
	if(wxGetEnvMap(&envVars))
	{
		wxString concatPaths;
		const wxArrayString &exePaths = R3DExternalPrograms::getInstance().getAllPaths();
		for(size_t i = 0; i < exePaths.GetCount(); i++)
		{
			const wxString &curPath = exePaths[i];

			if(!concatPaths.IsEmpty())
#if defined(R3D_WIN32)
				concatPaths.Append(wxT(";"));
#else
				concatPaths.Append(wxT(":"));
#endif

			concatPaths.Append(curPath);
		}
		wxEnvVariableHashMap::iterator iter = envVars.find(envVarName);
		if(iter != envVars.end())
		{
			wxString val = iter->second;
			if(!val.IsEmpty())
#if defined(R3D_WIN32)
				val.Append(wxT(";"));
#else
				val.Append(wxT(":"));
#endif
			val.Append(concatPaths);
			envVars[envVarName] = val;
		}
		else
		{
			envVars[envVarName] = concatPaths;
		}
		env_.env = envVars;
	}
#endif

	relativePMVSOutPath_ = wxString( paths.relativePMVSOutPath_.c_str(), wxConvLibc );

	cmds_.Clear();
	progressTexts_.Clear();
	stepNames_.Clear();

	if(pDensification->densificationType_ == R3DProject::DTCMVSPMVS)
	{
		wxString pmvsPath(relativePMVSOutPath_);
		pmvsPath.Append(wxT("/"));

		// The scene is exported by openMVG_main_openMVG2PMVS. It creates a
		// "PMVS" directory below -o, which is exactly relativePMVSOutPath_,
		// so what it has to be given is the densification directory.
		const wxString openMVG2PMVSExe = R3DExternalPrograms::getInstance().getOpenMVG2PMVSPath();
		cmds_.Add(quoted(openMVG2PMVSExe)
			+ wxT(" -i ") + quoted(wxString(paths.relativeTriSfmDataFilename_.c_str(), wxConvLibc))
			+ wxT(" -o ") + quoted(wxString(paths.relativeDensificationPath_.c_str(), wxConvLibc))
			+ wxT(" -r 1")		// Full resolution, as the built-in export used
			+ wxString::Format(wxT(" -c %d"), pDensification->pmvsNumThreads_)
			+ wxString::Format(wxT(" -v %d"), (pDensification->useCMVS_ ? 1 : 0)));
		progressTexts_.Add(wxT("Exporting project to PMVS"));
		stepNames_.Add(wxT("openMVG_main_openMVG2PMVS"));
		writePMVSOptions_ = true;

		wxString pmvsExe = R3DExternalPrograms::getInstance().getPMVSPath();
		wxString cmvsExe = R3DExternalPrograms::getInstance().getCMVSPath();
		wxString genOptionExe = R3DExternalPrograms::getInstance().getGenOptionPath();
		if(pDensification->useCMVS_)
		{
			cmds_.Add(cmvsExe + wxString(wxT(" ")) + pmvsPath
				+ wxString::Format(wxT(" %d %d"),
				pDensification->pmvsMaxClusterSize_, pDensification->pmvsNumThreads_));
			progressTexts_.Add(wxT("Clustering images (CMVS)"));
			stepNames_.Add(wxT("cmvs"));
			cmds_.Add(genOptionExe + wxString(wxT(" ")) + pmvsPath
				+ wxString::Format(wxT(" %d %d %f %d %d %d"),
				pDensification->pmvsLevel_, pDensification->pmvsCSize_,
				pDensification->pmvsThreshold_, pDensification->pmvsWSize_,
				pDensification->pmvsMinImageNum_, pDensification->pmvsNumThreads_));
			progressTexts_.Add(wxT("Generating options"));
			stepNames_.Add(wxT("genOption"));

			pDensification->finalDenseModelName_ = wxT("option-0000.ply");

			// CMVS will run at a later stage, check whether more than one option-xxxx files exist
			checkForClusters_ = true;
		}
		else
		{
			wxString cmdLine = pmvsExe + wxString(wxT(" ")) + pmvsPath + wxString(wxT(" pmvs_options.txt"));
			cmds_.Add(cmdLine);
			progressTexts_.Add(wxT("Densify point cloud (PMVS)"));
			stepNames_.Add(wxT("pmvs2"));

			pDensification->finalDenseModelName_ = wxT("pmvs_options.txt.ply");
		}
	}
	else if(pDensification->densificationType_ == R3DProject::DTMVE)
	{
		// The MVE scene is written by openMVG_main_openMVG2MVE2, which creates
		// the "MVE" directory below its -o. Only once: densification and
		// surface generation share the scene, and it is expensive to write.
		if(!wxFileName::DirExists(wxString(paths.relativeMVESceneDir_.c_str(), wxConvLibc)))
		{
			cmds_.Add(quoted(R3DExternalPrograms::getInstance().getOpenMVG2MVE2Path())
				+ wxT(" -i ") + quoted(wxString(paths.relativeTriSfmDataFilename_.c_str(), wxConvLibc))
				+ wxT(" -o ") + quoted(wxString(paths.relativeOutPath_.c_str(), wxConvLibc)));
			progressTexts_.Add(wxT("Exporting project to MVE"));
			stepNames_.Add(wxT("openMVG_main_openMVG2MVE2"));
		}

		wxString dmreconExe = R3DExternalPrograms::getInstance().getDMReconPath();
		wxString scene2psetExe = R3DExternalPrograms::getInstance().getScene2PsetPath();
		wxString mveSceneDir = wxString(paths.relativeMVESceneDir_.c_str(), wxConvLibc);
		wxString outputModelFilename = wxT("mve_model.ply");

		cmds_.Add(dmreconExe + wxString::Format(wxT(" --scale=%d --filter-width=%d --force "),
			pDensification->mveScale_, pDensification->mveFilterWidth_) + mveSceneDir);
		progressTexts_.Add(wxT("Densify point cloud (MVE)"));
		stepNames_.Add(wxT("dmrecon"));

		wxFileName outputModelFN(wxString(paths.relativeDensificationPath_.c_str(), wxConvLibc), outputModelFilename);
		cmds_.Add(scene2psetExe
			+ wxString::Format(wxT(" -ddepth-L%d -iundist-L%d -n -s -c "), pDensification->mveScale_, pDensification->mveScale_)
			+ mveSceneDir + wxT(" ") + outputModelFN.GetFullPath());
		progressTexts_.Add(wxT("Generating point cloud model (MVE)"));
		stepNames_.Add(wxT("scene2pset"));

		pDensification->finalDenseModelName_ = outputModelFilename;
	}
	else if(pDensification->densificationType_ == R3DProject::DTSMVS)
	{
		// The MVE scene is written by openMVG_main_openMVG2MVE2, which creates
		// the "MVE" directory below its -o. Only once: densification and
		// surface generation share the scene, and it is expensive to write.
		if(!wxFileName::DirExists(wxString(paths.relativeMVESceneDir_.c_str(), wxConvLibc)))
		{
			cmds_.Add(quoted(R3DExternalPrograms::getInstance().getOpenMVG2MVE2Path())
				+ wxT(" -i ") + quoted(wxString(paths.relativeTriSfmDataFilename_.c_str(), wxConvLibc))
				+ wxT(" -o ") + quoted(wxString(paths.relativeOutPath_.c_str(), wxConvLibc)));
			progressTexts_.Add(wxT("Exporting project to MVE"));
			stepNames_.Add(wxT("openMVG_main_openMVG2MVE2"));
		}

		wxString smvsreconExe = R3DExternalPrograms::getInstance().getSMVSReconPath();
		cpuid::cpuinfo cpuinfo;
		if(cpuinfo.has_sse4_1())		// TODO: Allow user to fall back to generic version
			smvsreconExe = R3DExternalPrograms::getInstance().getSMVSReconSSE41Path();

		wxString mveSceneDir = wxString(paths.relativeMVESceneDir_.c_str(), wxConvLibc);
		wxString outputModelFilename;
		outputModelFilename.Printf(wxT("smvs-%s%d.ply"),
			(pDensification->smvsEnableShadingBasedOptimization_ ? wxT("S") : wxT("B")),
			pDensification->smvsInputScale_);

		cmds_.Add(smvsreconExe + wxString::Format(wxT(" --scale=%d --output-scale=%d %s %s --alpha=%f --force "),
			pDensification->smvsInputScale_, pDensification->smvsOutputScale_,
			(pDensification->smvsEnableShadingBasedOptimization_ ? wxT("-S") : wxT("")),
			(pDensification->smvsEnableSemiGlobalMatching_ ? wxT("") : wxT("--no-sgm")),
			pDensification->smvsAlpha_)
			+ mveSceneDir);
		progressTexts_.Add(wxT("Densify point cloud (SMVS)"));
		stepNames_.Add(wxT("smvsrecon"));

		wxFileName outputModelFN(wxString(paths.relativeDensificationPath_.c_str(), wxConvLibc), outputModelFilename);

		pDensification->finalDenseModelName_ = outputModelFilename;
	}
	else if(pDensification->densificationType_ == R3DProject::DTCOLMAP)
	{
		// COLMAP is driven entirely through its own CLI. Normally in four steps:
		//   openMVG_main_openMVG2Colmap  -> cameras.txt/images.txt/points3D.txt (a sparse model)
		//   colmap image_undistorter    -> undistorted images + a binary sparse model
		//   colmap patch_match_stereo   -> per-image depth/normal maps (needs a CUDA GPU)
		//   colmap stereo_fusion        -> colmap_fused.ply
		// When the triangulation itself already ran COLMAP's mapper (see
		// R3DProject::Triangulation::computeEngine_), its sparse model is used
		// directly and the export step is skipped - there is nothing to
		// convert, and nothing to patch either (fixColmapPoints3DFile exists
		// only to work around a units quirk of the OpenMVG export).
		const bool nativeColmapTriangulation = (paths.triangulationEngine_ == 2);
		const wxString densificationDir(paths.relativeDensificationPath_.c_str(), wxConvLibc);
		const wxString imagePath(quoted(wxString(paths.relativeImagePath_.c_str(), wxConvLibc)));
		wxString sparsePath;
		if(nativeColmapTriangulation)
		{
			wxFileName nativeSparseFN(wxString(paths.relativeColmapModelPath_.c_str(), wxConvLibc), wxEmptyString);
			nativeSparseFN.AppendDir(wxT("0"));
			sparsePath = quoted(nativeSparseFN.GetPath(wxPATH_GET_VOLUME));
			fixColmapSparseModel_ = false;
		}
		else
		{
			wxFileName sparseFN(densificationDir, wxEmptyString);
			sparseFN.AppendDir(wxT("colmap_sparse"));
			relativeColmapSparsePath_ = sparseFN.GetPath(wxPATH_GET_VOLUME);
			sparsePath = quoted(relativeColmapSparsePath_);
			fixColmapSparseModel_ = true;
		}
		wxFileName denseFN(densificationDir, wxEmptyString);
		denseFN.AppendDir(wxT("colmap_dense"));
		const wxString densePath(quoted(denseFN.GetPath(wxPATH_GET_VOLUME)));
		const wxString outputModelFilename(wxT("colmap_fused.ply"));
		wxFileName outputModelFN(densificationDir, outputModelFilename);
		const wxString outputModelPath(quoted(outputModelFN.GetFullPath()));

		// -1 means "original size": leave the size arguments off, COLMAP's own
		// default is -1 as well
		wxString maxImageSizeArg;
		if(pDensification->colmapMaxImageSize_ > 0)
			maxImageSizeArg = wxString::Format(wxT(" --max_image_size %d"), pDensification->colmapMaxImageSize_);
		wxString maxImageSizePMArg;
		if(pDensification->colmapMaxImageSize_ > 0)
			maxImageSizePMArg = wxString::Format(wxT(" --PatchMatchStereo.max_image_size %d"), pDensification->colmapMaxImageSize_);

		R3DExternalPrograms &extPrograms = R3DExternalPrograms::getInstance();
		const wxString openMVG2ColmapExe = extPrograms.getOpenMVG2ColmapPath();

		// COLMAP's patch_match_stereo has no CPU implementation - it hard-fails
		// at runtime even in a "without GPU support" build ("Dense stereo
		// reconstruction requires CUDA, which is not available on your
		// system."), so colmap_nocuda can never actually densify. Always use
		// colmap_cuda; if it's missing, the generic "could not be started"
		// error a few steps down catches it (Regard3DMainFrame also checks
		// this upfront before the process is even created).
		const wxString colmapExe = extPrograms.getColmapCudaPath();

		if(!nativeColmapTriangulation)
		{
			cmds_.Add(quoted(openMVG2ColmapExe)
				+ wxT(" -i ") + quoted(wxString(paths.relativeTriSfmDataFilename_.c_str(), wxConvLibc))
				+ wxT(" -o ") + sparsePath);
			progressTexts_.Add(wxT("Exporting project to Colmap"));
			stepNames_.Add(wxT("openMVG_main_openMVG2Colmap"));
		}

		cmds_.Add(quoted(colmapExe) + wxT(" image_undistorter")
			+ wxT(" --image_path ") + imagePath
			+ wxT(" --input_path ") + sparsePath
			+ wxT(" --output_path ") + densePath
			+ wxT(" --output_type COLMAP")
			+ maxImageSizeArg);
		progressTexts_.Add(wxT("Undistorting images (COLMAP)"));
		stepNames_.Add(wxT("colmap image_undistorter"));

		cmds_.Add(quoted(colmapExe) + wxT(" patch_match_stereo")
			+ wxT(" --workspace_path ") + densePath
			+ maxImageSizePMArg
			+ wxString::Format(wxT(" --PatchMatchStereo.window_radius %d"), pDensification->colmapWindowRadius_)
			+ wxString::Format(wxT(" --PatchMatchStereo.geom_consistency %d"), (pDensification->colmapGeomConsistency_ ? 1 : 0))
			+ wxString::Format(wxT(" --PatchMatchStereo.filter %d"), (pDensification->colmapFilter_ ? 1 : 0)));
		progressTexts_.Add(wxT("Computing depth maps (COLMAP)"));
		stepNames_.Add(wxT("colmap patch_match_stereo"));

		// Fusing from photometric-only depth maps needs --input_type photometric;
		// otherwise stereo_fusion looks for the geometric ones patch_match_stereo
		// just wrote
		cmds_.Add(quoted(colmapExe) + wxT(" stereo_fusion")
			+ wxT(" --workspace_path ") + densePath
			+ wxT(" --input_type ") + (pDensification->colmapGeomConsistency_ ? wxT("geometric") : wxT("photometric"))
			+ wxT(" --output_type PLY")
			+ wxT(" --output_path ") + outputModelPath
			// Read with the C locale, so not wxString::Format
			+ wxT(" --StereoFusion.max_reproj_error ") + wxString::FromCDouble(pDensification->colmapMaxReprojError_, 2));
		progressTexts_.Add(wxT("Fusing point cloud (COLMAP)"));
		stepNames_.Add(wxT("colmap stereo_fusion"));

		pDensification->finalDenseModelName_ = outputModelFilename;
	}

	if(cmds_.IsEmpty())
	{
		// An unimplemented/unrecognized densification type: nothing was
		// queued, so OnTerminate would never run and the progress dialog
		// would otherwise sit there forever
		isOK_ = false;
		errorMessage_ = wxT("This densification method is not implemented.");
		pMainFrame_->sendDensificationFinishedEvent();
		return false;
	}

	runSingleCommand();

	return (processId_ > 0);
}

void R3DDensificationProcess::readConsoleOutput()
{
	// Forwarded to std::cout/std::cerr, where the console output window picks
	// it up the same way as the OpenMVG library's own logging
	wxInputStream *pIn = GetInputStream();
	if(pIn != NULL)
	{
		std::string buf;
		while(pIn->CanRead())
		{
			int curc = pIn->GetC();
			if(curc != wxEOF)
				buf.push_back( static_cast<char>(curc) );
		}

		if(!buf.empty())
			std::cout << buf << std::flush;
	}
	wxInputStream *pErr = GetErrorStream();
	if(pErr != NULL)
	{
		std::string buf;
		while(pErr->CanRead())
		{
			int curc = pErr->GetC();
			if(curc != wxEOF)
				buf.push_back( static_cast<char>(curc) );
		}

		if(!buf.empty())
			std::cerr << buf << std::flush;
	}
}

wxString R3DDensificationProcess::getRuntimeStr()
{
	wxTimeSpan runTime = wxDateTime::UNow() - beginTime_;
	wxString runTimeStr;
	if(runTime.GetHours() > 0)
		runTimeStr = runTime.Format(wxT("%H:%M:%S.%l"));
	else
		runTimeStr = runTime.Format(wxT("%M:%S.%l"));

	return runTimeStr;
}

void R3DDensificationProcess::cancel()
{
	if(wasCancelled_ || processId_ <= 0)
		return;

	wasCancelled_ = true;

	// Whatever is still queued would run on data the killed tool never wrote,
	// and the clusters CMVS was going to produce will not be there either
	cmds_.Clear();
	progressTexts_.Clear();
	stepNames_.Clear();
	checkForClusters_ = false;

	// wxKILL_CHILDREN in case the tool started helpers of its own
	wxProcess::Kill(processId_, wxSIGKILL, wxKILL_CHILDREN);
}

void R3DDensificationProcess::OnTerminate(int pid, int status)
{
	readConsoleOutput();	// Finish reading streams

	// This process is gone; cancel() must not kill a recycled pid
	processId_ = 0;

	// All these tools return a non-zero exit code on failure; stop here
	// instead of running the remaining steps on data the failed tool never
	// wrote, which otherwise just piles on confusing, unrelated errors
	bool stepFailed = false;
	if(wasCancelled_)
	{
		isOK_ = false;
		errorMessage_ = wxT("Aborted.");
	}
	else if(status != 0)
	{
		stepFailed = true;
		isOK_ = false;
		errorMessage_ = wxString::Format(
			wxT("%s returned with error code %d.\n\nPlease check the console output for details."),
			currentStepName_.c_str(), status);
	}

	if(writePMVSOptions_)
	{
		// The export was the first command of the queue, so this is the
		// moment its pmvs_options.txt exists
		writePMVSOptions_ = false;
		if(!wasCancelled_ && !stepFailed)
			writePMVSOptions();
	}

	if(fixColmapSparseModel_)
	{
		// The Colmap export was the first command of the queue, so this is
		// the moment its images.txt/points3D.txt exist, and image_undistorter
		// (the next command) needs the fixed-up version
		fixColmapSparseModel_ = false;
		if(!wasCancelled_ && !stepFailed && !fixColmapPoints3DFile())
		{
			stepFailed = true;
			isOK_ = false;
			errorMessage_ = wxT("Could not patch Colmap's points3D.txt after the export.\n\n")
				wxT("image_undistorter would fail to read the sparse model.");
		}
	}

	if(stepFailed)
	{
		// Whatever is still queued would run on data the failed tool never
		// wrote, and the clusters CMVS was going to produce will not be
		// there either
		cmds_.Clear();
		progressTexts_.Clear();
		stepNames_.Clear();
		checkForClusters_ = false;
	}

	if(cmds_.IsEmpty())
	{
		bool foundClusters = false;
		if(checkForClusters_)
		{
			wxString pmvsExe = R3DExternalPrograms::getInstance().getPMVSPath();
			wxString pmvsPath(relativePMVSOutPath_);
			pmvsPath.Append(wxT("/"));
			int i = 0;
			wxString optionFilename(wxString::Format(wxT("option-%04d"), i));
			wxFileName optionFN(relativePMVSOutPath_, optionFilename);
			while(optionFN.FileExists())
			{
				foundClusters = true;
				cmds_.Add(pmvsExe + wxString(wxT(" ")) + pmvsPath + wxString(wxT(" ")) + optionFilename);
				stepNames_.Add(wxT("pmvs2"));
				i++;
				optionFilename = wxString::Format(wxT("option-%04d"), i);
				optionFN.SetFullName(optionFilename);
			}

			numberOfClusters_ = i;
			for(int j = 0; j < i; j++)
				progressTexts_.Add(wxString::Format(wxT("Densify point cloud (PMVS) part %d/%d"),
					j + 1, i));

			checkForClusters_ = false;
			if(foundClusters)
				runSingleCommand();
		}

		if(!foundClusters)
			pMainFrame_->sendDensificationFinishedEvent();
	}
	else
		runSingleCommand();
}

/**
 * Puts the parameters of the densification dialog into pmvs_options.txt.
 *
 * openMVG_main_openMVG2PMVS writes that file with openMVG's own values and
 * has no options for these six. Everything else it wrote is kept, above all
 * timages, which counts the views it really exported.
 */
bool R3DDensificationProcess::writePMVSOptions()
{
	if(pDensification_ == NULL)
		return false;

	const wxFileName optionsFN(relativePMVSOutPath_, wxT("pmvs_options.txt"));
	wxTextFile optionsFile(optionsFN.GetFullPath());
	if(!optionsFile.Open())
		return false;

	for(size_t i = 0; i < optionsFile.GetLineCount(); i++)
	{
		const wxString key(optionsFile[i].BeforeFirst(wxT(' ')));

		if(key.IsSameAs(wxT("level")))
			optionsFile[i] = wxString::Format(wxT("level %d"), pDensification_->pmvsLevel_);
		else if(key.IsSameAs(wxT("csize")))
			optionsFile[i] = wxString::Format(wxT("csize %d"), pDensification_->pmvsCSize_);
		else if(key.IsSameAs(wxT("threshold")))
			// PMVS reads this with the C locale, so not wxString::Format
			optionsFile[i] = wxT("threshold ") + wxString::FromCDouble(pDensification_->pmvsThreshold_);
		else if(key.IsSameAs(wxT("wsize")))
			optionsFile[i] = wxString::Format(wxT("wsize %d"), pDensification_->pmvsWSize_);
		else if(key.IsSameAs(wxT("minImageNum")))
			optionsFile[i] = wxString::Format(wxT("minImageNum %d"), pDensification_->pmvsMinImageNum_);
		else if(key.IsSameAs(wxT("CPU")))
			optionsFile[i] = wxString::Format(wxT("CPU %d"), pDensification_->pmvsNumThreads_);
	}

	const bool isOK = optionsFile.Write();
	optionsFile.Close();

	return isOK;
}

/**
 * Patches openMVG_main_openMVG2Colmap's points3D.txt in place.
 *
 * The exporter writes each track observation as (IMAGE_ID, the raw openMVG
 * feature index), but Colmap's reader expects (IMAGE_ID, the position of
 * that observation within the *same* image's own POINTS2D[] line in
 * images.txt) -- otherwise it looks up the wrong entry and aborts with
 * "Check failed: point2D.point3D_id == point3D_id". Rebuilding that mapping
 * from the images.txt the export just wrote and rewriting points3D.txt's
 * indices accordingly is enough to fix it without touching openMVG itself.
 */
bool R3DDensificationProcess::fixColmapPoints3DFile()
{
	const wxFileName imagesFN(relativeColmapSparsePath_, wxT("images.txt"));
	const wxFileName points3DFN(relativeColmapSparsePath_, wxT("points3D.txt"));

	std::ifstream imagesIn(std::string(imagesFN.GetFullPath().mb_str(wxConvLibc)));
	if(!imagesIn.is_open())
		return false;

	// imageId -> (point3dId -> its position in that image's POINTS2D[] line)
	std::unordered_map<long long, std::unordered_map<long long, int>> pointIndexByImage;

	std::string line;
	while(std::getline(imagesIn, line))
	{
		if(line.empty() || line[0] == '#')
			continue;

		std::istringstream headerStream(line);
		long long imageId = -1;
		headerStream >> imageId;

		if(!std::getline(imagesIn, line))
			break;		// Malformed file, one header line without its points line

		std::istringstream pointsStream(line);
		double x, y;
		long long point3dId;
		int idx = 0;
		std::unordered_map<long long, int> &indexMap = pointIndexByImage[imageId];
		while(pointsStream >> x >> y >> point3dId)
			indexMap[point3dId] = idx++;
	}
	imagesIn.close();

	std::ifstream points3DIn(std::string(points3DFN.GetFullPath().mb_str(wxConvLibc)));
	if(!points3DIn.is_open())
		return false;

	std::ostringstream out;
	while(std::getline(points3DIn, line))
	{
		if(line.empty() || line[0] == '#')
		{
			out << line << "\n";
			continue;
		}

		// Split into tokens rather than parsing X/Y/Z/error as doubles, so
		// re-writing the line cannot lose precision on the coordinates
		std::vector<std::string> tokens;
		{
			std::istringstream lineStream(line);
			std::string tok;
			while(lineStream >> tok)
				tokens.push_back(tok);
		}
		// POINT3D_ID, X, Y, Z, R, G, B, ERROR, then (IMAGE_ID, POINT2D_IDX) pairs
		if(tokens.size() < 8 || ((tokens.size() - 8) % 2) != 0)
		{
			out << line << "\n";		// Not a track line we understand, leave as is
			continue;
		}

		const long long point3dId = std::atoll(tokens[0].c_str());
		out << tokens[0];
		for(size_t i = 1; i < 8; i++)
			out << " " << tokens[i];

		for(size_t i = 8; i + 1 < tokens.size(); i += 2)
		{
			const long long imageId = std::atoll(tokens[i].c_str());
			int fixedIdx = std::atoi(tokens[i + 1].c_str());		// Fallback if not found below

			std::unordered_map<long long, std::unordered_map<long long, int>>::const_iterator imgIt
				= pointIndexByImage.find(imageId);
			if(imgIt != pointIndexByImage.end())
			{
				std::unordered_map<long long, int>::const_iterator idxIt = imgIt->second.find(point3dId);
				if(idxIt != imgIt->second.end())
					fixedIdx = idxIt->second;
			}
			out << " " << imageId << " " << fixedIdx;
		}
		out << "\n";
	}
	points3DIn.close();

	std::ofstream points3DOut(std::string(points3DFN.GetFullPath().mb_str(wxConvLibc)), std::ios::trunc);
	if(!points3DOut.is_open())
		return false;
	points3DOut << out.str();

	return true;
}

void R3DDensificationProcess::runSingleCommand()
{
	if(!cmds_.IsEmpty())
	{
		Redirect();	// Redirect I/O, hide console window

		wxString cmdLine = cmds_[0];
		cmds_.RemoveAt(0);
		wxString progressText = progressTexts_[0];
		progressTexts_.RemoveAt(0);
		currentStepName_ = !stepNames_.IsEmpty() ? stepNames_[0] : progressText;
		if(!stepNames_.IsEmpty())
			stepNames_.RemoveAt(0);
		pMainFrame_->sendUpdateProgressBarEvent(-1.0f, progressText);

		// Cleanup
		if(GetInputStream() != NULL)
			delete GetInputStream();
		if(GetErrorStream() != NULL)
			delete GetErrorStream();
		if(GetOutputStream() != NULL)
			delete GetOutputStream();
		SetPipeStreams(NULL, NULL, NULL);
#if wxCHECK_VERSION(2, 9, 0)
		processId_ = wxExecute(cmdLine, wxEXEC_ASYNC, this, &env_);
#else
		processId_ = wxExecute(cmdLine, wxEXEC_ASYNC, this);
#endif

		if(processId_ <= 0)
		{
			// OnTerminate is never called for a process that never started
			isOK_ = false;
			errorMessage_ = wxString::Format(wxT("%s could not be started."), currentStepName_.c_str());
			cmds_.Clear();
			progressTexts_.Clear();
			stepNames_.Clear();
			checkForClusters_ = false;
			pMainFrame_->sendDensificationFinishedEvent();
		}
	}
}
