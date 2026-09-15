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
#include "R3DTriangulationProcess.h"
#include "R3DOpenMVGOptions.h"
#include "Regard3DMainFrame.h"
#include "R3DExternalPrograms.h"

#include <iostream>

namespace
{
	wxString quoted(const wxString &str)
	{
		return wxT("\"") + str + wxT("\"");
	}
}

R3DTriangulationProcess::R3DTriangulationProcess(Regard3DMainFrame *pMainFrame)
	: wxProcess(pMainFrame), pMainFrame_(pMainFrame), pTriangulation_(NULL),
	processId_(0), stepCount_(0), stepsDone_(0), isOK_(true), wasCancelled_(false)
{
}

R3DTriangulationProcess::~R3DTriangulationProcess()
{
}

bool R3DTriangulationProcess::runTriangulationProcess(R3DProject::Triangulation *pTriangulation)
{
	R3DProject *pProject = R3DProject::getInstance();
	pTriangulation_ = pTriangulation;

	beginTime_ = wxDateTime::UNow();

	if(!pProject->getProjectPathsTri(paths_, pTriangulation))
	{
		isOK_ = false;
		errorMessage_ = wxT("Could not determine the project paths for the triangulation.");
		finish();
		return false;
	}

#if wxCHECK_VERSION(2, 9, 0)
	env_.cwd = paths_.absoluteProjectPath_;		// All paths are relative to project

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

	if(pTriangulation->computeEngine_ == 2)
	{
		if(!buildColmapCommandList(paths_))
		{
			finish();
			return false;
		}

		stepCount_ = static_cast<int>(cmds_.GetCount());
		stepsDone_ = 0;

		runSingleCommand();

		return (processId_ > 0);
	}

	if(!buildCommand(paths_))
	{
		finish();
		return false;
	}

	Redirect();		// Redirect I/O, hide console window

	pMainFrame_->sendUpdateProgressBarEvent(-1.0f, wxT("Triangulation (OpenMVG)"));

	processId_ = wxExecute(cmd_, wxEXEC_ASYNC, this, &env_);
	if(processId_ <= 0)
	{
		// OnTerminate is never called for a process that never started
		isOK_ = false;
		errorMessage_ = wxT("openMVG_main_SfM could not be started.");
		finish();
		return false;
	}

	return true;
}

/**
 * Turns the stored parameters into the openMVG_main_SfM command line.
 *
 * The averaging methods use their long option names on purpose: main_SfM
 * registers -r and -t twice, for the resection and triangulation methods
 * first, so the short forms would never reach the global engine's options.
 */
bool R3DTriangulationProcess::buildCommand(const R3DProjectPaths &paths)
{
	const R3DOpenMVGTriangulationParams &params = pTriangulation_->openMVGParams_;
	const wxString sfmExe(R3DExternalPrograms::getInstance().getSfMPath());

	if(sfmExe.IsEmpty())
	{
		isOK_ = false;
		errorMessage_ = wxT("openMVG_main_SfM was not found.\n\n")
			wxT("Please put it into the subdirectory \"openmvg\" of the external\n")
			wxT("tools directory, or use the built-in triangulation.");
		return false;
	}

	const wxString sfmData(paths.matchesSfmDataFilename_.c_str(), wxConvLibc);
	const wxString matchesDir(paths.relativeMatchesPath_.c_str(), wxConvLibc);
	const wxString outDir(paths.relativeOutPath_.c_str(), wxConvLibc);

	// Which matches to reconstruct from. Without -M the tool would always take
	// matches.f.txt first, which is the wrong input for the global engine.
	wxString matchesFile(R3DOpenMVGOptions::matchesFileName(params.matchesFile_));
	if(matchesFile.IsEmpty())
		matchesFile = (pTriangulation_->algorithm_ == R3DProject::R3DTA_Global
			? wxT("matches.e.txt") : wxT("matches.f.txt"));

	cmd_ = quoted(sfmExe)
		+ wxT(" -i ") + quoted(sfmData)
		+ wxT(" -m ") + quoted(matchesDir)
		+ wxT(" -M ") + quoted(matchesFile)
		+ wxT(" -o ") + quoted(outDir)
		+ wxT(" -s ") + R3DOpenMVGOptions::sfmEngineName(pTriangulation_->algorithm_);

	// Bundle adjustment. The dialog's "Refine camera intrinsics" checkbox is
	// what switches the intrinsics off, the choice only says which of them.
	cmd_.Append(wxT(" -f "));
	cmd_.Append(pTriangulation_->refineIntrinsics_
		? R3DOpenMVGOptions::intrinsicRefinementName(params.intrinsicRefinement_)
		: wxT("NONE"));
	cmd_.Append(wxT(" -e "));
	cmd_.Append(R3DOpenMVGOptions::extrinsicRefinementName(params.extrinsicRefinement_));
	if(pTriangulation_->useGPSInfo_)
		cmd_.Append(wxT(" -P"));		// A switch, it takes no value

	if(pTriangulation_->algorithm_ == R3DProject::R3DTA_Global)
	{
		cmd_.Append(wxString::Format(wxT(" --rotationAveraging %d"),
			pTriangulation_->rotAveraging_));
		cmd_.Append(wxString::Format(wxT(" --translationAveraging %d"),
			pTriangulation_->transAveraging_));
	}
	else
	{
		cmd_.Append(wxString::Format(wxT(" --triangulation_method %d --resection_method %d -c %d"),
			params.triangulationMethod_, params.resectionMethod_, params.cameraModel_));

		if(pTriangulation_->algorithm_ == R3DProject::R3DTA_Incremental2)
		{
			cmd_.Append(wxT(" -S "));
			cmd_.Append(R3DOpenMVGOptions::sceneInitializerName(pTriangulation_->triInitialization_));
		}
		else
		{
			// The initial pair is given by image filename, not by view id
			wxString imageA, imageB;
			R3DProject *pProject = R3DProject::getInstance();
			R3DProject::Object *pObject = pProject->getObjectByTypeAndID(
				R3DProject::R3DTreeItem::TypePictureSet, paths.pictureSetId_);
			R3DProject::PictureSet *pPictureSet = dynamic_cast<R3DProject::PictureSet *>(pObject);
			if(pPictureSet != NULL)
			{
				const ImageInfoVector &iiv = pPictureSet->getImageInfoVector();
				if(pTriangulation_->initialImageIndexA_ < iiv.size())
					imageA = iiv[pTriangulation_->initialImageIndexA_].importedFilename_;
				if(pTriangulation_->initialImageIndexB_ < iiv.size())
					imageB = iiv[pTriangulation_->initialImageIndexB_].importedFilename_;
			}

			// Both or neither: with only one of them openMVG_main_SfM fails
			// instead of falling back to its own pair selection
			if(!imageA.IsEmpty() && !imageB.IsEmpty() && imageA != imageB)
			{
				cmd_.Append(wxT(" -a ") + quoted(imageA));
				cmd_.Append(wxT(" -b ") + quoted(imageB));
			}
		}
	}

	return true;
}

/**
 * COLMAP variant: mapper reconstructs from the database a COLMAP ComputeMatches
 * wrote, then model_converter turns its sparse model into FinalColorized.ply -
 * COLMAP extracted point colors itself while mapping, so this is directly the
 * file Regard3DMainFrame looks for after a triangulation finishes, whichever
 * engine produced it.
 *
 * mapper always writes below output_path/<model id>, numbering from 0; model 0
 * is what Regard3D uses. A scene that does not form one connected
 * reconstruction can end up with no model 0 at all, which model_converter then
 * fails on - caught the same way any other step failing is, by its exit code.
 */
bool R3DTriangulationProcess::buildColmapCommandList(const R3DProjectPaths &paths)
{
	R3DExternalPrograms &progs = R3DExternalPrograms::getInstance();
	const wxString colmapExe(progs.getBestColmapPath());
	if(colmapExe.IsEmpty())
	{
		isOK_ = false;
		errorMessage_ = wxT("COLMAP was not found.\n\n")
			wxT("Please put colmap.exe into the subdirectory \"colmap_cuda\" and/or\n")
			wxT("\"colmap_nocuda\" of the external tools directory, or use a\n")
			wxT("different triangulation engine.");
		return false;
	}

	const wxString databasePath(quoted(wxString(paths.relativeColmapDatabaseFilename_.c_str(), wxConvLibc)));
	const wxString imagePath(quoted(wxString(paths.relativeImagePath_.c_str(), wxConvLibc)));

	wxFileName modelPathFN(wxString(paths.relativeColmapModelPath_.c_str(), wxConvLibc), wxEmptyString);
	if(!modelPathFN.DirExists())
#if wxCHECK_VERSION(2, 9, 0)
		wxFileName::Mkdir(modelPathFN.GetPath(wxPATH_GET_VOLUME), wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL);
#else
		wxFileName::Mkdir(modelPathFN.GetPath(wxPATH_GET_VOLUME), 0777, wxPATH_MKDIR_FULL);
#endif
	const wxString modelPath(quoted(modelPathFN.GetPath(wxPATH_GET_VOLUME)));

	wxFileName model0FN(modelPathFN);
	model0FN.AppendDir(wxT("0"));
	const wxString model0Path(quoted(model0FN.GetPath(wxPATH_GET_VOLUME)));

	wxFileName finalPlyFN(wxString(paths.relativeOutPath_.c_str(), wxConvLibc), wxT("FinalColorized.ply"));
	const wxString finalPlyPath(quoted(finalPlyFN.GetFullPath()));

	cmds_.Clear();
	progressTexts_.Clear();
	stepNames_.Clear();

	cmds_.Add(quoted(colmapExe) + wxT(" mapper")
		+ wxT(" --database_path ") + databasePath
		+ wxT(" --image_path ") + imagePath
		+ wxT(" --output_path ") + modelPath);
	progressTexts_.Add(wxT("Reconstructing scene (COLMAP)"));
	stepNames_.Add(wxT("colmap mapper"));

	cmds_.Add(quoted(colmapExe) + wxT(" model_converter")
		+ wxT(" --input_path ") + model0Path
		+ wxT(" --output_path ") + finalPlyPath
		+ wxT(" --output_type PLY"));
	progressTexts_.Add(wxT("Exporting model (COLMAP)"));
	stepNames_.Add(wxT("colmap model_converter"));

	return true;
}

void R3DTriangulationProcess::runSingleCommand()
{
	if(cmds_.IsEmpty())
		return;

	Redirect();	// Redirect I/O, hide console window

	wxString cmdLine = cmds_[0];
	cmds_.RemoveAt(0);
	wxString progressText = progressTexts_[0];
	progressTexts_.RemoveAt(0);
	currentStepName_ = stepNames_[0];
	stepNames_.RemoveAt(0);

	const float progress = (stepCount_ > 0
		? static_cast<float>(stepsDone_) / static_cast<float>(stepCount_) : 0.0f);
	pMainFrame_->sendUpdateProgressBarEvent(progress, progressText);

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
		isOK_ = false;
		errorMessage_ = wxString::Format(wxT("%s could not be started."),
			currentStepName_.c_str());
		cmds_.Clear();
		progressTexts_.Clear();
		stepNames_.Clear();
		finish();
	}
}

void R3DTriangulationProcess::readConsoleOutput()
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

void R3DTriangulationProcess::cancel()
{
	if(wasCancelled_ || processId_ <= 0)
		return;

	wasCancelled_ = true;

	// Whatever is still queued would run on data the killed tool never wrote
	cmds_.Clear();
	progressTexts_.Clear();
	stepNames_.Clear();

	// wxKILL_CHILDREN in case the tool started helpers of its own
	wxProcess::Kill(processId_, wxSIGKILL, wxKILL_CHILDREN);
}

void R3DTriangulationProcess::OnTerminate(int pid, int status)
{
	readConsoleOutput();	// Finish reading streams

	// This process is gone; cancel() must not kill a recycled pid
	processId_ = 0;

	const bool isColmap = (pTriangulation_ != NULL && pTriangulation_->computeEngine_ == 2);

	if(isColmap)
	{
		stepsDone_++;

		if(wasCancelled_)
		{
			isOK_ = false;
			errorMessage_ = wxT("Aborted.");
		}
		else if(status != 0)
		{
			isOK_ = false;
			errorMessage_ = wxString::Format(
				wxT("%s returned with error code %d.\n\nPlease check the console output for details."),
				currentStepName_.c_str(), status);
			cmds_.Clear();
			progressTexts_.Clear();
			stepNames_.Clear();
		}

		if(cmds_.IsEmpty())
		{
			if(isOK_)
			{
				wxFileName finalPlyFN(wxString(paths_.relativeOutPath_.c_str(), wxConvLibc), wxT("FinalColorized.ply"));
				finalPlyFN.MakeAbsolute(paths_.absoluteProjectPath_);
				if(!finalPlyFN.FileExists())
				{
					isOK_ = false;
					errorMessage_ = wxT("COLMAP wrote no reconstruction.\n\n")
						wxT("Please check the console output for details.");
				}
			}
			finish();
		}
		else
			runSingleCommand();

		return;
	}

	if(wasCancelled_)
	{
		// A partial sfm_data.bin may well exist, but it is not a reconstruction
		isOK_ = false;
		errorMessage_ = wxT("Aborted.");
	}
	else if(status != 0)
	{
		isOK_ = false;
		errorMessage_ = wxString::Format(
			wxT("openMVG_main_SfM returned with error code %d.\n\n")
			wxT("Please check the console output for details."), status);
	}
	else
	{
		// The tool reports success even when the scene stayed empty
		wxFileName sfmDataFN(wxString(paths_.relativeTriSfmDataFilename_.c_str(), wxConvLibc));
		sfmDataFN.MakeAbsolute(paths_.absoluteProjectPath_);
		if(!sfmDataFN.FileExists())
		{
			isOK_ = false;
			errorMessage_ = wxT("openMVG_main_SfM wrote no reconstruction.\n\n")
				wxT("Please check the console output for details.");
		}
	}

	finish();
}

void R3DTriangulationProcess::finish()
{
	if(pMainFrame_ != NULL)
		pMainFrame_->sendTriangulationProcessFinishedEvent();
}
