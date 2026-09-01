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
	processId_(0), isOK_(true), wasCancelled_(false)
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

	// wxKILL_CHILDREN in case the tool started helpers of its own
	wxProcess::Kill(processId_, wxSIGKILL, wxKILL_CHILDREN);
}

void R3DTriangulationProcess::OnTerminate(int pid, int status)
{
	readConsoleOutput();	// Finish reading streams

	// This process is gone; cancel() must not kill a recycled pid
	processId_ = 0;

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
