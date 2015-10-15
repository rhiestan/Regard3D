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
#include "R3DSurfaceGenProcess.h"
#include "Regard3DMainFrame.h"
#include "R3DExternalPrograms.h"
#include "minilog/minilog.h"

R3DSurfaceGenProcess::R3DSurfaceGenProcess(Regard3DMainFrame *pMainFrame)
	: wxProcess(pMainFrame), pMainFrame_(pMainFrame),
	processId_(0)
{
}

R3DSurfaceGenProcess::~R3DSurfaceGenProcess()
{
}

bool R3DSurfaceGenProcess::runSurfaceGenProcess(R3DProject::Surface *pSurface)
{
	R3DProject *pProject = R3DProject::getInstance();
	R3DProjectPaths paths;
	if(!pProject->getProjectPathsSrf(paths, pSurface))
		return false;
	pSurface_ = pSurface;

	beginTime_ = wxDateTime::UNow();


#if wxCHECK_VERSION(2, 9, 0)
	env_.cwd = paths.absoluteProjectPath_;

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

	cmds_.Clear();
	progressTexts_.Clear();

	wxString relativeSurfacePath(paths.relativeSurfacePath_.c_str(), wxConvLibc);
	wxString relativeSurfaceFilename;		// Used for texturing

	if(pSurface->surfaceType_ == R3DProject::STPoissonRecon)
	{
		wxString poissonReconExe = R3DExternalPrograms::getInstance().getPoissonReconPath();
		wxString surfaceTrimmerExe = R3DExternalPrograms::getInstance().getSurfaceTrimmerPath();
		wxString texreconExe = R3DExternalPrograms::getInstance().getTexReconPath();

		wxString poissonReconCmd = poissonReconExe + wxT(" --in ") + wxString(paths.relativeDenseModelName_.c_str(), wxConvLibc)
			+ wxT(" --out ") + relativeSurfacePath + wxT("/model_surface.ply")
			+ wxString::Format(wxT(" --depth %d --pointWeight %g --samplesPerNode %g"), 
			pSurface->poissonDepth_, pSurface->poissonPointWeight_, pSurface->poissonSamplesPerNode_);
		if(pSurface->poissonTrimThreshold_ > 0)
			poissonReconCmd.Append(wxT(" --density"));
		poissonReconCmd.Append(wxT(" --verbose"));
		cmds_.Add(poissonReconCmd);
		progressTexts_.Add(wxT("Generating surface (PoissonRecon)"));
		wxString surfaceFilename(wxT("model_surface.ply"));

		if(pSurface->poissonTrimThreshold_ > 0)
		{
			cmds_.Add(surfaceTrimmerExe + wxT(" --in ") + relativeSurfacePath + wxT("/model_surface.ply ") 
				+ wxString::Format(wxT("--trim %g"), pSurface->poissonTrimThreshold_)
				+ wxT(" --out ") + relativeSurfacePath + wxT("/model_surface_trim.ply"));
			progressTexts_.Add(wxT("Trimming surface"));
			surfaceFilename = wxT("model_surface_trim.ply");
		}

		// Store for later, will either be overwritten (texture case) or used as input (colorizing vertices case)
		pSurface->finalSurfaceFilename_ = surfaceFilename;

		if(pSurface->colorizationType_ == R3DProject::CTTextures)
		{
			wxFileName surfaceFN(relativeSurfacePath, surfaceFilename);
			relativeSurfaceFilename = surfaceFN.GetFullPath();

			pSurface->finalSurfaceFilename_ = wxT("model_surface_text.obj");
		}
	}
	else if(pSurface->surfaceType_ == R3DProject::STFSSRecon)
	{
		wxString fssReconExe = R3DExternalPrograms::getInstance().getFSSReconPath();
		wxString meshcleanExe = R3DExternalPrograms::getInstance().getMeshCleanPath();

		wxString surfaceModelName(wxT("surface_model.ply"));
		wxString cleanedSurfaceModelName(wxT("surface_clean_model.ply"));
		wxFileName surfaceModelFN(wxString(paths.relativeSurfacePath_.c_str(), wxConvLibc), surfaceModelName);
		wxFileName cleanedSurfaceModelFN(wxString(paths.relativeSurfacePath_.c_str(), wxConvLibc), cleanedSurfaceModelName);

		cmds_.Add(fssReconExe + wxString::Format(wxT(" --scale-factor=%g --refine-octree=%d "),
			pSurface->fssrScaleFactorMultiplier_, pSurface->fssrRefineOctreeLevels_)
			+ wxString(paths.relativeDenseModelName_.c_str(), wxConvLibc)
			+ wxT(" ")
			+ surfaceModelFN.GetFullPath());
		progressTexts_.Add(wxT("Generating surface (FSSR)"));

		cmds_.Add(meshcleanExe + wxString::Format(wxT(" --threshold=%g --component-size=%d "),
			pSurface->fssrConfidenceThreshold_, pSurface->fssrMinComponentSize_)
			+ surfaceModelFN.GetFullPath()
			+ wxT(" ")
			+ cleanedSurfaceModelFN.GetFullPath());
		progressTexts_.Add(wxT("Cleaning mesh (FSSR)"));

		relativeSurfaceFilename = cleanedSurfaceModelFN.GetFullPath();

		// Used only in case of colored vertices
		pSurface->finalSurfaceFilename_ = cleanedSurfaceModelName;
	}

	if(pSurface->colorizationType_ == R3DProject::CTTextures)
	{
		wxString texreconExe = R3DExternalPrograms::getInstance().getTexReconPath();

		wxString texreconCmd(texreconExe + wxT(" "));
		if(pSurface->textGeometricVisibilityTest_ == false)
			texreconCmd += wxT("--skip_geometric_visibility_test ");
		if(pSurface->textGlobalSeamLeveling_ == false)
			texreconCmd += wxT("--skip_global_seam_leveling ");
		if(pSurface->textLocalSeamLeveling_ == false)
			texreconCmd += wxT("--skip_local_seam_leveling ");
		if(pSurface->textOutlierRemovalType_ == 1)
			texreconCmd += wxT("--outlier_removal=gauss_clamping ");
		if(pSurface->textOutlierRemovalType_ == 2)
			texreconCmd += wxT("--outlier_removal=gauss_damping ");
		texreconCmd += wxT("--no_intermediate_results ");
		texreconCmd += wxString(paths.relativeMVESceneDir_.c_str(), wxConvLibc)
			+ wxT("::undistorted ")
			+ relativeSurfaceFilename + wxT(" ")
			+ relativeSurfacePath + wxT("/model_surface_text");

		cmds_.Add(texreconCmd);
		progressTexts_.Add(wxT("Generating texture (texrecon)"));

		pSurface->finalSurfaceFilename_ = wxT("model_surface_text.obj");
	}
	else if(pSurface->colorizationType_ == R3DProject::CTColoredVertices)
	{
		// No external program, this case is handled in Regard3DMainFrame::OnSurfaceGenFinished
	}

	runSingleCommand();

	return (processId_ > 0);
}

void R3DSurfaceGenProcess::readConsoleOutput()
{
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

		MLOG << buf.c_str();
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

		MLOG << buf.c_str();
	}
}

wxString R3DSurfaceGenProcess::getRuntimeStr()
{
	wxTimeSpan runTime = wxDateTime::UNow() - beginTime_;
	wxString runTimeStr;
	if(runTime.GetHours() > 0)
		runTimeStr = runTime.Format(wxT("%H:%M:%S.%l"));
	else
		runTimeStr = runTime.Format(wxT("%M:%S.%l"));

	return runTimeStr;
}

void R3DSurfaceGenProcess::OnTerminate(int pid, int status)
{
	readConsoleOutput();	// Finish reading streams

	if(cmds_.IsEmpty())
		pMainFrame_->sendSurfaceGenFinishedEvent();
	else
		runSingleCommand();
}

void R3DSurfaceGenProcess::runSingleCommand()
{
	if(!cmds_.IsEmpty())
	{
		Redirect();	// Redirect I/O, hide console window

		wxString cmdLine = cmds_[0];
		cmds_.RemoveAt(0);
		wxString progressText = progressTexts_[0];
		progressTexts_.RemoveAt(0);
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
	}
}
