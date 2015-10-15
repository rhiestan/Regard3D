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
#include "minilog/minilog.h"

R3DDensificationProcess::R3DDensificationProcess(Regard3DMainFrame *pMainFrame)
	: wxProcess(pMainFrame), pMainFrame_(pMainFrame),
	processId_(0), checkForClusters_(false), numberOfClusters_(0)
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

	if(pDensification->densificationType_ == R3DProject::DTCMVSPMVS)
	{
		wxString pmvsPath(relativePMVSOutPath_);
		pmvsPath.Append(wxT("/"));

		wxString pmvsExe = R3DExternalPrograms::getInstance().getPMVSPath();
		wxString cmvsExe = R3DExternalPrograms::getInstance().getCMVSPath();
		wxString genOptionExe = R3DExternalPrograms::getInstance().getGenOptionPath();
		if(pDensification->useCMVS_)
		{
			cmds_.Add(cmvsExe + wxString(wxT(" ")) + pmvsPath
				+ wxString::Format(wxT(" %d %d"),
				pDensification->pmvsMaxClusterSize_, pDensification->pmvsNumThreads_));
			progressTexts_.Add(wxT("Clustering images (CMVS)"));
			cmds_.Add(genOptionExe + wxString(wxT(" ")) + pmvsPath
				+ wxString::Format(wxT(" %d %d %f %d %d %d"),
				pDensification->pmvsLevel_, pDensification->pmvsCSize_,
				pDensification->pmvsThreshold_, pDensification->pmvsWSize_,
				pDensification->pmvsMinImageNum_, pDensification->pmvsNumThreads_));
			progressTexts_.Add(wxT("Generating options"));

			pDensification->finalDenseModelName_ = wxT("option-0000.ply");

			// CMVS will run at a later stage, check whether more than one option-xxxx files exist
			checkForClusters_ = true;
		}
		else
		{
			wxString cmdLine = pmvsExe + wxString(wxT(" ")) + pmvsPath + wxString(wxT(" pmvs_options.txt"));
			cmds_.Add(cmdLine);
			progressTexts_.Add(wxT("Densify point cloud (PMVS)"));

			pDensification->finalDenseModelName_ = wxT("pmvs_options.txt.ply");
		}
	}
	else if(pDensification->densificationType_ == R3DProject::DTMVE)
	{
		wxString dmreconExe = R3DExternalPrograms::getInstance().getDMReconPath();
		wxString scene2psetExe = R3DExternalPrograms::getInstance().getScene2PsetPath();
		wxString mveSceneDir = wxString(paths.relativeMVESceneDir_.c_str(), wxConvLibc);
		wxString outputModelFilename = wxT("mve_model.ply");

		cmds_.Add(dmreconExe + wxString::Format(wxT(" --scale=%d --filter-width=%d --force "),
			pDensification->mveScale_, pDensification->mveFilterWidth_) + mveSceneDir);
		progressTexts_.Add(wxT("Densify point cloud (MVE)"));

		wxFileName outputModelFN(wxString(paths.relativeDensificationPath_.c_str(), wxConvLibc), outputModelFilename);
		cmds_.Add(scene2psetExe
			+ wxString::Format(wxT(" -ddepth-L%d -iundist-L%d -n -s -c "), pDensification->mveScale_, pDensification->mveScale_)
			+ mveSceneDir + wxT(" ") + outputModelFN.GetFullPath());
		progressTexts_.Add(wxT("Generating point cloud model (MVE)"));

		pDensification->finalDenseModelName_ = outputModelFilename;
	}

	runSingleCommand();

	return (processId_ > 0);
}

void R3DDensificationProcess::readConsoleOutput()
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

void R3DDensificationProcess::OnTerminate(int pid, int status)
{
	readConsoleOutput();	// Finish reading streams

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

void R3DDensificationProcess::runSingleCommand()
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
