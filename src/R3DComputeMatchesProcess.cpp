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
#include "R3DComputeMatchesProcess.h"
#include "R3DOpenMVGOptions.h"
#include "Regard3DMainFrame.h"
#include "R3DExternalPrograms.h"

#include <algorithm>
#include <fstream>
#include <iostream>

namespace
{
	// Project paths are user chosen and regularly contain spaces
	wxString quoted(const wxString &str)
	{
		return wxT("\"") + str + wxT("\"");
	}

	wxString inMatchesDir(const wxString &matchesDir, const wxString &filename)
	{
		wxFileName fn(matchesDir, filename);
		return quoted(fn.GetFullPath());
	}

	// Counts the features in an OpenMVG .feat file, which holds one feature per
	// line (see saveFeatsToFile in openMVG/features/feature.hpp). Returns -1 if
	// the file cannot be read.
	int countFeatureLines(const wxString &filename)
	{
		std::ifstream stream(filename.mb_str(wxConvLibc), std::ios::in | std::ios::binary);
		if(!stream.is_open())
			return -1;

		std::vector<char> buffer(64 * 1024);
		int lines = 0;
		bool lastCharWasNewline = true;
		while(stream)
		{
			stream.read(&buffer[0], static_cast<std::streamsize>(buffer.size()));
			const std::streamsize count = stream.gcount();
			if(count <= 0)
				break;
			lines += static_cast<int>(std::count(buffer.begin(), buffer.begin() + count, '\n'));
			lastCharWasNewline = (buffer[static_cast<size_t>(count) - 1] == '\n');
		}

		// A trailing feature without a final newline still is a feature
		if(!lastCharWasNewline)
			lines++;

		return lines;
	}
}

R3DComputeMatchesProcess::R3DComputeMatchesProcess(Regard3DMainFrame *pMainFrame)
	: wxProcess(pMainFrame), pMainFrame_(pMainFrame), pComputeMatches_(NULL),
	processId_(0), stepCount_(0), stepsDone_(0), isOK_(true)
{
}

R3DComputeMatchesProcess::~R3DComputeMatchesProcess()
{
}

bool R3DComputeMatchesProcess::runComputeMatchesProcess(R3DProject::ComputeMatches *pComputeMatches)
{
	R3DProject *pProject = R3DProject::getInstance();
	pComputeMatches_ = pComputeMatches;

	beginTime_ = wxDateTime::UNow();

	if(!pProject->getProjectPathsCM(paths_, pComputeMatches))
	{
		isOK_ = false;
		errorMessage_ = wxT("Could not determine the project paths for computing matches.");
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

	if(!buildCommandList(paths_))
	{
		finish();
		return false;
	}

	stepCount_ = static_cast<int>(cmds_.GetCount());
	stepsDone_ = 0;

	runSingleCommand();

	return (processId_ > 0);
}

/**
 * Turns the stored parameters into the list of commands to run.
 *
 * Every step is forced (-f 1): the matches directory has just been emptied by
 * R3DProject::prepareComputeMatches, and a leftover file from an earlier run
 * with another engine would silently be reused otherwise.
 */
bool R3DComputeMatchesProcess::buildCommandList(const R3DProjectPaths &paths)
{
	const R3DOpenMVGMatchingParams &params = pComputeMatches_->openMVGParams_;
	R3DExternalPrograms &progs = R3DExternalPrograms::getInstance();

	const wxString matchesDir(paths.relativeMatchesPath_.c_str(), wxConvLibc);
	const wxString sfmData(paths.matchesSfmDataFilename_.c_str(), wxConvLibc);
	const wxString quotedSfmData(quoted(sfmData));
	const wxString quotedMatchesDir(quoted(matchesDir));
	// openMVG_main_PairGenerator always writes a text file, one pair per line,
	// whatever the extension says (see savePairs in Pair_Builder.hpp)
	const wxString pairsFile(inMatchesDir(matchesDir, wxT("pairs.txt")));
	const wxString putativeMatches(inMatchesDir(matchesDir, wxT("matches.putative.txt")));

	cmds_.Clear();
	progressTexts_.Clear();
	stepNames_.Clear();
	expectedOutputs_.Clear();

	// --- Features ------------------------------------------------------------
	const bool useOpenCV = R3DOpenMVGOptions::describerIsOpenCV(params.describerMethod_);
	const wxString computeFeaturesExe(useOpenCV
		? progs.getComputeFeaturesOpenCVPath() : progs.getComputeFeaturesPath());
	const wxString pairGeneratorExe(progs.getPairGeneratorPath());
	const wxString computeMatchesExe(progs.getComputeMatchesPath());
	const wxString geometricFilterExe(progs.getGeometricFilterPath());

	if(computeFeaturesExe.IsEmpty() || pairGeneratorExe.IsEmpty()
		|| computeMatchesExe.IsEmpty() || geometricFilterExe.IsEmpty())
	{
		isOK_ = false;
		errorMessage_ = wxT("The OpenMVG executables were not found.\n\n")
			wxT("Please put openMVG_main_ComputeFeatures, openMVG_main_PairGenerator,\n")
			wxT("openMVG_main_ComputeMatches and openMVG_main_GeometricFilter into the\n")
			wxT("subdirectory \"openmvg\" of the external tools directory.");
		return false;
	}

	wxString cmd(quoted(computeFeaturesExe)
		+ wxT(" -i ") + quotedSfmData
		+ wxT(" -o ") + quotedMatchesDir
		+ wxT(" -f 1 -m ") + R3DOpenMVGOptions::describerName(params.describerMethod_));
	if(!useOpenCV)
	{
		// openMVG_main_ComputeFeatures_OpenCV only understands -i, -o, -f and -m
		cmd.Append(wxT(" -p "));
		cmd.Append(R3DOpenMVGOptions::presetName(params.describerPreset_));
		cmd.Append(params.upright_ ? wxT(" -u 1") : wxT(" -u 0"));
		if(params.numThreads_ > 0)
			cmd.Append(wxString::Format(wxT(" -n %d"), params.numThreads_));
	}
	cmds_.Add(cmd);
	progressTexts_.Add(wxT("Computing features (OpenMVG)"));
	stepNames_.Add(wxT("openMVG_main_ComputeFeatures"));

	// --- Pairs ---------------------------------------------------------------
	cmd = quoted(pairGeneratorExe)
		+ wxT(" -i ") + quotedSfmData
		+ wxT(" -o ") + pairsFile
		+ wxT(" -m ") + R3DOpenMVGOptions::pairModeName(params.pairMode_);
	if(params.pairMode_ != 0)
		cmd.Append(wxString::Format(wxT(" -c %d"), params.contiguousCount_));
	cmds_.Add(cmd);
	progressTexts_.Add(wxT("Generating image pairs (OpenMVG)"));
	stepNames_.Add(wxT("openMVG_main_PairGenerator"));

	// --- Putative matches ----------------------------------------------------
	cmd = quoted(computeMatchesExe)
		+ wxT(" -i ") + quotedSfmData
		+ wxT(" -o ") + putativeMatches
		+ wxT(" -p ") + pairsFile
		+ wxT(" -f 1")
		// FromCDouble, not Format: the tools read this with the C locale, so a
		// comma as the decimal separator would silently be parsed as 0
		+ wxT(" -r ") + wxString::FromCDouble(params.distanceRatio_, 4)
		+ wxT(" -n ") + R3DOpenMVGOptions::matcherName(params.nearestMatchingMethod_);
	if(params.cacheSize_ > 0)
		cmd.Append(wxString::Format(wxT(" -c %d"), params.cacheSize_));
	if(params.preemptiveFeatureCount_ > 0)
		cmd.Append(wxString::Format(wxT(" -P %d"), params.preemptiveFeatureCount_));
	cmds_.Add(cmd);
	progressTexts_.Add(wxT("Computing putative matches (OpenMVG)"));
	stepNames_.Add(wxT("openMVG_main_ComputeMatches"));

	// --- Geometric filtering, one run per selected model ----------------------
	// The filenames are the ones the rest of Regard3D reads: the incremental
	// triangulations use matches.f.txt, the global one matches.e.txt.
	const struct { bool selected_; const wxChar *model_; const wxChar *file_; const wxChar *text_; } models[] = {
		{ params.computeFundamental_, wxT("f"), wxT("matches.f.txt"), wxT("Geometric filtering, fundamental matrix (OpenMVG)") },
		{ params.computeEssential_,   wxT("e"), wxT("matches.e.txt"), wxT("Geometric filtering, essential matrix (OpenMVG)") },
		{ params.computeHomography_,  wxT("h"), wxT("matches.h.txt"), wxT("Geometric filtering, homography matrix (OpenMVG)") }
	};

	for(size_t i = 0; i < WXSIZEOF(models); i++)
	{
		if(!models[i].selected_)
			continue;

		const wxString outputFile(inMatchesDir(matchesDir, models[i].file_));
		cmd = quoted(geometricFilterExe)
			+ wxT(" -i ") + quotedSfmData
			+ wxT(" -m ") + putativeMatches
			+ wxT(" -o ") + outputFile
			+ wxT(" -f 1 -g ") + models[i].model_;
		if(params.guidedMatching_)
			cmd.Append(wxT(" -r 1"));
		if(params.cacheSize_ > 0)
			cmd.Append(wxString::Format(wxT(" -c %d"), params.cacheSize_));
		cmds_.Add(cmd);
		progressTexts_.Add(models[i].text_);
		stepNames_.Add(wxT("openMVG_main_GeometricFilter"));

		wxFileName outputFN(paths.absoluteMatchesPath_, models[i].file_);
		expectedOutputs_.Add(outputFN.GetFullPath());
	}

	return true;
}

void R3DComputeMatchesProcess::readConsoleOutput()
{
	// The output of the tools is forwarded to std::cout/std::cerr, where the
	// console output window picks it up the same way as the OpenMVG library's
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

wxString R3DComputeMatchesProcess::getRuntimeStr()
{
	wxTimeSpan runTime = wxDateTime::UNow() - beginTime_;
	wxString runTimeStr;
	if(runTime.GetHours() > 0)
		runTimeStr = runTime.Format(wxT("%H:%M:%S.%l"));
	else
		runTimeStr = runTime.Format(wxT("%M:%S.%l"));

	return runTimeStr;
}

void R3DComputeMatchesProcess::OnTerminate(int pid, int status)
{
	readConsoleOutput();	// Finish reading streams

	stepsDone_++;

	if(status != 0)
	{
		// All these tools return EXIT_FAILURE on any error, so stop here instead
		// of running the remaining steps on data that was never written
		isOK_ = false;
		errorMessage_ = wxString::Format(
			wxT("%s returned with error code %d.\n\nPlease check the console output for details."),
			currentStepName_.c_str(), status);
		cmds_.Clear();
		progressTexts_.Clear();
		stepNames_.Clear();
	}

	if(cmds_.IsEmpty())
		finish();
	else
		runSingleCommand();
}

/**
 * Collects the statistics and tells the main frame that we are done.
 */
void R3DComputeMatchesProcess::finish()
{
	if(isOK_)
	{
		// A tool can report success and still have written nothing useful,
		// so check that what the next steps need is really there
		for(size_t i = 0; i < expectedOutputs_.GetCount(); i++)
		{
			if(!wxFileName::FileExists(expectedOutputs_[i]))
			{
				isOK_ = false;
				errorMessage_ = wxString::Format(
					wxT("The matches file\n%s\nwas not created.\n\nPlease check the console output for details."),
					expectedOutputs_[i].c_str());
				break;
			}
		}
	}

	resultStrings_.Clear();

	if(isOK_ && pComputeMatches_ != NULL)
	{
		std::vector<int> numberOfKeypoints;
		collectKeypointCounts(numberOfKeypoints);

		if(!numberOfKeypoints.empty())
		{
			std::vector<int> sorted(numberOfKeypoints);
			std::sort(sorted.begin(), sorted.end());

			long long sum = 0;
			for(size_t i = 0; i < sorted.size(); i++)
				sum += sorted[i];

			resultStrings_.Add(wxT("Keypoints per image (min/max/avg/median)"));
			resultStrings_.Add(wxString::Format(wxT("%d/%d/%d/%d"),
				sorted.front(), sorted.back(),
				static_cast<int>(sum / static_cast<long long>(sorted.size())),
				sorted[sorted.size() / 2]));
		}

		pComputeMatches_->numberOfKeypoints_ = numberOfKeypoints;
	}

	const wxString runTimeStr(getRuntimeStr());
	if(isOK_)
	{
		resultStrings_.Add(wxT("Elapsed time"));
		resultStrings_.Add(runTimeStr);
	}
	if(pComputeMatches_ != NULL)
		pComputeMatches_->runningTime_ = runTimeStr;

	if(pMainFrame_ != NULL)
		pMainFrame_->sendComputeMatchesFinishedEvent();
}

/**
 * Reads back how many features each image ended up with.
 *
 * The library engine gets these numbers from the extractor itself; here they
 * only exist on disk, as one .feat file per image.
 */
void R3DComputeMatchesProcess::collectKeypointCounts(std::vector<int> &numberOfKeypoints)
{
	numberOfKeypoints.clear();

	R3DProject *pProject = R3DProject::getInstance();
	R3DProject::Object *pObject = pProject->getObjectByTypeAndID(
		R3DProject::R3DTreeItem::TypePictureSet, paths_.pictureSetId_);
	R3DProject::PictureSet *pPictureSet = dynamic_cast<R3DProject::PictureSet *>(pObject);
	if(pPictureSet == NULL)
		return;

	const ImageInfoVector &iiv = pPictureSet->getImageInfoVector();
	for(size_t i = 0; i < iiv.size(); i++)
	{
		wxFileName featFN(paths_.absoluteMatchesPath_, iiv[i].importedFilename_);
		featFN.SetExt(wxT("feat"));

		const int count = countFeatureLines(featFN.GetFullPath());
		if(count >= 0)
			numberOfKeypoints.push_back(count);
	}
}

void R3DComputeMatchesProcess::runSingleCommand()
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

	// One step is one command, there is no progress within a command
	const float progress = (stepCount_ > 0
		? static_cast<float>(stepsDone_) / static_cast<float>(stepCount_) : 0.0f);
	pMainFrame_->sendUpdateProgressBarEvent(progress, progressText);

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
		errorMessage_ = wxString::Format(wxT("%s could not be started."),
			currentStepName_.c_str());
		cmds_.Clear();
		progressTexts_.Clear();
		stepNames_.Clear();
		finish();
	}
}
