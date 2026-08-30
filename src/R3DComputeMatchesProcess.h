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
#ifndef R3DCOMPUTEMATCHESPROCESS_H
#define R3DCOMPUTEMATCHESPROCESS_H

class Regard3DMainFrame;

#include <wx/process.h>

#include "R3DProject.h"

/**
 * Computes features and matches by running the OpenMVG command line tools.
 *
 * This is the counterpart of R3DComputeMatchesThread, which does the same work
 * with the OpenMVG library linked into Regard3D. Which of the two runs is
 * decided by R3DProject::ComputeMatches::computeEngine_.
 *
 * One Regard3D step is several executables here, run one after the other:
 *
 *   openMVG_main_ComputeFeatures[_OpenCV]  -> .feat/.desc, image_describer.json
 *   openMVG_main_PairGenerator             -> pairs.bin
 *   openMVG_main_ComputeMatches            -> matches.putative.txt
 *   openMVG_main_GeometricFilter           -> matches.{f,e,h}.txt, once per model
 *
 * They are queued in cmds_ and started one at a time from OnTerminate, the same
 * way R3DDensificationProcess chains CMVS and PMVS.
 *
 * The caller must make sure the matches directory and sfm_data.bin exist before
 * calling runComputeMatchesProcess; that is slow enough to want a thread, and
 * wxExecute has to be called from the main thread.
 */
class R3DComputeMatchesProcess : public wxProcess
{
public:
	R3DComputeMatchesProcess(Regard3DMainFrame *pMainFrame);
	virtual ~R3DComputeMatchesProcess();

	bool runComputeMatchesProcess(R3DProject::ComputeMatches *pComputeMatches);
	void readConsoleOutput();

	R3DProject::ComputeMatches *getComputeMatches() const { return pComputeMatches_; }
	wxString getRuntimeStr();

	// Results, read by Regard3DMainFrame::OnComputeMatchesFinished
	bool getIsOK() const { return isOK_; }
	const wxString &getErrorMessage() const { return errorMessage_; }
	const wxArrayString &getResultStrings() const { return resultStrings_; }

protected:
	virtual void OnTerminate(int pid, int status);

	void runSingleCommand();
	bool buildCommandList(const R3DProjectPaths &paths);
	void finish();
	void collectKeypointCounts(std::vector<int> &numberOfKeypoints);

private:
	Regard3DMainFrame *pMainFrame_;
	R3DProject::ComputeMatches *pComputeMatches_;
	R3DProjectPaths paths_;
	wxDateTime beginTime_;

	int processId_;
#if wxCHECK_VERSION(2, 9, 2)
	wxExecuteEnv env_;
#endif
	wxArrayString cmds_, progressTexts_;
	// Names of the executables, used to say which step failed
	wxArrayString stepNames_;
	wxString currentStepName_;
	int stepCount_, stepsDone_;

	// Files the run has to produce, checked once everything has terminated
	wxArrayString expectedOutputs_;

	bool isOK_;
	wxString errorMessage_;
	wxArrayString resultStrings_;
};

#endif
