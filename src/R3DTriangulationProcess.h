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
#ifndef R3DTRIANGULATIONPROCESS_H
#define R3DTRIANGULATIONPROCESS_H

class Regard3DMainFrame;

#include <wx/process.h>

#include "R3DProject.h"

/**
 * Reconstructs the scene by running openMVG_main_SfM.
 *
 * This is the counterpart of R3DTriangulationThread, which does the same work
 * with the OpenMVG library linked into Regard3D. Which of the two runs is
 * decided by R3DProject::Triangulation::computeEngine_.
 *
 * It is a single command, but it is built and run the same way as the other
 * external steps so that the console output and the error handling behave
 * alike. openMVG_main_SfM writes sfm_data.bin, cloud_and_poses.ply and
 * SfMReconstruction_Report.html into the triangulation's out directory; the
 * colorized model Regard3D displays is produced afterwards, by
 * R3DTriangulationThread::finishExternalTriangulation.
 *
 * When R3DProject::Triangulation::computeEngine_ is 2 (COLMAP), this instead
 * chains colmap mapper (reads the database a COLMAP ComputeMatches wrote,
 * writes a sparse model) and colmap model_converter (turns that model into
 * FinalColorized.ply directly - COLMAP extracts point colors itself, so there
 * is no separate colorization step the way there is for OpenMVG). Regard3DMainFrame
 * finishes a COLMAP run right here, without R3DTriangulationThread.
 */
class R3DTriangulationProcess : public wxProcess
{
public:
	R3DTriangulationProcess(Regard3DMainFrame *pMainFrame);
	virtual ~R3DTriangulationProcess();

	bool runTriangulationProcess(R3DProject::Triangulation *pTriangulation);
	void readConsoleOutput();

	R3DProject::Triangulation *getTriangulation() const { return pTriangulation_; }
	wxTimeSpan getRunTime() const { return wxDateTime::UNow() - beginTime_; }

	/**
	 * Kills the executable that is running at the moment.
	 *
	 * The queue is emptied first, so OnTerminate reports the step as finished
	 * instead of starting the next command. Called from the progress dialog.
	 */
	void cancel();
	bool getWasCancelled() const { return wasCancelled_; }

	bool getIsOK() const { return isOK_; }
	const wxString &getErrorMessage() const { return errorMessage_; }

protected:
	virtual void OnTerminate(int pid, int status);

	bool buildCommand(const R3DProjectPaths &paths);
	bool buildColmapCommandList(const R3DProjectPaths &paths);
	void runSingleCommand();
	void finish();

private:
	Regard3DMainFrame *pMainFrame_;
	R3DProject::Triangulation *pTriangulation_;
	R3DProjectPaths paths_;
	wxDateTime beginTime_;

	int processId_;
#if wxCHECK_VERSION(2, 9, 2)
	wxExecuteEnv env_;
#endif
	// openMVG_main_SfM: a single command, run directly
	wxString cmd_;
	// COLMAP: mapper + model_converter, chained the way
	// R3DComputeMatchesProcess chains its steps. Empty for the OpenMVG engine.
	wxArrayString cmds_, progressTexts_;
	wxArrayString stepNames_;
	wxString currentStepName_;
	int stepCount_, stepsDone_;

	bool isOK_, wasCancelled_;
	wxString errorMessage_;
};

#endif
