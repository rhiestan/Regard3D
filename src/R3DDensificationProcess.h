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
#ifndef R3DDENSIFICATIONPROCESS_H
#define R3DDENSIFICATIONPROCESS_H

class Regard3DMainFrame;

#include <wx/process.h>

#include "R3DProject.h"

class R3DDensificationProcess : public wxProcess
{
public:
	R3DDensificationProcess(Regard3DMainFrame *pMainFrame);
	virtual ~R3DDensificationProcess();

	bool runDensificationProcess(R3DProject::Densification *pDensification);
	void readConsoleOutput();
	R3DProject::Densification *getDensification() const { return pDensification_; }
	wxString getRuntimeStr();
	int getNumberOfClusters() const { return numberOfClusters_; }

	/**
	 * Kills the executable that is running at the moment.
	 *
	 * The queue is emptied first, so OnTerminate reports the step as finished
	 * instead of starting the next command. Called from the progress dialog.
	 */
	void cancel();
	bool getWasCancelled() const { return wasCancelled_; }

	// Results, read by Regard3DMainFrame::OnDensificationFinished
	bool getIsOK() const { return isOK_; }
	const wxString &getErrorMessage() const { return errorMessage_; }

protected:
	virtual void OnTerminate(int pid, int status);

	void runSingleCommand();
	bool writePMVSOptions();
	bool fixColmapPoints3DFile();

private:
	Regard3DMainFrame *pMainFrame_;
	R3DProject::Densification *pDensification_;
	wxDateTime beginTime_;

	int processId_;
#if wxCHECK_VERSION(2, 9, 2)
	wxExecuteEnv env_;
#endif
	wxArrayString cmds_, progressTexts_;
	// Name of the executable behind each queued command, used to say which
	// step failed; currentStepName_ is the one belonging to the command
	// that is running (or just finished) at any given moment
	wxArrayString stepNames_;
	wxString currentStepName_;

	bool checkForClusters_, wasCancelled_;
	bool isOK_;
	wxString errorMessage_;
	// The parameters of the dialog have to go into the pmvs_options.txt that
	// openMVG_main_openMVG2PMVS wrote, once it has run
	bool writePMVSOptions_;
	wxString relativePMVSOutPath_;
	int numberOfClusters_;

	// openMVG_main_openMVG2Colmap writes points3D.txt with each observation's
	// raw feature index instead of the position Colmap expects within the
	// image's own POINTS2D[] list (see fixColmapPoints3DFile()); patched once
	// the export, the first command of the queue, has run
	bool fixColmapSparseModel_;
	wxString relativeColmapSparsePath_;
};

#endif
