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
#ifndef R3DSURFACEGENPROCESS_H
#define R3DSURFACEGENPROCESS_H

class Regard3DMainFrame;

#include <wx/process.h>

#include "R3DProject.h"


class R3DSurfaceGenProcess: public wxProcess
{
public:
	R3DSurfaceGenProcess(Regard3DMainFrame *pMainFrame);
	virtual ~R3DSurfaceGenProcess();

	bool runSurfaceGenProcess(R3DProject::Surface *pSurface);
	void readConsoleOutput();
	R3DProject::Surface *getSurface() const { return pSurface_; }
	wxString getRuntimeStr();

	/**
	 * Kills the executable that is running at the moment.
	 *
	 * The queue is emptied first, so OnTerminate reports the step as finished
	 * instead of starting the next command. Called from the progress dialog.
	 */
	void cancel();
	bool getWasCancelled() const { return wasCancelled_; }

	// Results, read by Regard3DMainFrame::OnSurfaceGenFinished
	bool getIsOK() const { return isOK_; }
	const wxString &getErrorMessage() const { return errorMessage_; }

protected:
	virtual void OnTerminate(int pid, int status);

	void runSingleCommand();
	/**
	 * Queues a command.
	 *
	 * stepName names the executable in error messages; requiredOutput, if not
	 * empty, is a file (relative to the project) the step has to have written,
	 * for tools that can exit with 0 without having produced anything.
	 */
	void addCommand(const wxString &cmd, const wxString &progressText,
		const wxString &stepName, const wxString &requiredOutput = wxEmptyString);
	void clearCommands();

private:
	Regard3DMainFrame *pMainFrame_;
	R3DProject::Surface *pSurface_;
	wxDateTime beginTime_;

	int processId_;
#if wxCHECK_VERSION(2, 9, 2)
	wxExecuteEnv env_;
#endif
	wxArrayString cmds_, progressTexts_;
	wxArrayString stepNames_, requiredOutputs_;
	wxString currentStepName_, currentRequiredOutput_;

	bool wasCancelled_;
	bool isOK_;
	wxString errorMessage_;

	// The OpenMVS tools leave their log files in the project directory, see
	// R3DExternalPrograms::collectOpenMVSLog
	wxString absoluteProjectPath_, relativeSurfacePath_;
};

#endif
