/**
 * Copyright (C) 2026 Roman Hiestand
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
#ifndef R3DEXPORTPROCESS_H
#define R3DEXPORTPROCESS_H

class Regard3DMainFrame;

#include <wx/process.h>

#include "R3DProject.h"

/**
 * Runs one of the OpenMVG export tools.
 *
 * The exports are all the same shape: one executable, an SfM_Data file in and
 * a directory or a file out, nothing to do afterwards. So unlike the steps of
 * the pipeline this needs no command queue, only somewhere to run the command
 * and somewhere to say how it went.
 *
 * The working directory is the project directory, so the paths of
 * R3DProjectPaths can be passed as they are.
 */
class R3DExportProcess : public wxProcess
{
public:
	R3DExportProcess(Regard3DMainFrame *pMainFrame);
	virtual ~R3DExportProcess();

	/**
	 * @param name what the export is called in the progress dialog and in an
	 *        error message, for example "Colmap"
	 * @param command the whole command line, quoted where it has to be
	 * @param expectedOutput a file the tool has to have written, checked when
	 *        it has terminated; empty if there is nothing sensible to check
	 */
	bool runExportProcess(const R3DProjectPaths &paths, const wxString &name,
		const wxString &command, const wxString &expectedOutput);
	void readConsoleOutput();

	/**
	 * Kills the running tool. The step is reported as failed afterwards.
	 */
	void cancel();
	bool getWasCancelled() const { return wasCancelled_; }

	const wxString &getName() const { return name_; }
	bool getIsOK() const { return isOK_; }
	const wxString &getErrorMessage() const { return errorMessage_; }

protected:
	virtual void OnTerminate(int pid, int status);

	void finish();

private:
	Regard3DMainFrame *pMainFrame_;

	int processId_;
#if wxCHECK_VERSION(2, 9, 2)
	wxExecuteEnv env_;
#endif
	wxString name_, command_, expectedOutput_;

	bool isOK_, wasCancelled_;
	wxString errorMessage_;
};

#endif
