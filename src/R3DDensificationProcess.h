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

protected:
	virtual void OnTerminate(int pid, int status);

	void runSingleCommand();

private:
	Regard3DMainFrame *pMainFrame_;
	R3DProject::Densification *pDensification_;
	wxDateTime beginTime_;

	int processId_;
#if wxCHECK_VERSION(2, 9, 2)
	wxExecuteEnv env_;
#endif
	wxArrayString cmds_, progressTexts_;

	bool checkForClusters_;
	wxString relativePMVSOutPath_;
	int numberOfClusters_;
};

#endif
