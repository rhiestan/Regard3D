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

#ifndef R3DFEATURESTHREAD_H
#define R3DFEATURESTHREAD_H

#include <string>
#include <vector>
#include <wx/thread.h>
#include <wx/arrstr.h>
#include "Regard3DFeatures.h"

class R3DFeaturesThread : public wxThread
{
public:

	static bool extractFeaturesAndDescriptors(
		const std::vector<std::string> &vec_fileNames, // input filenames
		const std::string &sOutDir,  // Output directory where features and descriptor will be saved
		const Regard3DFeatures::R3DFParams &params);

	static const std::vector<int> &getNumberOfKeypoints() { return numberOfKeypoints_; }

protected:
	R3DFeaturesThread();
	virtual ~R3DFeaturesThread();

	virtual wxThread::ExitCode Entry();

	wxString getNewWorkItem();
	void processWorkItem(wxString &workItem);

	void sendMsgToMainFrame(const wxString &workItem, size_t nrOfKeypoints);

private:

	class R3DFeaturesThreadTasks
	{
	public:
		wxMutex mutex_;
		wxArrayString fileNames_;
		wxString outDir_;
		Regard3DFeatures::R3DFParams params_;
	};

	R3DFeaturesThreadTasks *pTasks_;

	static wxMutex *pMutex_;
	static size_t doneCount_, totalCount_;
	static std::vector<int> numberOfKeypoints_;
};

#endif

