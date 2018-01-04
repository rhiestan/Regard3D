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
#ifndef R3DComputeMatchesThread_H
#define R3DComputeMatchesThread_H

class Regard3DMainFrame;
class R3DComputeMatches;

#include "Regard3DFeatures.h"
#include "R3DComputeMatches.h"
#include "R3DProject.h"

#include <wx/thread.h>

class R3DComputeMatchesThread: public wxThread
{
public:
	R3DComputeMatchesThread();
	virtual ~R3DComputeMatchesThread();

	void setMainFrame(Regard3DMainFrame *pMainFrame);
	void setParameters(const Regard3DFeatures::R3DFParams &params, bool svgOutput, int cameraModel, int matchingAlgorithm);
	const Regard3DFeatures::R3DFParams &getParameters() const { return params_; }
	void setComputeMatches(R3DProject::ComputeMatches *pComputeMatches, R3DProject::PictureSet *pPictureSet);
	R3DProject::ComputeMatches *getComputeMatches() const { return pComputeMatches_; }

	// Starting/stopping the compute matches thread
	bool startComputeMatchesThread();
	void stopComputeMatchesThread();

	// Results
	bool getIsOK() const { return isOK_; }
	const wxString &getErrorMessage() const { return errorMessage_; }
	const wxArrayString &getResultStrings() const { return resultStrings_; }

protected:
	virtual wxThread::ExitCode Entry();

	void prepareResultStrings(const R3DComputeMatches::R3DComputeMatchesStatistics &statistics,
		wxTimeSpan runTime);

private:
	Regard3DMainFrame *pMainFrame_;

	R3DComputeMatches *pR3DComputeMatches_;

	Regard3DFeatures::R3DFParams params_;
	R3DProject::ComputeMatches *pComputeMatches_;
	R3DProject::PictureSet *pPictureSet_;
	R3DProjectPaths paths_;

	bool isOK_;
	wxString errorMessage_;
	wxArrayString resultStrings_;
	bool svgOutput_;
	int cameraModel_;
	int matchingAlgorithm_;
};

#endif
