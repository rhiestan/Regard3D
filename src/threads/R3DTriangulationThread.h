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

#ifndef R3DTRIANGULATIONTHREAD_H
#define R3DTRIANGULATIONTHREAD_H

class Regard3DMainFrame;
namespace openMVG
{
	struct reconstructorHelper;
	class ReconstructionEngine;
}

#include "R3DProject.h"

#if !defined(R3D_USE_OPENMVG_PRE08)
#include "openMVG/sfm/sfm_data.hpp"
#endif

class R3DTriangulationThread : public wxThread
{
public:
	R3DTriangulationThread();
	virtual ~R3DTriangulationThread();

	void setMainFrame(Regard3DMainFrame *pMainFrame);

	void setParameters(R3DProject::R3DTriangulationAlgorithm algorithm,
		int initialPairA, int initialPairB,
		int rotAveraging, int transAveraging,
		bool refineIntrinsics, bool useGPSInfo,
		R3DProject::R3DTriangulationInitialization triInitialization);
	void setTriangulation(R3DProject *pProject, R3DProject::Triangulation *pTriangulation);
	R3DProject::Triangulation *getTriangulation() const { return pTriangulation_; }

	// Starting/stopping the triangulation thread
	bool startTriangulationThread();
	void stopTriangulationThread();

	// Results
	bool getIsOK() const { return isOK_; }
	const wxString &getErrorMessage() const { return errorMessage_; }
	const wxArrayString &getResultStrings() const { return resultStrings_; }

protected:
	virtual wxThread::ExitCode Entry();
#if defined(R3D_USE_OPENMVG_PRE08)
	void prepareResultStrings(const openMVG::reconstructorHelper* pReconstructorHelper,
		const std::vector<float> &residuals, size_t numImages, wxTimeSpan runTime);
#else
	void prepareResultStrings(const openMVG::sfm::SfM_Data &sfm_Data,
		size_t numImages, wxTimeSpan runTime);
#endif

	void updateProgressBar(float progress, const wxString &str);
	void sendFinishedEvent();

private:
	Regard3DMainFrame *pMainFrame_;

	R3DProject::R3DTriangulationAlgorithm algorithm_;
	int initialPairA_, initialPairB_;
	int rotAveraging_, transAveraging_;
	bool refineIntrinsics_;
	bool useGPSInfo_;
	R3DProject::R3DTriangulationInitialization triInitialization_;

	R3DProject *pProject_;
	R3DProject::Triangulation *pTriangulation_;
	R3DProjectPaths paths_;

	bool isOK_;
	wxString errorMessage_;
	wxArrayString resultStrings_;
};

#endif
