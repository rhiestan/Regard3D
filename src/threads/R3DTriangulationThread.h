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
		R3DProject::R3DTriangulationInitialization triInitialization,
		const R3DOpenMVGTriangulationParams &openMVGParams);
	void setTriangulation(R3DProject *pProject, R3DProject::Triangulation *pTriangulation);

	/**
	 * Switches the thread to finishing what openMVG_main_SfM produced.
	 *
	 * The executable writes sfm_data.bin, the .ply of points and poses and the
	 * HTML report, but not the colorized model Regard3D displays, and the
	 * statistics still have to be read back out of the scene. Doing that here
	 * means both engines end in the same OnTriangulationFinished.
	 *
	 * isOK false carries a failure of the executable through unchanged.
	 */
	void setExternalResult(bool isOK, const wxString &errorMessage,
		wxTimeSpan runTime, bool wasCancelled);
	R3DProject::Triangulation *getTriangulation() const { return pTriangulation_; }

	// Starting/stopping the triangulation thread
	bool startTriangulationThread();
	void stopTriangulationThread();

	// Results
	bool getIsOK() const { return isOK_; }
	// True when the user aborted openMVG_main_SfM; always false for the
	// built-in engine, which cannot be stopped halfway
	bool getWasCancelled() const { return wasCancelled_; }
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

	void finishExternalTriangulation();
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
	// Not only for openMVG_main_SfM: the refinement options in there are
	// understood by the built-in engines as well
	R3DOpenMVGTriangulationParams openMVGParams_;

	// Set by setExternalResult: openMVG_main_SfM has already run
	bool finishExternalOnly_;
	wxTimeSpan externalRunTime_;

	R3DProject *pProject_;
	R3DProject::Triangulation *pTriangulation_;
	R3DProjectPaths paths_;

	bool isOK_, wasCancelled_;
	wxString errorMessage_;
	wxArrayString resultStrings_;
};

#endif
