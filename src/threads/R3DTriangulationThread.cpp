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

#include "CommonIncludes.h"
#include "Regard3DMainFrame.h"
#include "R3DTriangulationThread.h"
#include "R3DProject.h"

// boost
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>

// OpenMVG
#include "software/SfM/SfMIncrementalEngine.hpp"
#include "software/globalSfM/SfMGlobalEngine.hpp"
#include "openMVG/system/timer.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

using namespace openMVG;

R3DTriangulationThread::R3DTriangulationThread() : wxThread(wxTHREAD_JOINABLE),
	pMainFrame_(NULL), global_(false),
	initialPairA_(0), initialPairB_(0),
	globalMSTBasedRot_(true), isOK_(true)
{
}

R3DTriangulationThread::~R3DTriangulationThread()
{
}

void R3DTriangulationThread::setMainFrame(Regard3DMainFrame *pMainFrame)
{
	pMainFrame_ = pMainFrame;
}

void R3DTriangulationThread::setParameters(bool global,
	int initialPairA, int initialPairB,
	bool globalMSTBasedRot)
{
	global_ = global;
	initialPairA_ = initialPairA;
	initialPairB_ = initialPairB;
	globalMSTBasedRot_ = globalMSTBasedRot;
}

void R3DTriangulationThread::setTriangulation(R3DProject *pProject, R3DProject::Triangulation *pTriangulation)
{
	pProject_ = pProject;
	pTriangulation_ = pTriangulation;
	pProject_->getProjectPathsTri(paths_, pTriangulation_);
}

bool R3DTriangulationThread::startTriangulationThread()
{
	this->Create();
	this->Run();

	return true;
}

void R3DTriangulationThread::stopTriangulationThread()
{
	wxThread::ExitCode exitCode = this->Wait();
}

wxThread::ExitCode R3DTriangulationThread::Entry()
{
	R3DProject *pProject = R3DProject::getInstance();
	wxASSERT(pProject != NULL);

	isOK_ = true;

	updateProgressBar(-1.f, wxT("Triangulating"));

	wxDateTime beginTime = wxDateTime::UNow();

	if(global_)
	{
		// The following code is copied form OpenMVG's main_GlobalSfM


		// Copyright (c) 2012, 2013, 2014 Pierre MOULON.

		// This Source Code Form is subject to the terms of the Mozilla Public
		// License, v. 2.0. If a copy of the MPL was not distributed with this
		// file, You can obtain one at http://mozilla.org/MPL/2.0/.

		std::string sImaDirectory(paths_.relativeImagePath_);	//pProject->getRelativeImagePath());
		std::string sMatchesDir(paths_.relativeMatchesPath_);	//pProject->getRelativeMatchesPath());
		std::string sOutDir(paths_.relativeOutPath_);			//pProject->getRelativeOutPath());
		bool bColoredPointCloud = true;
		int rotAvgMethod = 2;
		if(globalMSTBasedRot_)
			rotAvgMethod = 1;

		GlobalReconstructionEngine to3DEngine(sImaDirectory,
			sMatchesDir,
			sOutDir,
			true, rotAvgMethod);

		if (to3DEngine.Process())
		{
			updateProgressBar(0.8f, wxT("Colorize tracks"));

			//-- Compute color if requested
			const reconstructorHelper & reconstructorHelperRef = to3DEngine.refToReconstructorHelper();
			std::vector<Vec3> vec_tracksColor;
			if (bColoredPointCloud)
			{
				// Compute the color of each track
				to3DEngine.ColorizeTracks(to3DEngine.getTracks(), vec_tracksColor);
			}

			updateProgressBar(0.9f, wxT("Exporting model"));

			//-- Export computed data to disk
			if(!reconstructorHelperRef.exportToPly(
				stlplus::create_filespec(sOutDir, "FinalColorized", ".ply"),
				bColoredPointCloud ? &vec_tracksColor : NULL))
			{
				isOK_ = false;
				errorMessage_ = wxT("Error during exporting model to PLY");
			}

			// Export to openMVG format
			if(!reconstructorHelperRef.ExportToOpenMVGFormat(
				paths_.relativeSfmOutPath_,		//pProject->getRelativeSfMOutPath(),
				to3DEngine.getFilenamesVector(),
				sImaDirectory,
				to3DEngine.getImagesSize(),
				to3DEngine.getTracks(),
				bColoredPointCloud ? &vec_tracksColor : NULL,
				true,
				std::string("generated by the Global OpenMVG Calibration Engine")
				))
			{
				isOK_ = false;
				errorMessage_ = wxT("Error during exporting model to OpenMVG format");
			}

			std::vector<float> residuals;
			std::vector<double> residualsD = to3DEngine.getResiduals();
			for(size_t i = 0; i < residualsD.size(); i++)
				residuals.push_back(static_cast<float>(residualsD[i]));

			wxTimeSpan runTime = wxDateTime::UNow() - beginTime;
			prepareResultStrings(&reconstructorHelperRef, residuals,
				to3DEngine.getFilenamesVector().size(), runTime);
		}
		else
		{
			isOK_ = false;
			errorMessage_ = wxT("Error in global reconstruction engine");
		}


	}
	else
	{
		// The following code is copied form OpenMVG's main_incrementalSfM

		// Copyright (c) 2012, 2013 Pierre MOULON.

		// This Source Code Form is subject to the terms of the Mozilla Public
		// License, v. 2.0. If a copy of the MPL was not distributed with this
		// file, You can obtain one at http://mozilla.org/MPL/2.0/.

		std::string sImaDirectory(paths_.relativeImagePath_);			//pProject->getRelativeImagePath());
		std::string sMatchesDir(paths_.relativeMatchesPath_);			//pProject->getRelativeMatchesPath());
		std::string sOutDir(paths_.relativeOutPath_);					//pProject->getRelativeOutPath());
		bool bRefinePPandDisto = true;
		bool bColoredPointCloud = true;

		//openMVG::Timer timer;
		IncrementalReconstructionEngine to3DEngine(sImaDirectory,
			sMatchesDir, sOutDir, true);

		std::pair<size_t,size_t> initialPair(initialPairA_, initialPairB_);
		to3DEngine.setInitialPair(initialPair);
		to3DEngine.setIfRefinePrincipalPointAndRadialDisto(bRefinePPandDisto);

		if (to3DEngine.Process())
		{
	//		std::cout << std::endl << " Ac-Sfm took (s): " << timer.elapsed() << "." << std::endl;

			updateProgressBar(0.8f, wxT("Colorize tracks"));

			const reconstructorHelper & reconstructorHelperRef = to3DEngine.refToReconstructorHelper();
			std::vector<Vec3> vec_tracksColor;
			if (bColoredPointCloud)
			{
				// Compute the color of each track
				to3DEngine.ColorizeTracks(vec_tracksColor);
			}

			updateProgressBar(0.9f, wxT("Exporting model"));

			if(!reconstructorHelperRef.exportToPly(
				stlplus::create_filespec(sOutDir, "FinalColorized", ".ply"),
				bColoredPointCloud ? &vec_tracksColor : NULL))
			{
				isOK_ = false;
				errorMessage_ = wxT("Error during exporting model to PLY");
			}

			// Export to openMVG format
			if(!reconstructorHelperRef.ExportToOpenMVGFormat(
				paths_.relativeSfmOutPath_,		//pProject->getRelativeSfMOutPath(),
				to3DEngine.getFilenamesVector(),
				sImaDirectory,
				to3DEngine.getImagesSize(),
				to3DEngine.getTracks(),
				bColoredPointCloud ? &vec_tracksColor : NULL,
				true,
				std::string("generated by the Sequential OpenMVG Calibration Engine")
			))
			{
				isOK_ = false;
				errorMessage_ = wxT("Error during exporting model to OpenMVG format");
			}

			std::vector<float> residuals = to3DEngine.getResiduals();
			wxTimeSpan runTime = wxDateTime::UNow() - beginTime;
			prepareResultStrings(&reconstructorHelperRef, residuals,
				to3DEngine.getFilenamesVector().size(), runTime);
		}
		else
		{
			isOK_ = false;
			errorMessage_ = wxT("Error in incremental reconstruction engine");
		}
	}

	sendFinishedEvent();

	return 0;
}

using namespace boost::accumulators;

void R3DTriangulationThread::prepareResultStrings(const openMVG::reconstructorHelper* pReconstructorHelper,
	const std::vector<float> &residuals, size_t numImages, wxTimeSpan runTime)
{
	resultStrings_.Clear();
	if(pReconstructorHelper != NULL)
	{
		wxString cameraStr = wxString::Format(wxT("%d/%d"), 
			static_cast<int>(pReconstructorHelper->map_Camera.size()),
			static_cast<int>(numImages));
		resultStrings_.Add(wxT("Cameras calibrated/total"));
		resultStrings_.Add(cameraStr);

		wxString numPointsStr = wxString::Format(wxT("%d"), 
			static_cast<int>(pReconstructorHelper->map_3DPoints.size()));
		resultStrings_.Add(wxT("Number of 3D points"));
		resultStrings_.Add(numPointsStr);

		pTriangulation_->resultCameras_ = cameraStr;
		pTriangulation_->resultNumberOfTracks_ = numPointsStr;
	}

	if(!residuals.empty())
	{
		accumulator_set<float, stats<tag::median, tag::min, tag::max, tag::mean> > acc;
		for(size_t i = 0; i < residuals.size(); i++)
		{
			acc(residuals[i]);
		}
		wxString residualErrorsString = wxString::Format(wxT("%.3g/%.3g/%.3g/%.3g"), 
			static_cast<double>((boost::accumulators::min)(acc)),
			static_cast<double>((boost::accumulators::max)(acc)),
			static_cast<double>(mean(acc)),
			static_cast<double>(median(acc)));
		resultStrings_.Add(wxT("Residual errors (min/max/avg/median)"));
		resultStrings_.Add(residualErrorsString);
		pTriangulation_->resultResidualErrors_ = residualErrorsString;
	}

	wxString runTimeStr;
	if(runTime.GetHours() > 0)
		runTimeStr = runTime.Format(wxT("%H:%M:%S.%l"));
	else
		runTimeStr = runTime.Format(wxT("%M:%S.%l"));
	resultStrings_.Add(wxT("Elapsed time"));
	resultStrings_.Add(runTimeStr);
	pTriangulation_->runningTime_ = runTimeStr;
}

void R3DTriangulationThread::updateProgressBar(float progress, const wxString &str)
{
	if(pMainFrame_ != NULL)
		pMainFrame_->sendUpdateProgressBarEvent(progress, str);
}

void R3DTriangulationThread::sendFinishedEvent()
{
	if(pMainFrame_ != NULL)
		pMainFrame_->sendTriangulationFinishedEvent();
}
