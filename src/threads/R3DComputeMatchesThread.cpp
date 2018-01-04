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
#include "R3DComputeMatchesThread.h"
#include "R3DComputeMatches.h"
#include "Regard3DMainFrame.h"
#include "R3DProject.h"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>

R3DComputeMatchesThread::R3DComputeMatchesThread() : wxThread(wxTHREAD_JOINABLE),
	pMainFrame_(NULL), pR3DComputeMatches_(NULL), isOK_(true), svgOutput_(false),
	pComputeMatches_(NULL), pPictureSet_(NULL), cameraModel_(3), matchingAlgorithm_(0)
{
}

R3DComputeMatchesThread::~R3DComputeMatchesThread()
{
}

void R3DComputeMatchesThread::setMainFrame(Regard3DMainFrame *pMainFrame)
{
	pMainFrame_ = pMainFrame;
}

void R3DComputeMatchesThread::setParameters(const Regard3DFeatures::R3DFParams &params, bool svgOutput, int cameraModel, int matchingAlgorithm)
{
	params_ = params;
	svgOutput_ = svgOutput;
	cameraModel_ = cameraModel;
	matchingAlgorithm_ = matchingAlgorithm;
}

void R3DComputeMatchesThread::setComputeMatches(R3DProject::ComputeMatches *pComputeMatches, R3DProject::PictureSet *pPictureSet)
{
	R3DProject *pProject = R3DProject::getInstance();
	pComputeMatches_ = pComputeMatches;
	pPictureSet_ = pPictureSet;
	pProject->getProjectPathsCM(paths_, pComputeMatches);
}

bool R3DComputeMatchesThread::startComputeMatchesThread()
{
	this->Create();
	this->Run();

	return true;
}

void R3DComputeMatchesThread::stopComputeMatchesThread()
{
	wxThread::ExitCode exitCode = this->Wait();
}

wxThread::ExitCode R3DComputeMatchesThread::Entry()
{
	R3DProject *pProject = R3DProject::getInstance();
	wxASSERT(pProject != NULL);

	wxDateTime beginTime = wxDateTime::UNow();

	// This call can be slow, as the matches directory might be deleted.
	// That's why it is done here and not in the main thread
	pProject->prepareComputeMatches(paths_, params_.threshold_, params_.distRatio_);

	if(pR3DComputeMatches_ != NULL)
		delete pR3DComputeMatches_;

	pR3DComputeMatches_ = new R3DComputeMatches();
	pR3DComputeMatches_->setMainFrame(pMainFrame_);

	if(pMainFrame_ != NULL)
		pMainFrame_->sendUpdateProgressBarEvent(0.1f, wxT("Importing images"));

	pProject->importAllImages(paths_);

	if(pMainFrame_ != NULL)
		pMainFrame_->sendUpdateProgressBarEvent(0.2f, wxT("Computing matches"));

	pR3DComputeMatches_->addImages(pPictureSet_->getImageInfoVector());
	isOK_ = pR3DComputeMatches_->computeMatches(params_, svgOutput_, paths_, cameraModel_, matchingAlgorithm_);
//	pR3DComputeMatches_->computeMatchesOpenCV(R3DComputeMatches::R3DFER3DF);
//	pR3DComputeMatches_->computeMatchesOpenCV_NLOPT();

	wxTimeSpan runTime = wxDateTime::UNow() - beginTime;
	prepareResultStrings(pR3DComputeMatches_->getStatistics(), runTime);

	if(pMainFrame_ != NULL)
		pMainFrame_->sendComputeMatchesFinishedEvent();

	return 0;
}

using namespace boost::accumulators;

void R3DComputeMatchesThread::prepareResultStrings(const R3DComputeMatches::R3DComputeMatchesStatistics &statistics,
	wxTimeSpan runTime)
{
	resultStrings_.Clear();

	const std::vector<int> &numberOfKeypoints = statistics.numberOfKeypoints_;

	if(!numberOfKeypoints.empty())
	{
		accumulator_set<int, stats<tag::median, tag::min, tag::max, tag::mean> > acc;
		for(size_t i = 0; i < numberOfKeypoints.size(); i++)
		{
			acc(numberOfKeypoints[i]);
		}

		resultStrings_.Add(wxT("Keypoints per image (min/max/avg/median)"));
		resultStrings_.Add(wxString::Format(wxT("%d/%d/%d/%d"), 
			(min)(acc), (max)(acc),
			static_cast<int>(mean(acc)),
			static_cast<int>(median(acc))));
	}

	pComputeMatches_->numberOfKeypoints_ = numberOfKeypoints;

	wxString runTimeStr;
	if(runTime.GetHours() > 0)
		runTimeStr = runTime.Format(wxT("%H:%M:%S.%l"));
	else
		runTimeStr = runTime.Format(wxT("%M:%S.%l"));
	resultStrings_.Add(wxT("Elapsed time"));
	resultStrings_.Add(runTimeStr);
	pComputeMatches_->runningTime_ = runTimeStr;
}
