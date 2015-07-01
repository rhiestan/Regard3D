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
#include "R3DFeaturesThread.h"
#include "Regard3DMainFrame.h"

// OpenMVG
#include "openMVG/image/image.hpp"
#include "openMVG/features/features.hpp"

// OpenCV
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

wxMutex *R3DFeaturesThread::pMutex_ = NULL;
size_t R3DFeaturesThread::doneCount_ = 0;
size_t R3DFeaturesThread::totalCount_ = 0;
std::vector<int> R3DFeaturesThread::numberOfKeypoints_;

bool R3DFeaturesThread::extractFeaturesAndDescriptors(
	const std::vector<std::string> &vec_fileNames,
	const std::string &sOutDir,
	const Regard3DFeatures::R3DFParams &params)
{
	pMutex_ = new wxMutex();
	doneCount_ = 0;
	totalCount_ = vec_fileNames.size();
	numberOfKeypoints_.clear();

	R3DFeaturesThreadTasks tasks;
	// Copy parameters
	for(size_t j = 0; j < vec_fileNames.size(); j++)
	{
		wxString fn = wxString(vec_fileNames[j].c_str(), *wxConvCurrent);
		tasks.fileNames_.Add(fn);
	}
	tasks.outDir_ = wxString(sOutDir.c_str(), *wxConvCurrent);
	tasks.params_ = params;

	int numberOfThreads = std::max(1, wxThread::GetCPUCount() + 1);
	std::vector<R3DFeaturesThread*> fts;
	for(int i = 0; i < numberOfThreads; i++)
	{
		R3DFeaturesThread *pFT = new R3DFeaturesThread();
		pFT->pTasks_ = &tasks;

		fts.push_back(pFT);

		pFT->Create();
		pFT->Run();
	}

	// Wait until threads have completed
	for(int i = 0; i < numberOfThreads; i++)
	{
		R3DFeaturesThread *pFT = fts[i];
		pFT->Wait();
		delete pFT;
	}

	delete pMutex_;
	pMutex_ = NULL;

	return true;
}

R3DFeaturesThread::R3DFeaturesThread() : wxThread(wxTHREAD_JOINABLE)
{
}

R3DFeaturesThread::~R3DFeaturesThread()
{
}

wxThread::ExitCode R3DFeaturesThread::Entry()
{
	if(pTasks_ == NULL)
		return 0;

	wxString workItem = getNewWorkItem();
	while(!workItem.IsEmpty())
	{
		processWorkItem(workItem);

		workItem = getNewWorkItem();
	}
	
	return 0;
}

wxString R3DFeaturesThread::getNewWorkItem()
{
	wxString workItem;
	wxMutexLocker lock(pTasks_->mutex_);

	if(!pTasks_->fileNames_.IsEmpty())
	{
		workItem = pTasks_->fileNames_[0];
		pTasks_->fileNames_.RemoveAt(0);
	}

	return workItem;
}

void R3DFeaturesThread::processWorkItem(wxString &workItem)
{
	//Image<unsigned char> imageGray;
	openMVG::image::Image<float> imageGray;
	Regard3DFeatures::KeypointSetR3D kpSet;

	wxFileName fnImage(workItem);
	wxFileName fnFeature, fnDesc;

	fnFeature.SetPath(pTasks_->outDir_);
	fnFeature.SetName(fnImage.GetName());
	fnFeature.SetExt(wxT("feat"));
	fnDesc.Assign(fnFeature);
	fnDesc.SetExt(wxT("desc"));

	//Test if descriptor and feature was already computed
	if(fnFeature.FileExists() && fnDesc.FileExists())
	{

	}
	else
	{
		// Convert back to C string
		std::string filename(workItem.mb_str());
		std::string sFeat(fnFeature.GetFullPath().mb_str());
		std::string sDesc(fnDesc.GetFullPath().mb_str());

		//Not already computed, so compute and save
//		if (!openMVG::ReadImage(filename.c_str(), &imageGray))
//			return;

		int cols = imageGray.cols();
		int rows = imageGray.rows();

		// Load image, convert to float, then to gray image
		// Note: This is probably neither the fastest, nor the most space-efficient way to do this,
		// but this way achieves better quality than by first converting an uchar-RGB-image to uchar-gray
		// which involves quantization.
		{
			cv::Mat cvimg;
			cvimg = cv::imread(filename.c_str());
			int type = cvimg.type();
			if(type == CV_32F)
			{
				int jj = 27;
			}
			else if(type == CV_8UC3)
			{
				int jj = 27;
			}
			// Channels
			int channels = cvimg.channels();
			int w = cvimg.cols;
			int h = cvimg.rows;

			// Convert to float
			cv::Mat cvimg_float;
			cvimg.convertTo(cvimg_float, CV_32FC3, 1.0/255.0, 0);
			cvimg.release();		// Dispose of memory

			// Convert to gray
			cv::Mat cvimg_gray;
			cv::cvtColor(cvimg_float, cvimg_gray, CV_BGR2GRAY);
			cvimg_float.release();		// Dispose of memory
			type = cvimg_gray.type();
			channels = cvimg_gray.channels();


			imageGray = Eigen::Map<openMVG::image::Image<float>::Base>(cvimg_gray.ptr<float>(0), h, w);
			cvimg_gray.release();		// Dispose of memory
			int cols2 = imageGray.cols();
			int rows2 = imageGray.rows();
		}

		// Compute features and descriptors and export them to file
		Regard3DFeatures::detectAndExtract(imageGray,  kpSet.features(),
			kpSet.descriptors(), pTasks_->params_);
		kpSet.saveToBinFile(sFeat, sDesc);
		{
			wxMutexLocker lock(*pMutex_);
			doneCount_++;
			numberOfKeypoints_.push_back(static_cast<int>(kpSet.features().size()));
		}
		sendMsgToMainFrame(workItem, kpSet.features().size());

	}
}

void R3DFeaturesThread::sendMsgToMainFrame(const wxString &workItem, size_t nrOfKeypoints)
{
	float finishedFactor = 0;
	{
		wxMutexLocker lock(*pMutex_);
		finishedFactor = static_cast<float>(doneCount_) / static_cast<int>(totalCount_);
	}
	float progress = (finishedFactor * 0.4f) + 0.2f;	// Between 20% and 60%

	// Regard3DMainFrame
	wxWindow *pTopWindow = wxTheApp->GetTopWindow();
	if(pTopWindow != NULL)
	{
		Regard3DMainFrame *pRegard3DMainFrame = dynamic_cast<Regard3DMainFrame*>(pTopWindow);
		if(pRegard3DMainFrame != NULL)
		{
			//pRegard3DMainFrame->sendNewKeypointsEvent(workItem, nrOfKeypoints);
			pRegard3DMainFrame->sendUpdateProgressBarEvent(progress, wxEmptyString);
		}
	}
}
