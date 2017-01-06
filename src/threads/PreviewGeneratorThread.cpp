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
#include "PreviewGeneratorThread.h"
#include "Regard3DMainFrame.h"
#include "Regard3DFeatures.h"
#include "OpenCVHelper.h"

#include <wx/mstream.h>

// openMVG
#include "openMVG/features/features.hpp"
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/tracks/tracks.hpp"

// OpenCV
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/highgui/highgui.hpp"

PreviewGeneratorThread::PreviewGeneratorThread()
	: wxThread(wxTHREAD_JOINABLE),
	pMainFrame_(NULL),
	mutex_(), cond_(mutex_),
	hasNewPreviewRequest_(false), terminate_(false), cancelPreview_(false),
	outputMutex_(), previewImage_(), hasNewPreview_(false)
{
}

PreviewGeneratorThread::~PreviewGeneratorThread()
{
}

void PreviewGeneratorThread::setMainFrame(Regard3DMainFrame *pMainFrame)
{
	pMainFrame_ = pMainFrame;
}

bool PreviewGeneratorThread::startPreviewThread()
{
	wxThreadError err = Create();
	if(err != wxTHREAD_NO_ERROR)
		return false;

	err = Run();
	if(err != wxTHREAD_NO_ERROR)
		return false;

	return true;
}

void PreviewGeneratorThread::stopPreviewThread()
{
	{
		wxMutexLocker lock(mutex_);
		terminate_ = true;
		cond_.Broadcast();
	}

	wxThread::ExitCode code = Wait();
}

// Adding a new preview request
void PreviewGeneratorThread::addPreviewRequest(const wxString &filename)
{
	{
		wxMutexLocker lock(outputMutex_);

		cancelPreview_ = false;
	}

	{
		wxMutexLocker lock(mutex_);
		previewInfoIn_.type_ = PreviewInfo::PITKeypoints;
		previewInfoIn_.keypointType_ = PreviewInfo::PIKPTNoKeypoints;
		previewInfoIn_.showSingleKeypoints_ = false;
		previewInfoIn_.filename1_ = filename;
		previewInfoIn_.filename2_.Empty();
		previewInfoIn_.kpFilename1_.Empty();
		previewInfoIn_.kpFilename2_.Empty();
		previewInfoIn_.matchesFilename_.Empty();
		previewInfoIn_.index1_ = 0;
		previewInfoIn_.index2_ = 0;
		hasNewPreviewRequest_ = true;
		cond_.Signal();
	}
}

void PreviewGeneratorThread::addPreviewRequestKP(const wxString &filename, const wxString &kpfilename)
{
	{
		wxMutexLocker lock(outputMutex_);

		cancelPreview_ = false;
	}

	{
		wxMutexLocker lock(mutex_);

		previewInfoIn_.type_ = PreviewInfo::PITKeypoints;
		previewInfoIn_.keypointType_ = PreviewInfo::PIKPTRichKeypoints;
		previewInfoIn_.showSingleKeypoints_ = false;
		previewInfoIn_.filename1_ = filename;
		previewInfoIn_.filename2_.Empty();
		previewInfoIn_.kpFilename1_ = kpfilename;
		previewInfoIn_.kpFilename2_.Empty();
		previewInfoIn_.matchesFilename_.Empty();
		previewInfoIn_.index1_ = 0;
		previewInfoIn_.index2_ = 0;
		hasNewPreviewRequest_ = true;
		cond_.Signal();
	}
}

void PreviewGeneratorThread::addPreviewRequest(const PreviewInfo &previewInfo)
{
	{
		wxMutexLocker lock(outputMutex_);

		cancelPreview_ = false;
	}

	{
		wxMutexLocker lock(mutex_);

		previewInfoIn_ = previewInfo;

		hasNewPreviewRequest_ = true;
		cond_.Signal();
	}
}

// Ask for finished preview image
bool PreviewGeneratorThread::getNewPreviewImage(wxImage &previewImage, PreviewInfo &previewInfo)
{
	wxMutexLocker lock(outputMutex_);

	if(!hasNewPreview_)
		return false;

	if(cancelPreview_)
	{
		//cancelPreview_ = false;
		hasNewPreview_ = false;
		return false;
	}

	previewImage = previewImage_;
	previewInfo = previewInfoOut_;
	hasNewPreview_ = false;

	return true;
}

void PreviewGeneratorThread::cancelPreview()
{
	wxMutexLocker lock(outputMutex_);

	cancelPreview_ = true;
}

wxThread::ExitCode PreviewGeneratorThread::Entry()
{
	while(!TestDestroy() && !terminate_)
	{
		wxImage localPreviewImage;
		PreviewInfo localPreviewInfo;

		// Wait for new job
		{
			wxMutexLocker lock(mutex_);

			while(!hasNewPreviewRequest_
				&&!terminate_)
			{
				cond_.Wait();
			}

			localPreviewInfo = previewInfoIn_;
			hasNewPreviewRequest_ = false;
		}

		if(terminate_)
			return 0;

		bool fileReadOK = true;
		bool isMatchesPreview = false;

		try
		{
			fileReadOK = createPreview(localPreviewImage, localPreviewInfo);
		}
		catch(std::exception &WXUNUSED(e))
		{
			fileReadOK = false;
		}
		catch(...)
		{
			fileReadOK = false;
		}

		// Copy preview to previewImage_
		if(fileReadOK)
		{
			{
				wxMutexLocker lock(outputMutex_);
				previewImage_ = localPreviewImage.Copy();	// Make an actual copy, not just a new reference
				previewInfoOut_ = localPreviewInfo;
				hasNewPreview_ = true;
			}
			if(pMainFrame_ !=  NULL)
				pMainFrame_->sendNewPreviewImageEvent();
		}

	}

	return 0;
}


bool PreviewGeneratorThread::createPreview(wxImage &localPreviewImage, PreviewInfo &previewInfo)
{
	bool fileReadOK = false;

	{
		wxImage image2;

		// Temporarily switch off logging errors for component wx
		// This is to prevent telling the user in a pop-up window that loading a
		// malformed JPEG for the preview did not work
		wxLogLevel prevLevel = wxLog::GetLogLevel();
#if wxCHECK_VERSION(2, 9, 0)
		wxLog::SetComponentLevel(wxT("wx"), wxLOG_FatalError);
#else
		wxLog::SetLogLevel(wxLOG_FatalError);
#endif
		// Load JPEG image
		fileReadOK = localPreviewImage.LoadFile(previewInfo.filename1_, wxBITMAP_TYPE_JPEG);	// TODO: Allow all types ... ?

		if(!previewInfo.filename2_.IsEmpty())
		{
			fileReadOK = image2.LoadFile(previewInfo.filename2_, wxBITMAP_TYPE_JPEG);	// TODO: Allow all types ... ?
		}

#if wxCHECK_VERSION(2, 9, 0)
		wxLog::SetComponentLevel(wxT("wx"), prevLevel);
#else
		wxLog::SetLogLevel(prevLevel);
#endif

		if(previewInfo.type_ == PreviewInfo::PITKeypoints
			&& previewInfo.keypointType_ != PreviewInfo::PIKPTNoKeypoints
			&& !previewInfo.kpFilename1_.IsEmpty())
		{
			// Load features from file
			Regard3DFeatures::FeatsR3D features;
			std::string kpfilename(previewInfo.kpFilename1_.mb_str());
			if(openMVG::features::loadFeatsFromFile(kpfilename, features))
			{
				// Convert to OpenCV keypoints
				std::vector< cv::KeyPoint > vec_keypoints;
				vec_keypoints.reserve(features.size());
				for(size_t i = 0; i < features.size(); i++)
				{
					const Regard3DFeatures::FeatureR3D &ft = features[i];
					vec_keypoints.push_back( cv::KeyPoint( ft.x(), ft.y(), ft.scale() * 2.0f, ft.orientation() ) ); 
				}

				// Convert image to OpenCV
				cv::Mat img_cv;
				OpenCVHelper::convertWxImageToCVMat(localPreviewImage, img_cv);

				// Add keypoints to image
				cv::Mat img_outcv;
				int flags = cv::DrawMatchesFlags::DEFAULT;
				if(previewInfo.keypointType_ == PreviewInfo::PIKPTRichKeypoints)
					flags = cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
				cv::drawKeypoints(img_cv, vec_keypoints, img_outcv, cv::Scalar::all(-1), flags);
				//cv::imwrite("img_outcv.png", img_outcv);

				// Copy back to wxImage
				OpenCVHelper::convertCVMatToWxImage(img_outcv, localPreviewImage);
			}
		}

		if(previewInfo.type_ == PreviewInfo::PITMatches)
        {
            if(previewInfo.keypointType_ != PreviewInfo::PIKPTNoKeypoints
               && !previewInfo.matchesFilename_.IsEmpty())
            {
                // Load features from file
                std::vector< cv::KeyPoint > vec_keypoints1, vec_keypoints2;
                Regard3DFeatures::FeatsR3D features;
                std::string kpfilename(previewInfo.kpFilename1_.mb_str());
				if(openMVG::features::loadFeatsFromFile(kpfilename, features))
                {
                    // Convert to OpenCV keypoints
                    vec_keypoints1.reserve(features.size());
                    for(size_t i = 0; i < features.size(); i++)
                    {
                        const Regard3DFeatures::FeatureR3D &ft = features[i];
                        vec_keypoints1.push_back( cv::KeyPoint( ft.x(), ft.y(), ft.scale() * 2.0f, ft.orientation() ) ); 
                    }
                }
                features.clear();
                kpfilename = previewInfo.kpFilename2_.mb_str();
				if(openMVG::features::loadFeatsFromFile(kpfilename, features))
                {
                    // Convert to OpenCV keypoints
                    vec_keypoints2.reserve(features.size());
                    for(size_t i = 0; i < features.size(); i++)
                    {
                        const Regard3DFeatures::FeatureR3D &ft = features[i];
                        vec_keypoints2.push_back( cv::KeyPoint( ft.x(), ft.y(), ft.scale() * 2.0f, ft.orientation() ) ); 
                    }
                }
                // Load matches from file
                std::string sMatchFile(previewInfo.matchesFilename_.mb_str());
                openMVG::matching::PairWiseMatches map_Matches;
				openMVG::matching::Load(map_Matches, sMatchFile);

				std::vector< cv::DMatch > matches1to2;

				if(previewInfo.enableTrackFilter_)
				{
					// Calculate tracks from matches
					openMVG::tracks::TracksBuilder tracksBuilder;
					openMVG::tracks::STLMAPTracks map_tracks;
					tracksBuilder.Build(map_Matches);
					tracksBuilder.Filter();

					//-- Build tracks with STL compliant type :
					tracksBuilder.ExportToSTL(map_tracks);

					openMVG::tracks::STLMAPTracks map_tracksCommon;
					std::set<size_t> set_imageIndex;
					set_imageIndex.insert(previewInfo.index1_);
					set_imageIndex.insert(previewInfo.index2_);
					openMVG::tracks::TracksUtilsMap::GetTracksInImages(set_imageIndex, map_tracks, map_tracksCommon);

					openMVG::tracks::STLMAPTracks::const_iterator iterT = map_tracksCommon.begin();
					while(iterT != map_tracksCommon.end())
					{
						openMVG::tracks::submapTrack::const_iterator iter = iterT->second.begin();

						size_t kpIndex1 = iter->second; iter++;
						size_t kpIndex2 = iter->second;
						matches1to2.push_back(cv::DMatch(kpIndex1, kpIndex2, 1.0f));

						iterT++;
					}
				}
				else
				{
					openMVG::matching::PairWiseMatches::const_iterator iter = map_Matches.begin();
					while(iter != map_Matches.end())
					{
						size_t imgIndex1 = iter->first.first;
						size_t imgIndex2 = iter->first.second;

						if((imgIndex1 == previewInfo.index1_
								&& imgIndex2 == previewInfo.index2_)
							|| (imgIndex1 == previewInfo.index2_
								&& imgIndex2 == previewInfo.index1_))
						{
							const std::vector<openMVG::matching::IndMatch> &vec_FilteredMatches = iter->second;
							for(size_t i = 0; i < vec_FilteredMatches.size(); i++)
							{
								const openMVG::matching::IndMatch &match = vec_FilteredMatches[i];
								size_t kpIndex1 = match.i_;
								size_t kpIndex2 = match.j_;
								matches1to2.push_back(cv::DMatch(kpIndex1, kpIndex2, 1.0f));
							}
						}
						iter++;
					}
				}

                // Convert images to OpenCV
                cv::Mat img1, img2;
                OpenCVHelper::convertWxImageToCVMat(localPreviewImage, img1);
                OpenCVHelper::convertWxImageToCVMat(image2, img2);

                cv::Mat outimg;
				int flags = cv::DrawMatchesFlags::DEFAULT;
				if(previewInfo.keypointType_ == PreviewInfo::PIKPTRichKeypoints)
					flags = cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
				if(!previewInfo.showSingleKeypoints_)
					flags |= cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS;
                cv::drawMatches(img1, vec_keypoints1, img2, vec_keypoints2, matches1to2, outimg, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), flags);

                // Copy back to wxImage
				OpenCVHelper::convertCVMatToWxImage(outimg, localPreviewImage);
            }
            else
            {
                // Create an image with both images next to each other
                wxImage tmpImg;
                int w = localPreviewImage.GetWidth() + image2.GetWidth();
                int h = std::max( localPreviewImage.GetHeight(), image2.GetHeight());
                tmpImg.Create(w, h);
                tmpImg.Paste(localPreviewImage, 0, 0);
                tmpImg.Paste(image2, localPreviewImage.GetWidth(), 0);
                localPreviewImage = tmpImg;
            }
        }

	}

	return fileReadOK;
}
