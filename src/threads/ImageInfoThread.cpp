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
#include "ImageInfoThread.h"
#include "ExifParser.h"
#include "CameraDBLookup.h"
#include "Regard3DMainFrame.h"

#include <wx/mstream.h>

ImageInfoThread::ImageInfoThread()
	: wxThread(wxTHREAD_JOINABLE),
	pMainFrame_(NULL),
	mutex_(), cond_(mutex_), terminate_(false),
	outputMutex_(), outputCond_(outputMutex_), iiFilename_(wxEmptyString),
	hasNewImageInfo_(false)
{
	imageInfoRequestFilenames_.Clear();

	imageInfo_.imageWidth_ = 0;
	imageInfo_.imageHeight_ = 0;
	imageInfo_.cameraMaker_ = wxEmptyString;
	imageInfo_.cameraModel_ = wxEmptyString;
	imageInfo_.focalLength_ = 0;
	imageInfo_.sensorWidth_ = 0;
}

ImageInfoThread::~ImageInfoThread()
{
}

void ImageInfoThread::setMainFrame(Regard3DMainFrame *pMainFrame)
{
	pMainFrame_ = pMainFrame;
}

bool ImageInfoThread::startImageInfoThread()
{
	wxThreadError err = Create();
	if(err != wxTHREAD_NO_ERROR)
		return false;

	err = Run();
	if(err != wxTHREAD_NO_ERROR)
		return false;

	return true;
}

void ImageInfoThread::stopImageInfoThread()
{
	{
		wxMutexLocker lock(mutex_);
		terminate_ = true;
		cond_.Broadcast();
	}

	{
		wxMutexLocker lock(outputMutex_);
		hasNewImageInfo_ = false;
		outputCond_.Signal();
	}

	wxThread::ExitCode code = Wait();
}

// Adding a new preview request
void ImageInfoThread::addImageInfoRequest(const wxString &filename)
{
	{
		wxMutexLocker lock(outputMutex_);

	}

	{
		wxMutexLocker lock(mutex_);

		imageInfoRequestFilenames_.Add(filename);
		cond_.Signal();
	}
}

// Ask for finished preview image
bool ImageInfoThread::getNewImageInfo(wxString &iiFilename, ImageInfo &imageInfo)
{
	wxMutexLocker lock(outputMutex_);

	if(!hasNewImageInfo_)
		return false;

	imageInfo.imageWidth_ = imageInfo_.imageWidth_;
	imageInfo.imageHeight_ = imageInfo_.imageHeight_;
	imageInfo.cameraMaker_ = imageInfo_.cameraMaker_;
	imageInfo.cameraModel_ = imageInfo_.cameraModel_;
	imageInfo.focalLength_ = imageInfo_.focalLength_;
	imageInfo.sensorWidth_ = imageInfo_.sensorWidth_;
	iiFilename = iiFilename_;
	hasNewImageInfo_ = false;
	outputCond_.Signal();

	return true;
}

wxThread::ExitCode ImageInfoThread::Entry()
{
	while(!TestDestroy() && !terminate_)
	{
		wxString localImageInfoRequestFilename_;
		ImageInfo imageInfo;

		// Wait for new job
		{
			wxMutexLocker lock(mutex_);

			while(imageInfoRequestFilenames_.IsEmpty()
				&&!terminate_)
			{
				cond_.Wait();
			}

			if(imageInfoRequestFilenames_.GetCount() > 0)
			{
				// Consume first item in list
				localImageInfoRequestFilename_ = imageInfoRequestFilenames_.Item(0);
				imageInfoRequestFilenames_.RemoveAt(0);
			}
		}

		if(terminate_)
			return 0;

		bool fileReadOK = true;
		imageInfo.imageWidth_ = 0;
		imageInfo.imageHeight_ = 0;
		imageInfo.cameraMaker_ = wxEmptyString;
		imageInfo.cameraModel_ = wxEmptyString;
		imageInfo.focalLength_ = 0;
		imageInfo.sensorWidth_ = 0;

		try
		{
			fileReadOK = createImageInfo(localImageInfoRequestFilename_, imageInfo);
		}
		catch(std::exception &WXUNUSED(e))
		{
			fileReadOK = false;
		}
		catch(...)
		{
			fileReadOK = false;
		}

		// Copy image info to members
		if(fileReadOK)
		{
			{
				wxMutexLocker lock(outputMutex_);

				while(hasNewImageInfo_
					&&!terminate_)
				{
					outputCond_.Wait();	// Wait until output has been consumed
				}

				if(terminate_)
					return 0;

				imageInfo_.imageWidth_ = imageInfo.imageWidth_;
				imageInfo_.imageHeight_ = imageInfo.imageHeight_;
				imageInfo_.cameraMaker_ = imageInfo.cameraMaker_;
				imageInfo_.cameraModel_ = imageInfo.cameraModel_;
				imageInfo_.focalLength_ = imageInfo.focalLength_;
				imageInfo_.sensorWidth_ = imageInfo.sensorWidth_;
				iiFilename_ = localImageInfoRequestFilename_;
				hasNewImageInfo_ = true;
			}
			if(pMainFrame_ !=  NULL)
				pMainFrame_->sendNewImageInfoEvent();
		}

	}

	return 0;
}


bool ImageInfoThread::createImageInfo(const wxString &localFilename, ImageInfo &imageInfo)
{
	bool fileReadOK = false;

	{
		// Temporarily switch off logging errors for component wx
		// This is to prevent telling the user in a pop-up window that loading a
		// malformed JPEG for the preview did not work
		wxLogLevel prevLevel = wxLog::GetLogLevel();
#if wxCHECK_VERSION(2, 9, 0)
		wxLog::SetComponentLevel(wxT("wx"), wxLOG_FatalError);
#else
		wxLog::SetLogLevel(wxLOG_FatalError);
#endif

		static const int THUMBNAIL_WIDTH = 320;
		static const int THUMBNAIL_HEIGHT = 240;

		wxImage image;
#if wxCHECK_VERSION(2, 9, 0)
		image.SetOption(wxIMAGE_OPTION_MAX_WIDTH, THUMBNAIL_WIDTH);
		image.SetOption(wxIMAGE_OPTION_MAX_HEIGHT, THUMBNAIL_HEIGHT);
#endif
		//wxString imageSizeStr(wxT("Unknown file type"));
		fileReadOK = image.LoadFile(localFilename);

		if(fileReadOK)
		{
			if(image.IsOk()
#if wxCHECK_VERSION(2, 9, 0)
				&& image.GetType() != wxBITMAP_TYPE_INVALID
#endif
			)
			{
				int origWidth = image.GetWidth();
				int origHeight = image.GetHeight();
#if wxCHECK_VERSION(2, 9, 0)
				if(image.HasOption( wxIMAGE_OPTION_ORIGINAL_WIDTH )
					&& image.HasOption( wxIMAGE_OPTION_ORIGINAL_HEIGHT ))
				{
					origWidth = image.GetOptionInt( wxIMAGE_OPTION_ORIGINAL_WIDTH );
					origHeight = image.GetOptionInt( wxIMAGE_OPTION_ORIGINAL_HEIGHT );
				}
#else
				origWidth = image.GetWidth();
				origHeight = image.GetHeight();
#endif
				//imageSizeStr.Printf(wxT("%d x %d"), origWidth, origHeight);
				imageInfo.imageWidth_ = origWidth;
				imageInfo.imageHeight_ = origHeight;
			}
		}

#if wxCHECK_VERSION(2, 9, 0)
		wxLog::SetComponentLevel(wxT("wx"), prevLevel);
#else
		wxLog::SetLogLevel(prevLevel);
#endif

		ExifParser::EPExifInfo exifInfo;
		bool isOK = ExifParser::extractExifInfo(localFilename, exifInfo);
		if(isOK)
		{
			imageInfo.cameraMaker_ = exifInfo.cameraMaker_;
			imageInfo.cameraModel_ = exifInfo.cameraModel_;
			imageInfo.focalLength_ = exifInfo.focalLength_;

			double sensorWidth = 0;
			if(CameraDBLookup::getInstance().lookupCamera(exifInfo.cameraMaker_,
				exifInfo.cameraModel_, sensorWidth))
			{
				imageInfo.sensorWidth_ = sensorWidth;
			}
		}
	}

	return fileReadOK;
}
