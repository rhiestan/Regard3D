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
#include "ImageInfo.h"

ImageInfo::ImageInfo()
	:	isImported_(false),
	imageWidth_(0), imageHeight_(0),
	focalLength_(0), sensorWidth_(0),
	hasGPSInfo_(false)
{
	for(int i = 0; i < 3; i++)
	{
		lla_[i] = 0;
		ecef_[i] = 0;
	}
}

ImageInfo::ImageInfo(const ImageInfo &o)
{
	copy(o);
}

ImageInfo::~ImageInfo()
{
}

ImageInfo &ImageInfo::copy(const ImageInfo &o)
{
	filename_ = o.filename_;
	importedFilename_ = o.importedFilename_;
	isImported_ = o.isImported_;
	imageWidth_ = o.imageWidth_;
	imageHeight_ = o.imageHeight_;
	cameraMaker_ = o.cameraMaker_;
	cameraModel_ = o.cameraModel_;
	focalLength_ = o.focalLength_;
	sensorWidth_ = o.sensorWidth_;
	hasGPSInfo_ = o.hasGPSInfo_;
	for(int i = 0; i < 3; i++)
	{
		lla_[i] = o.lla_[i];
		ecef_[i] = o.ecef_[i];
	}

	return *this;
}


ImageInfo & ImageInfo::operator=(const ImageInfo & o)
{
	return copy(o);
}
