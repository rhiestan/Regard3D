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
	sensorWidthSource_(SWS_Unknown),
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
	sensorWidthSource_ = o.sensorWidthSource_;
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

wxString ImageInfo::getFocalLengthString() const
{
	if(focalLength_ <= 0)
		return wxT("Unknown");
	if(sensorWidthSource_ == SWS_Exif35mmOnly)
		return wxString::Format(wxT("%.3g mm (35 mm equiv.)"), focalLength_);
	return wxString::Format(wxT("%.3g mm"), focalLength_);
}

wxString ImageInfo::getSensorWidthString() const
{
	if(sensorWidth_ <= 0)
		return wxT("Unknown (will be estimated)");
	switch(sensorWidthSource_)
	{
	case SWS_UserDB:
		return wxString::Format(wxT("%.3g mm (user DB)"), sensorWidth_);
	case SWS_Exif35mm:
	case SWS_Exif35mmOnly:
		return wxString::Format(wxT("%.3g mm (from EXIF)"), sensorWidth_);
	default:
		return wxString::Format(wxT("%.3g mm"), sensorWidth_);
	}
}

bool ImageInfo::setFromFocalLength35mm(double focalLength35mm)
{
	// Diagonal of a 36 x 24 mm frame. The 35 mm equivalent focal length scales
	// the real one by the ratio of the diagonals (CIPA DC-008).
	const double fullFrameDiagonal = std::sqrt(36.0*36.0 + 24.0*24.0);

	if(!(focalLength35mm > 0))
		return false;

	// Fraction of the sensor diagonal that lies along the longer image side,
	// which is the side the sensor width refers to (see R3DProject::writeSfmData).
	// Without image dimensions assume 3:2.
	double widthOverDiagonal = 36.0 / fullFrameDiagonal;
	if(imageWidth_ > 0 && imageHeight_ > 0)
	{
		double w = static_cast<double>(std::max(imageWidth_, imageHeight_));
		double h = static_cast<double>(std::min(imageWidth_, imageHeight_));
		widthOverDiagonal = w / std::sqrt(w*w + h*h);
	}

	if(focalLength_ > 0)
	{
		double sensorWidth = fullFrameDiagonal * focalLength_ / focalLength35mm * widthOverDiagonal;
		// Reject implausible values: accept anything from tiny phone sensors
		// to medium format.
		if(sensorWidth < 1.0 || sensorWidth > 60.0)
			return false;
		sensorWidth_ = sensorWidth;
		sensorWidthSource_ = SWS_Exif35mm;
	}
	else
	{
		// Pretend to be a 35 mm camera: same field of view
		focalLength_ = focalLength35mm;
		sensorWidth_ = fullFrameDiagonal * widthOverDiagonal;
		sensorWidthSource_ = SWS_Exif35mmOnly;
	}

	return true;
}
