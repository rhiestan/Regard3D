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

#ifndef EXIFPARSER_H
#define EXIFPARSER_H

#include <Eigen/Core>

class ExifParser
{
public:

	struct EPExifInfo
	{
		int width_, height_;
		wxString cameraMaker_, cameraModel_;
		double focalLength_;
		bool hasGPS_;
		double lla_[3];
		double ecef_[3];
	};
	static bool extractExifInfo(const wxString &filename, EPExifInfo &exifInfo);

	static std::pair<bool, Eigen::Vector3d> checkGPS(const wxString &filename, int GPS_to_XYZ_method = 0);

private:
	ExifParser() { }
	virtual ~ExifParser() { }
};

#endif
