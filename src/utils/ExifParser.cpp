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
#include "ExifParser.h"

//#include "exif/exif_IO_EasyExif.hpp"

#include "third_party/easyexif/exif.h"

#include "openMVG/geodesy/geodesy.hpp"


bool ExifParser::extractExifInfo(const wxString &filename, ExifParser::EPExifInfo &epExifInfo)
{
	easyexif::EXIFInfo exifInfo;

	// Read file into memory
	wxFile imageFile(filename, wxFile::read);
	if(!imageFile.IsOpened())
		return false;
	wxFileOffset fileSize = imageFile.Length();
	std::vector<unsigned char> buf(fileSize);
	ssize_t bytesRead = imageFile.Read(&(buf[0]), fileSize);
	if(bytesRead == wxInvalidOffset)
		return false;
	imageFile.Close();


	int retVal = exifInfo.parseFrom(&(buf[0]), fileSize);
	if(retVal != PARSE_EXIF_SUCCESS)
		return false;

	epExifInfo.width_ = exifInfo.ImageWidth;
	epExifInfo.height_ = exifInfo.ImageHeight;
	epExifInfo.cameraMaker_ = wxString(exifInfo.Make.c_str(), wxConvLibc).Trim(false).Trim(true);
	epExifInfo.cameraModel_ = wxString(exifInfo.Model.c_str(), wxConvLibc).Trim(false).Trim(true);
	epExifInfo.focalLength_ = exifInfo.FocalLength;

	// Check existence of GPS coordinates
	double latitude = exifInfo.GeoLocation.Latitude;
	double longitude = exifInfo.GeoLocation.Longitude;
	double altitude = exifInfo.GeoLocation.Altitude;
	if(std::isfinite(latitude) &&
		std::isfinite(longitude) &&
		std::isfinite(altitude))
	{
		epExifInfo.hasGPS_ = true;
		epExifInfo.lla_[0] = latitude;
		epExifInfo.lla_[1] = longitude;
		epExifInfo.lla_[2] = altitude;

		// Add ECEF position to the GPS position array
		Eigen::Vector3d ecef = openMVG::geodesy::lla_to_ecef(latitude, longitude, altitude);
		epExifInfo.ecef_[0] = ecef(0);
		epExifInfo.ecef_[1] = ecef(1);
		epExifInfo.ecef_[2] = ecef(2);
	}
	else
	{
		epExifInfo.hasGPS_ = false;
		epExifInfo.lla_[0] = epExifInfo.lla_[1] = epExifInfo.lla_[2] = 0;
		epExifInfo.ecef_[0] = epExifInfo.ecef_[1] = epExifInfo.ecef_[2] = 0;
	}

	return true;
}

std::pair<bool, Eigen::Vector3d> ExifParser::checkGPS(const wxString & filename, int GPS_to_XYZ_method)
{
	std::pair<bool, Eigen::Vector3d> val(false, Eigen::Vector3d::Zero());

	easyexif::EXIFInfo exifInfo;

	// Read file into memory
	wxFile imageFile(filename, wxFile::read);
	if(!imageFile.IsOpened())
		return val;
	wxFileOffset fileSize = imageFile.Length();
	std::vector<unsigned char> buf(fileSize);
	ssize_t bytesRead = imageFile.Read(&(buf[0]), fileSize);
	if(bytesRead == wxInvalidOffset)
		return val;
	imageFile.Close();


	int retVal = exifInfo.parseFrom(&(buf[0]), fileSize);
	if(retVal != PARSE_EXIF_SUCCESS)
		return val;

	// Check existence of GPS coordinates
	double latitude = exifInfo.GeoLocation.Latitude;
	double longitude = exifInfo.GeoLocation.Longitude;
	double altitude = exifInfo.GeoLocation.Altitude;
	if(std::isfinite(latitude) &&
		std::isfinite(longitude) &&
		std::isfinite(altitude))
	{
		// Add ECEF or UTM XYZ position to the GPS position array
		val.first = true;
		switch(GPS_to_XYZ_method)
		{
		case 1:
			val.second = openMVG::geodesy::lla_to_utm(latitude, longitude, altitude);
			break;
		case 0:
		default:
			val.second = openMVG::geodesy::lla_to_ecef(latitude, longitude, altitude);
			break;
		}
	}

	return val;
}
