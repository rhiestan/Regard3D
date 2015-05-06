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

#include "third_party/easyexif/exif.h"

bool ExifParser::extractExifInfo(const wxString &filename, ExifParser::EPExifInfo &epExifInfo)
{
	EXIFInfo exifInfo;

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

	return true;
}
