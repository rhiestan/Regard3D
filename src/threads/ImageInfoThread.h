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

#ifndef IMAGEINFOTHREAD_H
#define IMAGEINFOTHREAD_H

class Regard3DMainFrame;

#include "ImageInfo.h"

class ImageInfoThread : public wxThread
{
public:
	ImageInfoThread();
	virtual ~ImageInfoThread();
	void setMainFrame(Regard3DMainFrame *pMainFrame);

	// Starting/stopping the ImageInfo thread
	bool startImageInfoThread();
	void stopImageInfoThread();

	// Adding a new image info request
	void addImageInfoRequest(const wxString &filename);

	// Ask for finished image info
	bool getNewImageInfo(wxString &iiFilename, ImageInfo &imageInfo);

protected:
	virtual wxThread::ExitCode Entry();

	bool createImageInfo(const wxString &localFilename, ImageInfo &imageInfo);

private:
	Regard3DMainFrame *pMainFrame_;

	wxMutex mutex_;
	wxCondition cond_;
	wxArrayString imageInfoRequestFilenames_;
	bool terminate_;

	wxMutex outputMutex_;
	wxCondition outputCond_;
	wxString iiFilename_;
	ImageInfo imageInfo_;
	bool hasNewImageInfo_;
};

#endif
