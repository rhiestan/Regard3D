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

#ifndef PREVIEWGENERATORTHREAD_H
#define PREVIEWGENERATORTHREAD_H

class Regard3DMainFrame;

/**
 * Class to hold preview information.
 */
class PreviewInfo
{
public:
	PreviewInfo() : type_(PITKeypoints), keypointType_(PIKPTNoKeypoints),
		showSingleKeypoints_(false), enableTrackFilter_(false),
		index1_(0), index2_(0) { }
	PreviewInfo(const PreviewInfo &o) { copy(o); }
	virtual ~PreviewInfo() { }
	PreviewInfo &copy(const PreviewInfo &o)
	{
		type_ = o.type_;
		keypointType_ = o.keypointType_;
		showSingleKeypoints_ = o.showSingleKeypoints_;
		enableTrackFilter_ = o.enableTrackFilter_;
		filename1_ = o.filename1_;
		filename2_ = o.filename2_;
		kpFilename1_ = o.kpFilename1_;
		kpFilename2_ = o.kpFilename2_;
		matchesFilename_ = o.matchesFilename_;
		index1_ = o.index1_;
		index2_ = o.index2_;
		return *this;
	}
	PreviewInfo & operator=(const PreviewInfo & o) { return copy(o); }
	void clear()
	{
		*this = PreviewInfo();	// Overwrite with default values
	}

	enum PreviewInfoType {
		PITKeypoints = 0,
		PITMatches = 1
	};
	enum PreviewInfoKPType {
		PIKPTNoKeypoints = 0,
		PIKPTSimpleKeypoints = 1,
		PIKPTRichKeypoints = 2
	};
	PreviewInfoType type_;
	PreviewInfoKPType keypointType_;
	bool showSingleKeypoints_, enableTrackFilter_;
	wxString filename1_, filename2_;
	wxString kpFilename1_, kpFilename2_;
	wxString matchesFilename_;
	size_t index1_, index2_;
};

class PreviewGeneratorThread : public wxThread
{
public:
	PreviewGeneratorThread();
	virtual ~PreviewGeneratorThread();
	void setMainFrame(Regard3DMainFrame *pMainFrame);

	// Starting/stopping the preview thread
	bool startPreviewThread();
	void stopPreviewThread();

	// Adding a new preview request
	void addPreviewRequest(const wxString &filename);
	void addPreviewRequestKP(const wxString &filename, const wxString &kpfilename);
	void addPreviewRequest(const PreviewInfo &previewInfo);

	// Ask for finished preview image
	bool getNewPreviewImage(wxImage &previewImage, PreviewInfo &previewInfo);

	// Cancel a running preview generation
	void cancelPreview();

protected:
	virtual wxThread::ExitCode Entry();

	bool createPreview(wxImage &localPreviewImage, PreviewInfo &previewInfo);

private:
	Regard3DMainFrame *pMainFrame_;

	wxMutex mutex_;
	wxCondition cond_;
	PreviewInfo previewInfoIn_;
	bool hasNewPreviewRequest_;
	bool terminate_;
	bool cancelPreview_;

	wxMutex outputMutex_;
	wxImage previewImage_;
	PreviewInfo previewInfoOut_;
	bool hasNewPreview_;
};

#endif
