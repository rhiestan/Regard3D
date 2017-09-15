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
#ifndef REGARD3DSETTINGS_H
#define REGARD3DSETTINGS_H

class Regard3DMainFrame;
class Regard3DConsoleOutputFrame;
class Regard3DImagePreviewDialog;
class Regard3DPictureSetDialog;
class Regard3DTriangulationDialog;
class Regard3DMatchingResultsDialog;
class Regard3DUserCameraDBDialog;

class Regard3DSettings
{
public:
	Regard3DSettings();
	virtual ~Regard3DSettings();

	void loadWindowLayoutFromConfig(Regard3DMainFrame *pMainFrame,
		Regard3DConsoleOutputFrame *pConsoleOutputFrame);
	void saveWindowLayoutToConfig(Regard3DMainFrame *pMainFrame,
		Regard3DConsoleOutputFrame *pConsoleOutputFrame);

	void loadImagePreviewLayoutFromConfig(Regard3DImagePreviewDialog *pDialog);
	void saveImagePreviewLayoutToConfig(Regard3DImagePreviewDialog *pDialog);
	void loadPictureSetLayoutFromConfig(Regard3DPictureSetDialog *pDialog);
	void savePictureSetLayoutToConfig(Regard3DPictureSetDialog *pDialog);
	void loadTriangulationLayoutFromConfig(Regard3DTriangulationDialog *pDialog);
	void saveTriangulationLayoutToConfig(Regard3DTriangulationDialog *pDialog);
	void loadMatchingResultsLayoutFromConfig(Regard3DMatchingResultsDialog *pDialog);
	void saveMatchingResultsLayoutToConfig(Regard3DMatchingResultsDialog *pDialog);
	void loadUserCameraDBLayoutFromConfig(Regard3DUserCameraDBDialog *pDialog);
	void saveUserCameraDBLayoutToConfig(Regard3DUserCameraDBDialog *pDialog);

	wxString getFontFilename();			// Defined by installation
	wxString getCameraDBFilename();		// Defined by installation
	wxString getExternalEXEPath();		// Defined by installation
	wxString getDefaultProjectPath();
	bool getIsMouseButtonSwitched();
	bool getIsMouseWheelSwitched();
	void setDefaultProjectPath(const wxString &defaultProjectPath);
	void setIsMouseButtonSwitched(bool isSwitched);
	void setIsMouseWheelSwitched(bool isSwitched);

	static Regard3DSettings &getInstance() { return instance_; }

private:
	wxString configWindowDimensions_;
	wxString configConsoleOutputFrameDimensions_;
	wxString configImagePreviewDimensions_;
	wxString configPictureSetDimensions_;
	wxString configTriangulationDimensions_;
	wxString configMatchingResultDimensions_;
	wxString configUserCameraDBDimensions_;
	wxString configFontFilename_;
	wxString configCameraDBFilename_;
	wxString configExternalEXEPath_;
	wxString configDefaultProjectPath_;
	wxString configIsMouseButtonSwitched_;
	wxString configIsMouseWheelSwitched_;

	static Regard3DSettings instance_;
};

#endif
