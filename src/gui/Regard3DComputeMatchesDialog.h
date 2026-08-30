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
#ifndef REGARD3DCOMPUTEMATCHESDIALOG_H
#define REGARD3DCOMPUTEMATCHESDIALOG_H

#include "Regard3DMainFrameBase.h"
#include "R3DProject.h"

/**
 * Everything the Compute Matches dialog collects.
 *
 * engine_ is the index of the selected choicebook page and decides which half
 * of this is meaningful: 0 the built-in engine, 1 the OpenMVG executables.
 * The camera model is shared, it feeds R3DProject::writeSfmData either way.
 */
struct R3DComputeMatchesDialogResults
{
	// The OpenMVG executables are the default; the dialog falls back to the
	// built-in engine when they are not installed
	R3DComputeMatchesDialogResults()
		: engine_(1), keypointSensitivity_(0.0007f), keypointMatchingRatio_(0.6f),
		keypointDetectorType_(0), addTBMR_(false), matchingAlgorithm_(0), cameraModel_(3) { }

	int engine_;

	// Built-in engine
	float keypointSensitivity_;
	float keypointMatchingRatio_;
	int keypointDetectorType_;
	bool addTBMR_;
	int matchingAlgorithm_;

	// Shared
	int cameraModel_;

	// OpenMVG engine
	R3DOpenMVGMatchingParams openMVG_;
};

class Regard3DComputeMatchesDialog: public Regard3DComputeMatchesDialogBase
{
public:
	Regard3DComputeMatchesDialog(wxWindow *pParent);
	virtual ~Regard3DComputeMatchesDialog();

	void setMaxPixelCount(long long maxPixelCount);

	void getResults(R3DComputeMatchesDialogResults &results);

protected:
	virtual void OnClose( wxCloseEvent& event );
	virtual void OnInitDialog( wxInitDialogEvent& event );
	virtual void OnKeypointSensitivitySlider( wxScrollEvent& event );
	virtual void OnKeypointMatchingRatioSlider( wxScrollEvent& event );
	virtual void OnOKButtonClick( wxCommandEvent& event );

	void updateICKeypointSensitivityText();
	void updateICKeypointMatchingRatioText();

	bool checkOpenMVGExecutables();
	void setOpenMVGToolTips();
	void initializeOpenMVGPage();
	bool readOpenMVGPage();
	void updateOpenMVGDescriberDependencies();
	void updateOpenMVGPairModeDependencies();
	bool readNumericField(wxTextCtrl *pTextCtrl, const wxString &name,
		double minValue, double maxValue, double &value);

private:
	float keypointSensitivity_;
	float keypointMatchingRatio_;
	long long maxPixelCount_;
	int keyPointDetectorType_;
	bool addTBMR_;
	int cameraModel_;
	int matchingAlgorithm_;
	R3DComputeMatchesDialogResults results_;

	DECLARE_EVENT_TABLE()
};

#endif
