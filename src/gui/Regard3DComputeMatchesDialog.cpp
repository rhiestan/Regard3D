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
#include "Regard3DComputeMatchesDialog.h"

Regard3DComputeMatchesDialog::Regard3DComputeMatchesDialog(wxWindow *pParent)
	: Regard3DComputeMatchesDialogBase(pParent),
	keypointSensitivity_(0.0007f), keypointMatchingRatio_(0.6f),
	maxPixelCount_(0), 	keyPointDetectorType_(0),
	addTBMR_(false), cameraModel_(3)
{
}

Regard3DComputeMatchesDialog::~Regard3DComputeMatchesDialog()
{
}

void Regard3DComputeMatchesDialog::setMaxPixelCount(long long maxPixelCount)
{
	maxPixelCount_ = maxPixelCount;
}

void Regard3DComputeMatchesDialog::getResults(float &keypointSensitivity,
	float &keypointMatchingRatio,
	int &keypointDetectorType,
	bool &addTBMR, int &cameraModel)
{
	keypointSensitivity = keypointSensitivity_;
	keypointMatchingRatio = keypointMatchingRatio_;
	keypointDetectorType = keyPointDetectorType_;
	addTBMR = addTBMR_;
	cameraModel = cameraModel_;
}

void Regard3DComputeMatchesDialog::OnClose(wxCloseEvent& event)
{
}

void Regard3DComputeMatchesDialog::OnInitDialog( wxInitDialogEvent& event )
{
	pKeypointSensitivitySlider_->SetValue(1);
	updateICKeypointSensitivityText();
	pKeypointMatchingRatioSlider_->SetValue(0);
	updateICKeypointMatchingRatioText();

	pKeypointDetectorRadioBox_->SetSelection(keyPointDetectorType_);
	pAddTBMRDetectorCheckBox_->SetValue(addTBMR_);

	Fit();
	CenterOnParent();
}

void Regard3DComputeMatchesDialog::OnKeypointSensitivitySlider( wxScrollEvent& event )
{
	updateICKeypointSensitivityText();
}

void Regard3DComputeMatchesDialog::OnKeypointMatchingRatioSlider( wxScrollEvent& event )
{
	updateICKeypointMatchingRatioText();
}

void Regard3DComputeMatchesDialog::OnOKButtonClick(wxCommandEvent& event)
{
	if(Validate() && TransferDataFromWindow())
	{
		keyPointDetectorType_ = pKeypointDetectorRadioBox_->GetSelection();
		addTBMR_ = pAddTBMRDetectorCheckBox_->GetValue();
		cameraModel_ = pCameraModelChoice_->GetCurrentSelection() + 1;
		EndModal(wxID_OK);
	}
}


// TODO: Put sliderValue/textStr/textvalStr triplets into vector
void Regard3DComputeMatchesDialog::updateICKeypointSensitivityText()
{
	int sliderValue = pKeypointSensitivitySlider_->GetValue();

	wxString textStr, textValStr;
	if(sliderValue == 0)
	{
		textStr = wxT("Minimal");
		keypointSensitivity_ = 0.001;
		textValStr = wxT("0.001");
	}
	else if(sliderValue == 1)
	{
		textStr = wxT("Normal");
		keypointSensitivity_ = 0.0007;
		textValStr = wxT("0.0007");
	}
	else if(sliderValue == 2)
	{
		textStr = wxT("High");
		keypointSensitivity_ = 0.0005;
		textValStr = wxT("0.0005");
	}
	else if(sliderValue == 3)
	{
		textStr = wxT("Ultra");
		keypointSensitivity_ = 0.0001;
		textValStr = wxT("0.0001");
	}

	pKeypointSensitivityTextCtrl_->SetValue(textStr);
	pKeypointSensitivityValTextCtrl_->SetValue(textValStr);
}

void Regard3DComputeMatchesDialog::updateICKeypointMatchingRatioText()
{
	int sliderValue = pKeypointMatchingRatioSlider_->GetValue();
	wxString textStr, textValStr;
	if(sliderValue == 0)
	{
		textStr = wxT("Normal");
		keypointMatchingRatio_ = 0.6;
		textValStr = wxT("0.6");
	}
	else if(sliderValue == 1)
	{
		textStr = wxT("Higher");
		keypointMatchingRatio_ = 0.7;
		textValStr = wxT("0.7");
	}
	else if(sliderValue == 2)
	{
		textStr = wxT("High");
		keypointMatchingRatio_ = 0.8;
		textValStr = wxT("0.8");
	}
	else if(sliderValue == 3)
	{
		textStr = wxT("Ultra");
		keypointMatchingRatio_ = 0.9;
		textValStr = wxT("0.9");
	}

	pKeypointMatchingRatioTextCtrl_->SetValue(textStr);
	pKeypointMatchingRatioValTextCtrl_->SetValue(textValStr);
}

BEGIN_EVENT_TABLE( Regard3DComputeMatchesDialog, Regard3DComputeMatchesDialogBase )
END_EVENT_TABLE()
