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
	maxPixelCount_(0)
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
	int &nrOfThreads)
{
	keypointSensitivity = keypointSensitivity_;
	keypointMatchingRatio = keypointMatchingRatio_;
	nrOfThreads = pNumberOfThreadsChoice_->GetSelection() + 1;
}

void Regard3DComputeMatchesDialog::OnInitDialog( wxInitDialogEvent& event )
{
	pKeypointSensitivitySlider_->SetValue(1);
	updateICKeypointSensitivityText();
	pKeypointMatchingRatioSlider_->SetValue(0);
	updateICKeypointMatchingRatioText();

	int maxNumberOfThreads = wxThread::GetCPUCount() + 1;
	for(int i = 1; i <= maxNumberOfThreads; i++)
		pNumberOfThreadsChoice_->Append(wxString::Format(wxT("%d"), i));
	pNumberOfThreadsChoice_->SetSelection(0);

	updateNumberOfThreadsText();

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

void Regard3DComputeMatchesDialog::OnNumberOfThreadsChoice( wxCommandEvent& event )
{
	updateNumberOfThreadsText();
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

void Regard3DComputeMatchesDialog::updateNumberOfThreadsText()
{
	// Calculate memory
	int numThreads = pNumberOfThreadsChoice_->GetSelection() + 1;

	// A-KAZE requires roughly 225MB per Megapixel
	wxULongLong memSize = 260000000L + 225L * static_cast<long long>(numThreads) * maxPixelCount_;
	wxString memSizeStr = wxFileName::GetHumanReadableSize(memSize, wxEmptyString, 1);
	wxString infoString = wxString(wxT("Estimated memory consumption: ")) + memSizeStr;
	pKeypointSuggestedNumThreadsStaticText_->SetLabel(infoString);
	pComputeMatchesDialogPanel_->Layout();
}

BEGIN_EVENT_TABLE( Regard3DComputeMatchesDialog, Regard3DComputeMatchesDialogBase )
END_EVENT_TABLE()
