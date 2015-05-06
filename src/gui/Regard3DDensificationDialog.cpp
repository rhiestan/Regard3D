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
#include "Regard3DDensificationDialog.h"

Regard3DDensificationDialog::Regard3DDensificationDialog(wxWindow *pParent)
	: Regard3DDensificationDialogBase(pParent)
{
	maxImage_ = wxString(wxT("100"));
}

Regard3DDensificationDialog::~Regard3DDensificationDialog()
{
}

void Regard3DDensificationDialog::getResults(R3DProject::Densification *pDensification)
{
	if(pDensificationMethodRadioBox_->GetSelection() == 0)
		pDensification->densificationType_ = R3DProject::DTCMVSPMVS;
	else if(pDensificationMethodRadioBox_->GetSelection() == 1)
		pDensification->densificationType_ = R3DProject::DTMVE;

	pDensification->pmvsNumThreads_ = pNumberOfThreadsChoice_->GetSelection() + 1;
	pDensification->useCMVS_ = pUseCMVSCheckBox_->GetValue();

	long maxClusterSize = 0;
	pMaxImageTextCtrl_->GetValue().ToLong(&maxClusterSize);
	pDensification->pmvsMaxClusterSize_ = maxClusterSize;

	pDensification->pmvsLevel_ = pPMVSLevelSlider_->GetValue();
	pDensification->pmvsCSize_ = pPMVSCellSizeSlider_->GetValue();
	pDensification->pmvsThreshold_ = static_cast<float>(pPMVSThresholdSlider_->GetValue())*0.01f;
	pDensification->pmvsWSize_ = pPMVSWSizeSlider_->GetValue();
	pDensification->pmvsMinImageNum_ = pPMVSMinImageNumSlider_->GetValue();

	pDensification->mveScale_ = pMVEScaleSlider_->GetValue();
	pDensification->mveFilterWidth_ = (pMVEFilterWidthSlider_->GetValue()) * 2 + 1;
}

void Regard3DDensificationDialog::OnInitDialog( wxInitDialogEvent& event )
{
	wxDialog::OnInitDialog(event);	// Call base class to initalize validators

	pDensificationMethodRadioBox_->Select(0);
	//pDensificationMethodRadioBox_->Enable(1, false);

	pMaxImageTextCtrl_->SetValue(wxT("100"));
	//TransferDataToWindow();

	int maxNumberOfThreads = wxThread::GetCPUCount() + 1;
	for(int i = 1; i <= maxNumberOfThreads; i++)
		pNumberOfThreadsChoice_->Append(wxString::Format(wxT("%d"), i));
	pNumberOfThreadsChoice_->SetSelection(maxNumberOfThreads - 2);

	enableDensificationWidgets();
	updatePMVSLevelText();
	updatePMVSCellSizeText();
	updatePMVSThresholdText();
	updatePMVSWSizeText();
	updatePMVSMinImageNumText();
	updateMVEScaleText();
	updateMVEFilterWidthText();

	Fit();
	CenterOnParent();
}

void Regard3DDensificationDialog::OnDensificationMethodRadioBox( wxCommandEvent& event )
{
	enableDensificationWidgets();
}

void Regard3DDensificationDialog::OnUseCMVSCheckBox( wxCommandEvent& event )
{
	pMaxImageTextCtrl_->Enable(event.IsChecked());
}

void Regard3DDensificationDialog::OnPMVSLevelSliderScroll( wxScrollEvent& event )
{
	updatePMVSLevelText();
}

void Regard3DDensificationDialog::OnPMVSCellSizeSliderScroll( wxScrollEvent& event )
{
	updatePMVSCellSizeText();
}

void Regard3DDensificationDialog::OnPMVSThresholdSliderScroll( wxScrollEvent& event )
{
	updatePMVSThresholdText();
}

void Regard3DDensificationDialog::OnPMVSWSizeSliderScroll( wxScrollEvent& event )
{
	updatePMVSWSizeText();
}

void Regard3DDensificationDialog::OnPMVSMinImageNumSliderScroll( wxScrollEvent& event )
{
	updatePMVSMinImageNumText();
}

void Regard3DDensificationDialog::OnMVEScaleSliderScroll( wxScrollEvent& event )
{
	updateMVEScaleText();
}

void Regard3DDensificationDialog::OnMVEFilterWidthSliderScroll( wxScrollEvent& event )
{
	updateMVEFilterWidthText();
}

void Regard3DDensificationDialog::enableDensificationWidgets()
{
	bool isMVE = (pDensificationMethodRadioBox_->GetSelection() == 1);

	pNumberOfThreadsChoice_->Enable(!isMVE);
	pUseCMVSCheckBox_->Enable(!isMVE);
	pMaxImageTextCtrl_->Enable(!isMVE);
	pPMVSLevelTextCtrl_->Enable(!isMVE);
	pPMVSLevelSlider_->Enable(!isMVE);
	pPMVSCellSizeTextCtrl_->Enable(!isMVE);
	pPMVSCellSizeSlider_->Enable(!isMVE);
	pPMVSThresholdTextCtrl_->Enable(!isMVE);
	pPMVSThresholdSlider_->Enable(!isMVE);
	pPMVSWSizeTextCtrl_->Enable(!isMVE);
	pPMVSWSizeSlider_->Enable(!isMVE);
	pPMVSMinImageNumTextCtrl_->Enable(!isMVE);
	pPMVSMinImageNumSlider_->Enable(!isMVE);

	pMVEScaleTextCtrl_->Enable(isMVE);
	pMVEScaleSlider_->Enable(isMVE);
	pMVEFilterWidthTextCtrl_->Enable(isMVE);
	pMVEFilterWidthSlider_->Enable(isMVE);
}

void Regard3DDensificationDialog::updatePMVSLevelText()
{
	int sliderValue = pPMVSLevelSlider_->GetValue();
	pPMVSLevelTextCtrl_->SetValue( wxString::Format( wxT("%d"), sliderValue ) );
}

void Regard3DDensificationDialog::updatePMVSCellSizeText()
{
	int sliderValue = pPMVSCellSizeSlider_->GetValue();
	pPMVSCellSizeTextCtrl_->SetValue( wxString::Format( wxT("%d"), sliderValue ) );
}

void Regard3DDensificationDialog::updatePMVSThresholdText()
{
	int sliderValue = pPMVSThresholdSlider_->GetValue();
	pPMVSThresholdTextCtrl_->SetValue( wxString::Format( wxT("%g"),
		static_cast<float>(sliderValue)*0.01f ) );
}

void Regard3DDensificationDialog::updatePMVSWSizeText()
{
	int sliderValue = pPMVSWSizeSlider_->GetValue();
	pPMVSWSizeTextCtrl_->SetValue( wxString::Format( wxT("%d"), sliderValue ) );
}

void Regard3DDensificationDialog::updatePMVSMinImageNumText()
{
	int sliderValue = pPMVSMinImageNumSlider_->GetValue();
	pPMVSMinImageNumTextCtrl_->SetValue( wxString::Format( wxT("%d"), sliderValue ) );
}

void Regard3DDensificationDialog::updateMVEScaleText()
{
	int sliderValue = pMVEScaleSlider_->GetValue();
	pMVEScaleTextCtrl_->SetValue( wxString::Format( wxT("%d"), sliderValue ) );
}

void Regard3DDensificationDialog::updateMVEFilterWidthText()
{
	int sliderValue = pMVEFilterWidthSlider_->GetValue();
	pMVEFilterWidthTextCtrl_->SetValue( wxString::Format( wxT("%d"), sliderValue*2 + 1 ) );
}

BEGIN_EVENT_TABLE( Regard3DDensificationDialog, Regard3DDensificationDialogBase )
END_EVENT_TABLE()
