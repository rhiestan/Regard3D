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
#include "Regard3DSurfaceDialog.h"


Regard3DSurfaceDialog::Regard3DSurfaceDialog(wxWindow *pParent)
	: Regard3DSurfaceDialogBase(pParent),
	enablePoisson_(true), enableFSSR_(false),
	fssrRefineOctreeLevels_(0),
	fssrScaleFactorMultiplier_(1.0f),
	fssrConfidenceThreshold_(1.0f),
	fssrMinComponentSize_(1000)
{
}

Regard3DSurfaceDialog::~Regard3DSurfaceDialog()
{
}

void Regard3DSurfaceDialog::setParams(R3DProject::Densification *pDensification)
{
	if(pDensification != NULL)
	{
		if(pDensification->densificationType_ == R3DProject::DTCMVSPMVS)
		{
			enablePoisson_ = true;
			enableFSSR_ = false;
		}
		else if(pDensification->densificationType_ == R3DProject::DTMVE
			|| pDensification->densificationType_ == R3DProject::DTSMVS)
		{
			enablePoisson_ = true;
			enableFSSR_ = true;
		}
	}
}

void Regard3DSurfaceDialog::getResults(R3DProject::Surface *pSurface)
{
	if(pSurfaceGenerationMethodRadioBox_->GetSelection() == 0)
		pSurface->surfaceType_ = R3DProject::STPoissonRecon;
	else
		pSurface->surfaceType_ = R3DProject::STFSSRecon;

	pSurface->poissonDepth_ = pPoissonDepthSlider_->GetValue();
	pSurface->poissonSamplesPerNode_ = static_cast<float>(pPoissonSamplesPerNodeSlider_->GetValue())/10.0f;
	pSurface->poissonPointWeight_ = static_cast<float>(pPoissonPointWeightSlider_->GetValue())/10.0f;
	pSurface->poissonTrimThreshold_ = static_cast<float>(pPoissonTrimThresholdSlider_->GetValue())/10.0f;

	pSurface->fssrRefineOctreeLevels_ = fssrRefineOctreeLevels_;
	pSurface->fssrScaleFactorMultiplier_ = fssrScaleFactorMultiplier_;
	pSurface->fssrConfidenceThreshold_ = fssrConfidenceThreshold_;
	pSurface->fssrMinComponentSize_ = fssrMinComponentSize_;

	if(pColorizationMethodRadioBox_->GetSelection() == 0)
		pSurface->colorizationType_ = R3DProject::CTColoredVertices;
	else
		pSurface->colorizationType_ = R3DProject::CTTextures;
	pSurface->colVertNumNeighbours_ = pColVertNumberOfNeighboursSlider_->GetValue();
	pSurface->textOutlierRemovalType_ = pTextOutlierRemovalChoice_->GetSelection();
	pSurface->textGeometricVisibilityTest_ = pTextGeomVisTestCheckBox_->GetValue();
	pSurface->textGlobalSeamLeveling_ = pTextGlobalSeamLevCheckBox_->GetValue();
	pSurface->textLocalSeamLeveling_ = pTextLocalSeamLevCheckBox_->GetValue();
}

void Regard3DSurfaceDialog::OnInitDialog( wxInitDialogEvent& event )
{
	wxDialog::OnInitDialog(event);

	if(enablePoisson_)
		pSurfaceGenerationMethodRadioBox_->Select(0);
	else
		pSurfaceGenerationMethodRadioBox_->Enable(0, false);

	if(enableFSSR_)
		pSurfaceGenerationMethodRadioBox_->Select(1);
	else
		pSurfaceGenerationMethodRadioBox_->Enable(1, false);

	enableSurfaceGenWidgets();
	enableColorizationWidgets();

	pColorizationMethodRadioBox_->Select(0);

	updatePoissonDepthText();
	updateSamplesPerNodeText();
	updatePoissonPointWeightText();
	updatePoissonTrimThresholdText();
	updateFSSRRefineOctreeLevelsText();
	updateFSSRScaleFactorMultiplierText();
	updateFSSRConfidenceThresholdText();
	updateFSSRMinComponentSizeText();
	updateColVertNumberOfNeighboursText();

	Fit();
	CenterOnParent();
}

void Regard3DSurfaceDialog::OnSurfaceGenerationMethodRadioBox( wxCommandEvent& event )
{
	enableSurfaceGenWidgets();
}

void Regard3DSurfaceDialog::OnPoissonDepthSliderScroll( wxScrollEvent& event )
{
	updatePoissonDepthText();
}

void Regard3DSurfaceDialog::OnPoissonSamplesPerNodeSliderScroll( wxScrollEvent& event )
{
	updateSamplesPerNodeText();
}

void Regard3DSurfaceDialog::OnPoissonPointWeightSliderScroll( wxScrollEvent& event )
{
	updatePoissonPointWeightText();
}

void Regard3DSurfaceDialog::OnPoissonTrimThresholdSliderScroll( wxScrollEvent& event )
{
	updatePoissonTrimThresholdText();
}

void Regard3DSurfaceDialog::OnFSSRRefineOctreeLevelsSliderScroll( wxScrollEvent& event )
{
	updateFSSRRefineOctreeLevelsText();
}

void Regard3DSurfaceDialog::OnFSSRScaleFactorMultiplierSliderScroll( wxScrollEvent& event )
{
	updateFSSRScaleFactorMultiplierText();
}

void Regard3DSurfaceDialog::OnFSSRConfidenceThresholdSliderScroll( wxScrollEvent& event )
{
	updateFSSRConfidenceThresholdText();
}

void Regard3DSurfaceDialog::OnFSSRMinComponentSizeSliderScroll( wxScrollEvent& event )
{
	updateFSSRMinComponentSizeText();
}

void Regard3DSurfaceDialog::OnColorizationMethodRadioBox( wxCommandEvent& event )
{
	enableColorizationWidgets();
}

void Regard3DSurfaceDialog::OnColVertNumberOfNeighboursSliderScroll( wxScrollEvent& event )
{
	updateColVertNumberOfNeighboursText();
}

void Regard3DSurfaceDialog::enableSurfaceGenWidgets()
{
	bool isPoisson = (pSurfaceGenerationMethodRadioBox_->GetSelection() == 0);
	pPoissonDepthTextCtrl_->Enable(isPoisson);
	pPoissonDepthSlider_->Enable(isPoisson);
	pPoissonSamplesPerNodeTextCtrl_->Enable(isPoisson);
	pPoissonSamplesPerNodeSlider_->Enable(isPoisson);
	pPoissonPointWeightTextCtrl_->Enable(isPoisson);
	pPoissonPointWeightSlider_->Enable(isPoisson);
	pPoissonTrimThresholdTextCtrl_->Enable(isPoisson);
	pPoissonTrimThresholdSlider_->Enable(isPoisson);
	pFSSRRefineOctreeLevelsTextCtrl_->Enable(!isPoisson);
	pFSSRRefineOctreeLevelsSlider_->Enable(!isPoisson);
	pFSSRScaleFactorMultiplierTextCtrl_->Enable(!isPoisson);
	pFSSRScaleFactorMultiplierSlider_->Enable(!isPoisson);
	pFSSRConfidenceThresholdTextCtrl_->Enable(!isPoisson);
	pFSSRConfidenceThresholdSlider_->Enable(!isPoisson);
	pFSSRMinComponentSizeTextCtrl_->Enable(!isPoisson);
	pFSSRMinComponentSizeSlider_->Enable(!isPoisson);
}

void Regard3DSurfaceDialog::enableColorizationWidgets()
{
	bool isColVert = (pColorizationMethodRadioBox_->GetSelection() == 0);

	pColVertNumberOfNeighboursTextCtrl_->Enable(isColVert);
	pColVertNumberOfNeighboursSlider_->Enable(isColVert);

	pTextOutlierRemovalChoice_->Enable(!isColVert);
	pTextGeomVisTestCheckBox_->Enable(!isColVert);
	pTextGlobalSeamLevCheckBox_->Enable(!isColVert);
	pTextLocalSeamLevCheckBox_->Enable(!isColVert);
}

void Regard3DSurfaceDialog::updatePoissonDepthText()
{
	int sliderValue = pPoissonDepthSlider_->GetValue();
	pPoissonDepthTextCtrl_->SetValue( wxString::Format( wxT("%d"), sliderValue ) );
}

void Regard3DSurfaceDialog::updateSamplesPerNodeText()
{
	int sliderValue = pPoissonSamplesPerNodeSlider_->GetValue();
	pPoissonSamplesPerNodeTextCtrl_->SetValue( wxString::Format( wxT("%g"),
		static_cast<float>(sliderValue)/10.0f ));
}

void Regard3DSurfaceDialog::updatePoissonPointWeightText()
{
	int sliderValue = pPoissonPointWeightSlider_->GetValue();
	pPoissonPointWeightTextCtrl_->SetValue( wxString::Format( wxT("%g"),
		static_cast<float>(sliderValue)/10.0f ));
}

void Regard3DSurfaceDialog::updatePoissonTrimThresholdText()
{
	int sliderValue = pPoissonTrimThresholdSlider_->GetValue();
	if(sliderValue == 0)
		pPoissonTrimThresholdTextCtrl_->SetValue(wxT("Off"));
	else
		pPoissonTrimThresholdTextCtrl_->SetValue( wxString::Format( wxT("%g"),
			static_cast<float>(sliderValue)/10.0f ));
}

void Regard3DSurfaceDialog::updateFSSRRefineOctreeLevelsText()
{
	int sliderValue = pFSSRRefineOctreeLevelsSlider_->GetValue();
	fssrRefineOctreeLevels_ = sliderValue;
	pFSSRRefineOctreeLevelsTextCtrl_->SetValue( wxString::Format( wxT("%d"),
		fssrRefineOctreeLevels_ ));
}

void Regard3DSurfaceDialog::updateFSSRScaleFactorMultiplierText()
{
	int sliderValue = pFSSRScaleFactorMultiplierSlider_->GetValue();
	// Values between 0.5..10
	fssrScaleFactorMultiplier_ = 0.5f + 9.5f*(static_cast<float>(sliderValue - pFSSRScaleFactorMultiplierSlider_->GetMin()) 
		/ static_cast<float>(pFSSRScaleFactorMultiplierSlider_->GetMax() - pFSSRScaleFactorMultiplierSlider_->GetMin()));
	pFSSRScaleFactorMultiplierTextCtrl_->SetValue( wxString::Format( wxT("%g"),
		fssrScaleFactorMultiplier_ ));
}

void Regard3DSurfaceDialog::updateFSSRConfidenceThresholdText()
{
	int sliderValue = pFSSRConfidenceThresholdSlider_->GetValue();
	// Values between 1.0 and 20.0
	fssrConfidenceThreshold_ = 1.0f + 19.0f*(static_cast<float>(sliderValue - pFSSRConfidenceThresholdSlider_->GetMin()) 
		/ static_cast<float>(pFSSRConfidenceThresholdSlider_->GetMax() - pFSSRConfidenceThresholdSlider_->GetMin()));
	pFSSRConfidenceThresholdTextCtrl_->SetValue( wxString::Format( wxT("%g"),
		fssrConfidenceThreshold_ ));
}

void Regard3DSurfaceDialog::updateFSSRMinComponentSizeText()
{
	int sliderValue = pFSSRMinComponentSizeSlider_->GetValue();
	float sliderValuef = static_cast<float>(sliderValue - pFSSRMinComponentSizeSlider_->GetMin()) 
		/ static_cast<float>(pFSSRMinComponentSizeSlider_->GetMax() - pFSSRMinComponentSizeSlider_->GetMin());
	float expValue = std::exp(sliderValuef);						// Between 1 and e
	float expValue01 = (expValue - 1.0f) / (std::exp(1.0f) - 1.0f);	// Between 0 and 1
	// Values between 1 and 100000, exponential scale
	fssrMinComponentSize_ = 1 + static_cast<int>(9999.0f * expValue01);
	pFSSRMinComponentSizeTextCtrl_->SetValue( wxString::Format( wxT("%d"),
		fssrMinComponentSize_ ));
}

void Regard3DSurfaceDialog::updateColVertNumberOfNeighboursText()
{
	int sliderValue = pColVertNumberOfNeighboursSlider_->GetValue();
	pColVertNumberOfNeighboursTextCtrl_->SetValue( wxString::Format( wxT("%d"), sliderValue ) );
}

BEGIN_EVENT_TABLE( Regard3DSurfaceDialog, Regard3DSurfaceDialogBase )
END_EVENT_TABLE()
