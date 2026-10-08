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
#include "R3DExternalPrograms.h"


Regard3DSurfaceDialog::Regard3DSurfaceDialog(wxWindow *pParent)
	: Regard3DSurfaceDialogBase(pParent),
	enablePoisson_(true), enableFSSR_(false), enableOpenMVS_(false),
	colmapTriangulation_(false), lastSurfaceMethod_(-1),
	fssrRefineOctreeLevels_(0),
	fssrScaleFactorMultiplier_(1.0f),
	fssrConfidenceThreshold_(1.0f),
	fssrMinComponentSize_(1000)
{
}

Regard3DSurfaceDialog::~Regard3DSurfaceDialog()
{
}

void Regard3DSurfaceDialog::setParams(R3DProject::Densification *pDensification, bool colmapTriangulation)
{
	colmapTriangulation_ = colmapTriangulation;
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
		else if(pDensification->densificationType_ == R3DProject::DTOPENMVS)
		{
			// ReconstructMesh needs the scene_dense.mvs only DensifyPointCloud
			// writes; the point cloud also has normals, so Poisson works too
			enablePoisson_ = true;
			enableFSSR_ = false;
			enableOpenMVS_ = !R3DExternalPrograms::getInstance().getReconstructMeshPath().IsEmpty();
			if(!enableOpenMVS_)
				openMVSReason_ = wxT("ReconstructMesh (OpenMVS) was not found in the \"openmvs\" ")
					wxT("subdirectory of the external tools directory.");
		}
	}
	if(pDensification == NULL || pDensification->densificationType_ != R3DProject::DTOPENMVS)
		openMVSReason_ = wxT("OpenMVS can only mesh a point cloud that OpenMVS densified.");
}

void Regard3DSurfaceDialog::getResults(R3DProject::Surface *pSurface)
{
	if(pSurfaceGenerationMethodRadioBox_->GetSelection() == 0)
		pSurface->surfaceType_ = R3DProject::STPoissonRecon;
	else if(pSurfaceGenerationMethodRadioBox_->GetSelection() == 1)
		pSurface->surfaceType_ = R3DProject::STFSSRecon;
	else
		pSurface->surfaceType_ = R3DProject::STOpenMVS;

	pSurface->poissonDepth_ = pPoissonDepthSlider_->GetValue();
	pSurface->poissonSamplesPerNode_ = static_cast<float>(pPoissonSamplesPerNodeSlider_->GetValue())/10.0f;
	pSurface->poissonPointWeight_ = static_cast<float>(pPoissonPointWeightSlider_->GetValue())/10.0f;
	pSurface->poissonTrimThreshold_ = static_cast<float>(pPoissonTrimThresholdSlider_->GetValue())/10.0f;

	pSurface->fssrRefineOctreeLevels_ = fssrRefineOctreeLevels_;
	pSurface->fssrScaleFactorMultiplier_ = fssrScaleFactorMultiplier_;
	pSurface->fssrConfidenceThreshold_ = fssrConfidenceThreshold_;
	pSurface->fssrMinComponentSize_ = fssrMinComponentSize_;

	pSurface->openMVSMinPointDistance_ = static_cast<float>(pOpenMVSMinPointDistanceSlider_->GetValue())/10.0f;
	pSurface->openMVSSmoothIterations_ = pOpenMVSSmoothIterationsSlider_->GetValue();
	pSurface->openMVSRefineMesh_ = pOpenMVSRefineMeshCheckBox_->GetValue()
		&& pOpenMVSRefineMeshCheckBox_->IsEnabled();

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

	if(enableOpenMVS_)
		pSurfaceGenerationMethodRadioBox_->Select(2);
	else
	{
		pSurfaceGenerationMethodRadioBox_->Enable(2, false);
		pSurfaceGenerationMethodRadioBox_->SetItemToolTip(2, openMVSReason_);
	}

	pColorizationMethodRadioBox_->Select(0);

	enableSurfaceGenWidgets();		// Calls enableColorizationWidgets()

	updatePoissonDepthText();
	updateSamplesPerNodeText();
	updatePoissonPointWeightText();
	updatePoissonTrimThresholdText();
	updateFSSRRefineOctreeLevelsText();
	updateFSSRScaleFactorMultiplierText();
	updateFSSRConfidenceThresholdText();
	updateFSSRMinComponentSizeText();
	updateColVertNumberOfNeighboursText();
	updateOpenMVSMinPointDistanceText();
	updateOpenMVSSmoothIterationsText();

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

void Regard3DSurfaceDialog::OnOpenMVSMinPointDistanceSliderScroll( wxScrollEvent& event )
{
	updateOpenMVSMinPointDistanceText();
}

void Regard3DSurfaceDialog::OnOpenMVSSmoothIterationsSliderScroll( wxScrollEvent& event )
{
	updateOpenMVSSmoothIterationsText();
}

/**
 * Whether the selected surface generation method can be textured.
 *
 * OpenMVS textures with TextureMesh from its own scene; the others with
 * texrecon from an MVE scene exported from sfm_data.bin, which a
 * COLMAP-native triangulation does not have.
 */
bool Regard3DSurfaceDialog::isTexturingPossible(wxString &reason)
{
	R3DExternalPrograms &extPrograms = R3DExternalPrograms::getInstance();
	if(pSurfaceGenerationMethodRadioBox_->GetSelection() == 2)
	{
		if(extPrograms.getTextureMeshPath().IsEmpty())
		{
			reason = wxT("TextureMesh (OpenMVS) was not found in the \"openmvs\" ")
				wxT("subdirectory of the external tools directory.");
			return false;
		}
		return true;
	}

	if(colmapTriangulation_)
	{
		reason = wxT("texrecon needs an OpenMVG reconstruction, and this triangulation was computed ")
			wxT("by COLMAP. Densify it with OpenMVS to get textures.");
		return false;
	}
	return true;
}

void Regard3DSurfaceDialog::enableSurfaceGenWidgets()
{
	bool isPoisson = (pSurfaceGenerationMethodRadioBox_->GetSelection() == 0);
	bool isFSSR = (pSurfaceGenerationMethodRadioBox_->GetSelection() == 1);
	bool isOpenMVS = (pSurfaceGenerationMethodRadioBox_->GetSelection() == 2);
	pPoissonDepthTextCtrl_->Enable(isPoisson);
	pPoissonDepthSlider_->Enable(isPoisson);
	pPoissonSamplesPerNodeTextCtrl_->Enable(isPoisson);
	pPoissonSamplesPerNodeSlider_->Enable(isPoisson);
	pPoissonPointWeightTextCtrl_->Enable(isPoisson);
	pPoissonPointWeightSlider_->Enable(isPoisson);
	pPoissonTrimThresholdTextCtrl_->Enable(isPoisson);
	pPoissonTrimThresholdSlider_->Enable(isPoisson);
	pFSSRRefineOctreeLevelsTextCtrl_->Enable(isFSSR);
	pFSSRRefineOctreeLevelsSlider_->Enable(isFSSR);
	pFSSRScaleFactorMultiplierTextCtrl_->Enable(isFSSR);
	pFSSRScaleFactorMultiplierSlider_->Enable(isFSSR);
	pFSSRConfidenceThresholdTextCtrl_->Enable(isFSSR);
	pFSSRConfidenceThresholdSlider_->Enable(isFSSR);
	pFSSRMinComponentSizeTextCtrl_->Enable(isFSSR);
	pFSSRMinComponentSizeSlider_->Enable(isFSSR);
	pOpenMVSMinPointDistanceTextCtrl_->Enable(isOpenMVS);
	pOpenMVSMinPointDistanceSlider_->Enable(isOpenMVS);
	pOpenMVSSmoothIterationsTextCtrl_->Enable(isOpenMVS);
	pOpenMVSSmoothIterationsSlider_->Enable(isOpenMVS);
	const bool hasRefineMesh = !R3DExternalPrograms::getInstance().getRefineMeshPath().IsEmpty();
	pOpenMVSRefineMeshCheckBox_->Enable(isOpenMVS && hasRefineMesh);
	if(!hasRefineMesh)
		pOpenMVSRefineMeshCheckBox_->SetToolTip(wxT("RefineMesh (OpenMVS) was not found in the ")
			wxT("\"openmvs\" subdirectory of the external tools directory."));

	// Which texturing tool would run depends on the method
	wxString texturingReason;
	const bool texturingPossible = isTexturingPossible(texturingReason);
	if(!texturingPossible && pColorizationMethodRadioBox_->GetSelection() == 1)
		pColorizationMethodRadioBox_->Select(0);
	pColorizationMethodRadioBox_->Enable(1, texturingPossible);
	pColorizationMethodRadioBox_->SetItemToolTip(1, texturingReason);

	// TextureMesh's seam leveling (global and local alike) turns large parts
	// of the texture atlas black in OpenMVS 2.4.0 - reproduced on several
	// data sets, the same mesh textures cleanly without it. So it starts out
	// switched off for OpenMVS, but stays selectable for builds without the
	// problem. Only on switching method, so the user's own choice sticks.
	const int surfaceMethod = pSurfaceGenerationMethodRadioBox_->GetSelection();
	if(surfaceMethod != lastSurfaceMethod_ && (isOpenMVS || lastSurfaceMethod_ == 2))
	{
		const wxString seamToolTip(isOpenMVS
			? wxT("Off by default for OpenMVS: in OpenMVS 2.4.0, TextureMesh's seam leveling ")
				wxT("turns large parts of the texture black.")
			: wxT(""));
		pTextGlobalSeamLevCheckBox_->SetValue(!isOpenMVS);
		pTextLocalSeamLevCheckBox_->SetValue(!isOpenMVS);
		pTextGlobalSeamLevCheckBox_->SetToolTip(seamToolTip);
		pTextLocalSeamLevCheckBox_->SetToolTip(seamToolTip);
	}
	lastSurfaceMethod_ = surfaceMethod;

	enableColorizationWidgets();
}

void Regard3DSurfaceDialog::enableColorizationWidgets()
{
	bool isColVert = (pColorizationMethodRadioBox_->GetSelection() == 0);

	pColVertNumberOfNeighboursTextCtrl_->Enable(isColVert);
	pColVertNumberOfNeighboursSlider_->Enable(isColVert);

	// texrecon only, TextureMesh has neither
	const bool isOpenMVS = (pSurfaceGenerationMethodRadioBox_->GetSelection() == 2);
	pTextOutlierRemovalChoice_->Enable(!isColVert && !isOpenMVS);
	pTextGeomVisTestCheckBox_->Enable(!isColVert && !isOpenMVS);
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

void Regard3DSurfaceDialog::updateOpenMVSMinPointDistanceText()
{
	int sliderValue = pOpenMVSMinPointDistanceSlider_->GetValue();
	pOpenMVSMinPointDistanceTextCtrl_->SetValue( sliderValue > 0
		? wxString::Format( wxT("%g px"), static_cast<float>(sliderValue)/10.0f )
		: wxString(wxT("Off")) );
}

void Regard3DSurfaceDialog::updateOpenMVSSmoothIterationsText()
{
	int sliderValue = pOpenMVSSmoothIterationsSlider_->GetValue();
	pOpenMVSSmoothIterationsTextCtrl_->SetValue( sliderValue > 0
		? wxString::Format( wxT("%d"), sliderValue ) : wxString(wxT("Off")) );
}

BEGIN_EVENT_TABLE( Regard3DSurfaceDialog, Regard3DSurfaceDialogBase )
END_EVENT_TABLE()
