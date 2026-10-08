/**
 * Copyright (C) 2017 Roman Hiestand
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
#include "R3DExternalPrograms.h"

namespace
{
	// Presets shown on the "Max image size" slider; index 0 is COLMAP's
	// "original size" (-1), the others are the longest side in pixels.
	const int colmapMaxImageSizePresets[] = { -1, 3200, 2400, 1600, 1200, 800 };
}

Regard3DDensificationDialog::Regard3DDensificationDialog(wxWindow *pParent)
	: Regard3DDensificationDialogBase(pParent), colmapTriangulation_(false)
{
	maxImage_ = wxString(wxT("100"));
}

Regard3DDensificationDialog::~Regard3DDensificationDialog()
{
}

void Regard3DDensificationDialog::setColmapTriangulation(bool colmapTriangulation)
{
	colmapTriangulation_ = colmapTriangulation;
}

void Regard3DDensificationDialog::getResults(R3DProject::Densification *pDensification)
{
	// By page rather than by index: OnInitDialog removes the pages of
	// methods that cannot run
	const wxWindow *pPage = pDensificationMethodChoicebook_->GetCurrentPage();
	if(pPage == pPMVSParamsPanel_)
		pDensification->densificationType_ = R3DProject::DTCMVSPMVS;
	else if(pPage == pDMReconParamsPanel_)
		pDensification->densificationType_ = R3DProject::DTMVE;
	else if(pPage == pSMVSReconParamsPanel_)
		pDensification->densificationType_ = R3DProject::DTSMVS;
	else if(pPage == pColmapReconParamsPanel_)
		pDensification->densificationType_ = R3DProject::DTCOLMAP;
	else if(pPage == pOpenMVSReconParamsPanel_)
		pDensification->densificationType_ = R3DProject::DTOPENMVS;

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

	pDensification->smvsInputScale_ = pSMVSInputScaleSlider_->GetValue();
	pDensification->smvsOutputScale_ = pSMVSOutputScaleSlider_->GetValue();
	pDensification->smvsEnableShadingBasedOptimization_ = pSMVSShadingOptCheckBox_->GetValue();
	pDensification->smvsEnableSemiGlobalMatching_ = pSMVSSemiGlobalMatcihingCheckBox_->GetValue();
	pDensification->smvsAlpha_ = static_cast<float>(pSMVSSurfaceSmoothingFactorSlider_->GetValue()) * 0.1f;

	pDensification->colmapMaxImageSize_ = colmapMaxImageSizePresets[pColmapMaxImageSizeSlider_->GetValue()];
	pDensification->colmapWindowRadius_ = pColmapWindowRadiusSlider_->GetValue();
	pDensification->colmapGeomConsistency_ = pColmapGeomConsistencyCheckBox_->GetValue();
	pDensification->colmapFilter_ = pColmapFilterCheckBox_->GetValue();
	pDensification->colmapMaxReprojError_ = static_cast<float>(pColmapMaxReprojErrorSlider_->GetValue()) * 0.1f;
	pDensification->colmapUseCuda_ = pColmapUseCudaCheckBox_->GetValue();

	pDensification->openMVSResolutionLevel_ = pOpenMVSResolutionLevelSlider_->GetValue();
	pDensification->openMVSNumberViews_ = pOpenMVSNumberViewsSlider_->GetValue();
	pDensification->openMVSNumberViewsFuse_ = pOpenMVSNumberViewsFuseSlider_->GetValue();
}

void Regard3DDensificationDialog::OnInitDialog( wxInitDialogEvent& event )
{
	wxDialog::OnInitDialog(event);	// Call base class to initalize validators

	// Only offer what can actually run on this triangulation
	wxString toolTip;
	if(colmapTriangulation_)
	{
		removeMethodPage(pPMVSParamsPanel_);
		removeMethodPage(pDMReconParamsPanel_);
		removeMethodPage(pSMVSReconParamsPanel_);
		toolTip = wxT("This triangulation was computed by COLMAP, so only COLMAP and OpenMVS ")
			wxT("can densify it: CMVS/PMVS, MVE and SMVS all need an OpenMVG ")
			wxT("reconstruction, which this triangulation does not have.");
	}
	// OpenMVS is not shipped with Regard3D, see R3DExternalPrograms
	wxString openMVSReason;
	if(!R3DExternalPrograms::getInstance().isOpenMVSDensificationPossible(colmapTriangulation_, openMVSReason))
	{
		removeMethodPage(pOpenMVSReconParamsPanel_);
		if(!toolTip.IsEmpty())
			toolTip += wxT("\n\n");
		toolTip += wxT("OpenMVS densification is not available. ") + openMVSReason
			+ wxT(" (expected in the \"openmvs\" subdirectory of the external tools directory).");
	}
	wxWindow *pChoiceCtrl = pDensificationMethodChoicebook_->GetChoiceCtrl();
	if(pChoiceCtrl != NULL && !toolTip.IsEmpty())
		pChoiceCtrl->SetToolTip(toolTip);
	pDensificationMethodChoicebook_->SetSelection(0);

	pMaxImageTextCtrl_->SetValue(wxT("100"));
	//TransferDataToWindow();

	int maxNumberOfThreads = wxThread::GetCPUCount() + 1;
	for(int i = 1; i <= maxNumberOfThreads; i++)
		pNumberOfThreadsChoice_->Append(wxString::Format(wxT("%d"), i));
	pNumberOfThreadsChoice_->SetSelection(maxNumberOfThreads - 2);

	updatePMVSLevelText();
	updatePMVSCellSizeText();
	updatePMVSThresholdText();
	updatePMVSWSizeText();
	updatePMVSMinImageNumText();
	updateMVEScaleText();
	updateMVEFilterWidthText();
	updateSMVSInputScaleText();
	updateSMVSOutputScaleText();
	updateSMVSSurfaceSmoothingFactorText();
	updateColmapMaxImageSizeText();
	updateColmapWindowRadiusText();
	updateColmapMaxReprojErrorText();
	updateColmapUseCudaCheckBox();
	updateOpenMVSResolutionLevelText();
	updateOpenMVSNumberViewsText();
	updateOpenMVSNumberViewsFuseText();

	Fit();
	CenterOnParent();
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

void Regard3DDensificationDialog::OnSMVSInputScaleSliderScroll(wxScrollEvent& event)
{
	updateSMVSInputScaleText();
}

void Regard3DDensificationDialog::OnSMVSOutputScaleSliderScroll(wxScrollEvent& event)
{
	updateSMVSOutputScaleText();
}

void Regard3DDensificationDialog::OnSMVSSurfaceSmoothingFactorSliderScroll(wxScrollEvent& event)
{
	updateSMVSSurfaceSmoothingFactorText();
}

void Regard3DDensificationDialog::OnColmapMaxImageSizeSliderScroll(wxScrollEvent& event)
{
	updateColmapMaxImageSizeText();
}

void Regard3DDensificationDialog::OnColmapWindowRadiusSliderScroll(wxScrollEvent& event)
{
	updateColmapWindowRadiusText();
}

void Regard3DDensificationDialog::OnColmapMaxReprojErrorSliderScroll(wxScrollEvent& event)
{
	updateColmapMaxReprojErrorText();
}

void Regard3DDensificationDialog::OnOpenMVSResolutionLevelSliderScroll(wxScrollEvent& event)
{
	updateOpenMVSResolutionLevelText();
}

void Regard3DDensificationDialog::OnOpenMVSNumberViewsSliderScroll(wxScrollEvent& event)
{
	updateOpenMVSNumberViewsText();
}

void Regard3DDensificationDialog::OnOpenMVSNumberViewsFuseSliderScroll(wxScrollEvent& event)
{
	updateOpenMVSNumberViewsFuseText();
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

void Regard3DDensificationDialog::updateSMVSInputScaleText()
{
	int sliderValue = pSMVSInputScaleSlider_->GetValue();
	pSMVSInputScaleTextCtrl_->SetValue( wxString::Format( wxT("%d"), sliderValue ) );
}

void Regard3DDensificationDialog::updateSMVSOutputScaleText()
{
	int sliderValue = pSMVSOutputScaleSlider_->GetValue();
	pSMVSOutputScaleTextCtrl_->SetValue( wxString::Format( wxT("%d"), sliderValue ) );
}

void Regard3DDensificationDialog::updateSMVSSurfaceSmoothingFactorText()
{
	int sliderValue = pSMVSSurfaceSmoothingFactorSlider_->GetValue();
	pSMVSSurfaceSmoothingFactorTextCtrl_->SetValue( wxString::Format( wxT("%g"),
		static_cast<float>(sliderValue)*0.1f ) );
}

void Regard3DDensificationDialog::updateColmapMaxImageSizeText()
{
	int pixelSize = colmapMaxImageSizePresets[pColmapMaxImageSizeSlider_->GetValue()];
	pColmapMaxImageSizeTextCtrl_->SetValue( pixelSize > 0
		? wxString::Format( wxT("%d"), pixelSize ) : wxString(wxT("Original")) );
}

void Regard3DDensificationDialog::updateColmapWindowRadiusText()
{
	int sliderValue = pColmapWindowRadiusSlider_->GetValue();
	pColmapWindowRadiusTextCtrl_->SetValue( wxString::Format( wxT("%d"), sliderValue ) );
}

void Regard3DDensificationDialog::updateColmapMaxReprojErrorText()
{
	int sliderValue = pColmapMaxReprojErrorSlider_->GetValue();
	pColmapMaxReprojErrorTextCtrl_->SetValue( wxString::Format( wxT("%g"),
		static_cast<float>(sliderValue)*0.1f ) );
}

// COLMAP's dense stereo (patch_match_stereo) has no CPU implementation - it
// requires an NVIDIA/CUDA GPU and the colmap_cuda build unconditionally, even
// a "without GPU support" build of COLMAP just fails at runtime rather than
// falling back to the CPU. So there is nothing to actually choose here: this
// checkbox is a disabled status indicator, always reflecting whether COLMAP
// densification can run at all (colmap_cuda installed, ideally with a CUDA
// driver detected too - see R3DExternalPrograms::hasCudaDriver()).
void Regard3DDensificationDialog::updateColmapUseCudaCheckBox()
{
	R3DExternalPrograms &extPrograms = R3DExternalPrograms::getInstance();
	const bool hasCudaBuild = !extPrograms.getColmapCudaPath().IsEmpty();

	pColmapUseCudaCheckBox_->SetValue(hasCudaBuild && extPrograms.hasCudaDriver());
	pColmapUseCudaCheckBox_->Enable(false);
}

void Regard3DDensificationDialog::updateOpenMVSResolutionLevelText()
{
	// Each level halves the images
	int sliderValue = pOpenMVSResolutionLevelSlider_->GetValue();
	pOpenMVSResolutionLevelTextCtrl_->SetValue( sliderValue > 0
		? wxString::Format( wxT("%d (1/%d size)"), sliderValue, 1 << sliderValue )
		: wxString(wxT("0 (full size)")) );
}

void Regard3DDensificationDialog::updateOpenMVSNumberViewsText()
{
	int sliderValue = pOpenMVSNumberViewsSlider_->GetValue();
	pOpenMVSNumberViewsTextCtrl_->SetValue( sliderValue > 0
		? wxString::Format( wxT("%d"), sliderValue ) : wxString(wxT("All")) );
}

void Regard3DDensificationDialog::updateOpenMVSNumberViewsFuseText()
{
	int sliderValue = pOpenMVSNumberViewsFuseSlider_->GetValue();
	pOpenMVSNumberViewsFuseTextCtrl_->SetValue( wxString::Format( wxT("%d"), sliderValue ) );
}

void Regard3DDensificationDialog::removeMethodPage(wxWindow *pPage)
{
	int pageIndex = pDensificationMethodChoicebook_->FindPage(pPage);
	if(pageIndex == wxNOT_FOUND)
		return;

	// RemovePage, not DeletePage: the page stays a child of the choicebook and
	// is destroyed with the dialog, it just isn't shown or selectable any more
	pDensificationMethodChoicebook_->RemovePage(static_cast<size_t>(pageIndex));
	pPage->Hide();
}

BEGIN_EVENT_TABLE( Regard3DDensificationDialog, Regard3DDensificationDialogBase )
END_EVENT_TABLE()
