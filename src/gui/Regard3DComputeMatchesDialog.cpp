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
#include "R3DOpenMVGOptions.h"
#include "R3DExternalPrograms.h"

namespace
{
	// The choices on the OpenMVG page are the ones the executables accept, in
	// the order R3DOpenMVGOptions lists them. Keep the .fbp and that header in
	// step: R3DOpenMVGMatchingParams stores indices into those tables.
	using R3DOpenMVGOptions::kMatchers;
	using R3DOpenMVGOptions::kMatcherCount;

	// The executables every OpenMVG run needs, whatever the describer
	bool openMVGCoreToolsPresent()
	{
		R3DExternalPrograms &progs = R3DExternalPrograms::getInstance();
		return !progs.getComputeFeaturesPath().IsEmpty()
			&& !progs.getPairGeneratorPath().IsEmpty()
			&& !progs.getComputeMatchesPath().IsEmpty()
			&& !progs.getGeometricFilterPath().IsEmpty();
	}

	// Selects by index, falling back to the first entry: a project written by a
	// newer version may hold a value this build does not have.
	void setChoiceSelection(wxChoice *pChoice, int index)
	{
		if(index < 0 || index >= static_cast<int>(pChoice->GetCount()))
			index = 0;
		pChoice->SetSelection(index);
	}
}

Regard3DComputeMatchesDialog::Regard3DComputeMatchesDialog(wxWindow *pParent)
	: Regard3DComputeMatchesDialogBase(pParent),
	keypointSensitivity_(0.0007f), keypointMatchingRatio_(0.6f),
	maxPixelCount_(0), 	keyPointDetectorType_(0),
	addTBMR_(false), cameraModel_(3), matchingAlgorithm_(0)
{
}

Regard3DComputeMatchesDialog::~Regard3DComputeMatchesDialog()
{
}

void Regard3DComputeMatchesDialog::setMaxPixelCount(long long maxPixelCount)
{
	maxPixelCount_ = maxPixelCount;
}

void Regard3DComputeMatchesDialog::getResults(R3DComputeMatchesDialogResults &results)
{
	results = results_;
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
	pMatchingAlgorithmChoice_->SetSelection(matchingAlgorithm_);
	pCameraModelChoice_->SetSelection(cameraModel_ - 1);

	initializeOpenMVGPage();

	// OpenMVG is the default, but only where its executables are installed
	if(results_.engine_ == 1 && !openMVGCoreToolsPresent())
		results_.engine_ = 0;

	// Page index is the engine id, so the page is never removed: when the
	// executables are missing the user is told on OK instead. Removing it
	// would shift the indices, which is the trap the triangulation dialog
	// works around in its getResults().
	pMatchingEngineChoicebook_->SetSelection(results_.engine_);

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
		matchingAlgorithm_ = pMatchingAlgorithmChoice_->GetCurrentSelection();

		results_.engine_ = pMatchingEngineChoicebook_->GetSelection();
		results_.keypointSensitivity_ = keypointSensitivity_;
		results_.keypointMatchingRatio_ = keypointMatchingRatio_;
		results_.keypointDetectorType_ = keyPointDetectorType_;
		results_.addTBMR_ = addTBMR_;
		results_.matchingAlgorithm_ = matchingAlgorithm_;
		results_.cameraModel_ = cameraModel_;

		if(results_.engine_ == 1)
		{
			if(!checkOpenMVGExecutables())
				return;

			if(!readOpenMVGPage())
				return;
		}

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

/**
 * Puts the OpenMVG defaults into the page and hooks up the two choices whose
 * selection changes what the rest of the page may contain. Those are bound
 * here rather than declared in the .fbp, so the generated base keeps its
 * current set of virtual handlers.
 */
/**
 * Makes sure every executable this run would need is really there.
 *
 * Which one computes the features depends on the describer: the OpenCV
 * variant is a separate executable that not every OpenMVG build ships.
 */
bool Regard3DComputeMatchesDialog::checkOpenMVGExecutables()
{
	R3DExternalPrograms &progs = R3DExternalPrograms::getInstance();
	const int describer = pOMVGDescriberMethodChoice_->GetCurrentSelection();
	const bool isOpenCV = R3DOpenMVGOptions::describerIsOpenCV(describer);

	wxArrayString missing;
	if((isOpenCV ? progs.getComputeFeaturesOpenCVPath() : progs.getComputeFeaturesPath()).IsEmpty())
		missing.Add(isOpenCV ? wxT("openMVG_main_ComputeFeatures_OpenCV")
			: wxT("openMVG_main_ComputeFeatures"));
	if(progs.getPairGeneratorPath().IsEmpty())
		missing.Add(wxT("openMVG_main_PairGenerator"));
	if(progs.getComputeMatchesPath().IsEmpty())
		missing.Add(wxT("openMVG_main_ComputeMatches"));
	if(progs.getGeometricFilterPath().IsEmpty())
		missing.Add(wxT("openMVG_main_GeometricFilter"));

	if(missing.IsEmpty())
		return true;

	wxString message(wxT("The following OpenMVG executables were not found:\n\n"));
	for(size_t i = 0; i < missing.GetCount(); i++)
		message.Append(wxT("    ") + missing[i] + wxT("\n"));
	message.Append(wxT("\nPlease put them into the \"openmvg\" directory next to Regard3D,\n")
		wxT("choose another describer, or use the built-in engine."));

	wxMessageBox(message, wxT("OpenMVG executables not found"), wxICON_ERROR | wxOK, this);
	return false;
}

/**
 * Explains the OpenMVG options in the terms the executables use them.
 *
 * These live here rather than in the .fbp so that they stay next to the code
 * that enables and disables the same controls.
 */
void Regard3DComputeMatchesDialog::setOpenMVGToolTips()
{
	pOMVGDescriberMethodChoice_->SetToolTip(
		wxT("Which feature detector and descriptor is used.\n")
		wxT("SIFT is the reference choice, AKAZE_FLOAT often spreads its features more evenly.\n")
		wxT("AKAZE_MLDB produces binary descriptors and therefore needs a Hamming matcher.\n")
		wxT("The two OPENCV entries need the separate openMVG_main_ComputeFeatures_OpenCV executable."));
	pOMVGDescriberPresetChoice_->SetToolTip(
		wxT("How many features to extract per image. HIGH and ULTRA lower the detection\n")
		wxT("threshold and give more features, at the cost of time and memory.\n")
		wxT("Start with NORMAL and raise it if too few images are reconstructed."));
	pOMVGUprightCheckBox_->SetToolTip(
		wxT("Fix the feature orientation to \"up\" instead of estimating it per feature.\n")
		wxT("Only for image sets that share the same up direction. There it is faster and\n")
		wxT("the descriptors are more distinctive, but rotated images will no longer match."));
	pOMVGNumThreadsTextCtrl_->SetToolTip(
		wxT("How many images are described in parallel. 0 lets OpenMVG use all cores."));

	pOMVGPairModeChoice_->SetToolTip(
		wxT("Which image pairs are tried.\n")
		wxT("Exhaustive tries every pair, the safe choice for an unordered set of photos.\n")
		wxT("Contiguous only matches each image with the following ones, which is much\n")
		wxT("faster for a video or a sequence that was shot in order."));
	pOMVGContiguousCountTextCtrl_->SetToolTip(
		wxT("How many following images each image is matched against, in contiguous mode.\n")
		wxT("5 matches image 0 with 1 to 5, image 1 with 2 to 6, and so on."));

	pOMVGDistanceRatioTextCtrl_->SetToolTip(
		wxT("Lowe's ratio test: a match is kept only if the best descriptor distance is\n")
		wxT("below this fraction of the second best. Lower means fewer but more reliable\n")
		wxT("matches. 0.8 is the OpenMVG default, try 0.6 to 0.7 for repetitive scenes."));
	pOMVGNearestMatchingMethodChoice_->SetToolTip(
		wxT("The algorithm that searches for the nearest descriptors.\n")
		wxT("AUTO picks one from the descriptor type and is a good default.\n")
		wxT("The list only offers what the selected describer allows: L2 methods for scalar\n")
		wxT("descriptors, Hamming methods for the binary ones."));
	pOMVGCacheSizeTextCtrl_->SetToolTip(
		wxT("How many images' features may be held in memory at once.\n")
		wxT("0 loads them all, which is the fastest. Set a limit if a large image set\n")
		wxT("exhausts the available memory."));
	pOMVGPreemptiveTextCtrl_->SetToolTip(
		wxT("Match on only the first N features per image and drop pairs that get too few\n")
		wxT("matches. This finds the promising pairs quickly, but the matches that are kept\n")
		wxT("come from the reduced feature set as well, so leave it at 0 unless a very\n")
		wxT("large image set is otherwise too slow."));

	pOMVGFundamentalCheckBox_->SetToolTip(
		wxT("Filter the putative matches with a robustly estimated fundamental matrix and\n")
		wxT("write matches.f.txt. The incremental triangulation methods need this file."));
	pOMVGEssentialCheckBox_->SetToolTip(
		wxT("Filter the putative matches with a robustly estimated essential matrix and\n")
		wxT("write matches.e.txt. The global triangulation method needs this file."));
	pOMVGHomographyCheckBox_->SetToolTip(
		wxT("Filter the putative matches with a robustly estimated homography and write\n")
		wxT("matches.h.txt. Useful for planar scenes; no triangulation method reads it."));
	pOMVGGuidedMatchingCheckBox_->SetToolTip(
		wxT("Once a model has been estimated, look for further matches that agree with it.\n")
		wxT("Recovers matches the ratio test threw away, at the cost of a slower filtering step."));
}

void Regard3DComputeMatchesDialog::initializeOpenMVGPage()
{
	const R3DOpenMVGMatchingParams &p = results_.openMVG_;

	setOpenMVGToolTips();

	setChoiceSelection(pOMVGDescriberMethodChoice_, p.describerMethod_);
	setChoiceSelection(pOMVGDescriberPresetChoice_, p.describerPreset_);
	pOMVGUprightCheckBox_->SetValue(p.upright_);
	pOMVGNumThreadsTextCtrl_->SetValue(wxString::Format(wxT("%d"), p.numThreads_));

	setChoiceSelection(pOMVGPairModeChoice_, p.pairMode_);
	pOMVGContiguousCountTextCtrl_->SetValue(wxString::Format(wxT("%d"), p.contiguousCount_));

	pOMVGDistanceRatioTextCtrl_->SetValue(wxString::Format(wxT("%.3g"), p.distanceRatio_));
	pOMVGCacheSizeTextCtrl_->SetValue(wxString::Format(wxT("%d"), p.cacheSize_));
	pOMVGPreemptiveTextCtrl_->SetValue(wxString::Format(wxT("%d"), p.preemptiveFeatureCount_));

	pOMVGFundamentalCheckBox_->SetValue(p.computeFundamental_);
	pOMVGEssentialCheckBox_->SetValue(p.computeEssential_);
	pOMVGHomographyCheckBox_->SetValue(p.computeHomography_);
	pOMVGGuidedMatchingCheckBox_->SetValue(p.guidedMatching_);

	// The list is rebuilt from the describer below, so seed the selection first
	// or the value stored in the project would be lost every time we open
	if(p.nearestMatchingMethod_ >= 0 && p.nearestMatchingMethod_ < static_cast<int>(kMatcherCount))
		pOMVGNearestMatchingMethodChoice_->SetStringSelection(kMatchers[p.nearestMatchingMethod_].name_);

	pOMVGDescriberMethodChoice_->Bind(wxEVT_CHOICE,
		[this](wxCommandEvent &) { updateOpenMVGDescriberDependencies(); });
	pOMVGPairModeChoice_->Bind(wxEVT_CHOICE,
		[this](wxCommandEvent &) { updateOpenMVGPairModeDependencies(); });

	updateOpenMVGDescriberDependencies();
	updateOpenMVGPairModeDependencies();
}

/**
 * The describer decides which matchers are legal and which of the
 * ComputeFeatures options exist at all.
 */
void Regard3DComputeMatchesDialog::updateOpenMVGDescriberDependencies()
{
	const int describer = pOMVGDescriberMethodChoice_->GetCurrentSelection();
	const bool binary = R3DOpenMVGOptions::describerIsBinary(describer);

	// Remember the current matcher by name, the list is about to be rebuilt
	const wxString current(pOMVGNearestMatchingMethodChoice_->GetStringSelection());

	pOMVGNearestMatchingMethodChoice_->Clear();
	for(size_t i = 0; i < kMatcherCount; i++)
	{
		if(binary ? kMatchers[i].binary_ : kMatchers[i].scalar_)
			pOMVGNearestMatchingMethodChoice_->Append(kMatchers[i].name_);
	}
	if(pOMVGNearestMatchingMethodChoice_->SetStringSelection(current) == false)
		pOMVGNearestMatchingMethodChoice_->SetSelection(0);		// AUTO

	// openMVG_main_ComputeFeatures_OpenCV only understands -i, -o, -f and -m
	const bool isOpenCV = R3DOpenMVGOptions::describerIsOpenCV(describer);
	pOMVGDescriberPresetChoice_->Enable(!isOpenCV);
	pOMVGUprightCheckBox_->Enable(!isOpenCV);
	pOMVGNumThreadsTextCtrl_->Enable(!isOpenCV);
}

void Regard3DComputeMatchesDialog::updateOpenMVGPairModeDependencies()
{
	// --contiguous_count only means something in CONTIGUOUS mode
	pOMVGContiguousCountTextCtrl_->Enable(pOMVGPairModeChoice_->GetCurrentSelection() == 1);
}

/**
 * Reads one numeric field, complaining about the field by name so the message
 * says which one is wrong. Returns false when the dialog must stay open.
 */
bool Regard3DComputeMatchesDialog::readNumericField(wxTextCtrl *pTextCtrl, const wxString &name,
	double minValue, double maxValue, double &value)
{
	double parsed = 0;
	if(!pTextCtrl->GetValue().Trim(true).Trim(false).ToDouble(&parsed))
	{
		wxMessageBox(name + wxT(" is not a valid number."),
			wxT("Invalid value"), wxICON_WARNING | wxOK, this);
		pTextCtrl->SetFocus();
		return false;
	}
	if(parsed < minValue || parsed > maxValue)
	{
		wxMessageBox(wxString::Format(wxT("%s must be between %.3g and %.3g."),
			name.c_str(), minValue, maxValue),
			wxT("Invalid value"), wxICON_WARNING | wxOK, this);
		pTextCtrl->SetFocus();
		return false;
	}

	value = parsed;
	return true;
}

bool Regard3DComputeMatchesDialog::readOpenMVGPage()
{
	R3DOpenMVGMatchingParams p;
	double value = 0;

	p.describerMethod_ = pOMVGDescriberMethodChoice_->GetCurrentSelection();
	p.describerPreset_ = pOMVGDescriberPresetChoice_->GetCurrentSelection();
	p.upright_ = pOMVGUprightCheckBox_->GetValue();
	if(!readNumericField(pOMVGNumThreadsTextCtrl_, wxT("Number of threads"), 0, 1024, value))
		return false;
	p.numThreads_ = static_cast<int>(value);

	p.pairMode_ = pOMVGPairModeChoice_->GetCurrentSelection();
	if(!readNumericField(pOMVGContiguousCountTextCtrl_, wxT("Contiguous count"), 1, 100000, value))
		return false;
	p.contiguousCount_ = static_cast<int>(value);

	if(!readNumericField(pOMVGDistanceRatioTextCtrl_, wxT("Distance ratio"), 0.1, 1.0, value))
		return false;
	p.distanceRatio_ = value;

	// The list only holds the matchers the describer allows, so store by name
	const wxString matcher(pOMVGNearestMatchingMethodChoice_->GetStringSelection());
	p.nearestMatchingMethod_ = 0;
	for(size_t i = 0; i < kMatcherCount; i++)
	{
		if(matcher == kMatchers[i].name_)
			p.nearestMatchingMethod_ = static_cast<int>(i);
	}

	if(!readNumericField(pOMVGCacheSizeTextCtrl_, wxT("Region cache size"), 0, 1000000, value))
		return false;
	p.cacheSize_ = static_cast<int>(value);
	if(!readNumericField(pOMVGPreemptiveTextCtrl_, wxT("Pre-emptive feature count"), 0, 1000000, value))
		return false;
	p.preemptiveFeatureCount_ = static_cast<int>(value);

	p.computeFundamental_ = pOMVGFundamentalCheckBox_->GetValue();
	p.computeEssential_ = pOMVGEssentialCheckBox_->GetValue();
	p.computeHomography_ = pOMVGHomographyCheckBox_->GetValue();
	p.guidedMatching_ = pOMVGGuidedMatchingCheckBox_->GetValue();

	if(!p.computeFundamental_ && !p.computeEssential_ && !p.computeHomography_)
	{
		wxMessageBox(wxT("Please select at least one geometric model.\n\n")
			wxT("Triangulation needs matches.f.txt for the incremental methods\n")
			wxT("and matches.e.txt for the global one."),
			wxT("No geometric model selected"), wxICON_WARNING | wxOK, this);
		return false;
	}

	results_.openMVG_ = p;
	return true;
}

BEGIN_EVENT_TABLE( Regard3DComputeMatchesDialog, Regard3DComputeMatchesDialogBase )
END_EVENT_TABLE()
