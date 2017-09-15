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
#include "Regard3DTriangulationDialog.h"
#include "PreviewGeneratorThread.h"
#include "OpenMVGHelper.h"
#include "ImageInfo.h"
#include "Regard3DSettings.h"

enum
{
	ID_R3D_TD_TIMER = 3800
};

Regard3DTriangulationDialog *Regard3DTriangulationDialog::pDialog_ = NULL;

Regard3DTriangulationDialog::Regard3DTriangulationDialog(wxWindow *pParent)
	: Regard3DTriangulationDialogBase(pParent),
	aTimer_(this, ID_R3D_TD_TIMER),
	pPreviewGeneratorThread_(NULL),
	pProject_(NULL), pComputeMatches_(NULL), pPictureSet_(NULL),
	isGlobalSfmAvailable_(false), initialImagePairListIsEmpty_(false),
	ipSortColumn_(0)
{
	for(int i = 0; i < 5; i++)
		ipSortDirections_[i] = 0;
}

Regard3DTriangulationDialog::~Regard3DTriangulationDialog()
{
}

void Regard3DTriangulationDialog::setPreviewGeneratorThread(PreviewGeneratorThread *pPreviewGeneratorThread)
{
	pPreviewGeneratorThread_ = pPreviewGeneratorThread;
}

void Regard3DTriangulationDialog::setComputeMatches(R3DProject *pProject, R3DProject::ComputeMatches *pComputeMatches)
{
	pProject_ = pProject;
	pComputeMatches_ = pComputeMatches;
	R3DProject::Object *pObject = pProject->getObjectByTypeAndID(R3DProject::R3DTreeItem::TypePictureSet, pComputeMatches->parentId_);
	if(pObject != NULL)
	{
		pPictureSet_ = dynamic_cast<R3DProject::PictureSet *>(pObject);
	}
	if(pPictureSet_ == NULL)
		return;
	pProject_->getProjectPathsCM(paths_, pComputeMatches);
}

bool Regard3DTriangulationDialog::isTriangulationPossible()
{
	// Check whether preconditions of incremental or global chain are met
	bool isIncrementalTriPossible = false, isGlobalTriPossible = false;

	isIncrementalTriPossible = true;	// We now allow incremental triangulation without known focal length
/*	// Incremental: At least one pair with known focal length
	OpenMVGHelper::ImagePairList imgPairList = OpenMVGHelper::getBestValidatedPairs(paths_, paths_.matchesFFilename_, 0);
	if(!imgPairList.empty())
	{
		const ImageInfoVector &imageInfoVector = pPictureSet_->imageList_;
		for(size_t i = 0; i < imgPairList.size(); i++)
		{
			const OpenMVGHelper::ImagePair &imgPair = imgPairList[i];

			bool focalLengthKnown1 = false, focalLengthKnown2 = false;
			if(imageInfoVector.size() > imgPair.indexA_)
			{
				const ImageInfo &imageInfo = imageInfoVector[imgPair.indexA_];
				focalLengthKnown1 = ((imageInfo.sensorWidth_ != 0)
					&& (imageInfo.focalLength_ != 0));
			}
			if(imageInfoVector.size() > imgPair.indexB_)
			{
				const ImageInfo &imageInfo = imageInfoVector[imgPair.indexB_];
				focalLengthKnown2 = ((imageInfo.sensorWidth_ != 0)
					&& (imageInfo.focalLength_ != 0));
			}

			if(focalLengthKnown1 && focalLengthKnown2)
			{
				isIncrementalTriPossible = true;
			}
		}
	}
	else
	{
		isIncrementalTriPossible = false;
	}*/
	// Store for later
	initialImagePairListIsEmpty_ = !isIncrementalTriPossible;

	// Global triangulation
	isGlobalTriPossible = OpenMVGHelper::isGlobalSfmAvailable(paths_, pPictureSet_);
	isGlobalSfmAvailable_ = isGlobalTriPossible;	// Store for later

	return (isIncrementalTriPossible || isGlobalTriPossible);
}

void Regard3DTriangulationDialog::getResults(bool &global, size_t &initialImageID1, size_t &initialImageID2,
	int &rotAveraging, int &transAveraging,
	bool &refineIntrinsics)
{
	if(pTriangulationMethodRadioBox_->GetSelection() > 0)
		global = true;
	else
		global = false;

	int stateMask = wxLIST_STATE_SELECTED;
	int imagePairCount = pTInitialImagePairListCtrl_->GetItemCount();
	for(int i = 0; i < imagePairCount; i++)
	{
		int itemState = pTInitialImagePairListCtrl_->GetItemState(i, stateMask);
		if(itemState != 0)
		{
			int index = pTInitialImagePairListCtrl_->GetItemData(i);
			// This item is selected, determine image indices
			if(static_cast<size_t>(index) < imageIDList_.size())
			{
				initialImageID1 = imageIDList_[index].first;
				initialImageID2 = imageIDList_[index].second;
			}
		}
	}

	rotAveraging = pTGlobalRotAvgMethodRatioBox_->GetSelection() + 1;
	transAveraging = pTGlobalTranslAvgMethodRadioBox_->GetSelection() + 1;
	refineIntrinsics = pTRefineCameraIntrinsicsCheckBox_->GetValue();
}

void Regard3DTriangulationDialog::OnPreviewFinished()
{
	checkForPreviewImage();
}

void Regard3DTriangulationDialog::OnNewImageInfos()
{
}

void Regard3DTriangulationDialog::EndModal(int retCode)
{
	Regard3DSettings::getInstance().saveTriangulationLayoutToConfig(this);

	wxDialog::EndModal(retCode);
}

void Regard3DTriangulationDialog::OnInitDialog( wxInitDialogEvent& event )
{
	updateInitialImagePairListCtrl();
	updateTriangulationMethodChoice();

	Regard3DSettings::getInstance().loadTriangulationLayoutFromConfig(this);
	aTimer_.Start(50);
}

void Regard3DTriangulationDialog::OnTriangulationMethodRadioBox( wxCommandEvent& event )
{
	if(event.GetInt() == 0)
	{
		// Incremental method
		pTInitialImagePairListCtrl_->Enable();
		pTPreviewWithMatchesCheckBox_->Enable();
		pTGlobalRotAvgMethodRatioBox_->Disable();
		pTGlobalTranslAvgMethodRadioBox_->Disable();
	}
	else
	{
		// Global method
		pTInitialImagePairListCtrl_->Disable();
		pTPreviewWithMatchesCheckBox_->Disable();
		pTGlobalRotAvgMethodRatioBox_->Enable();
		pTGlobalTranslAvgMethodRadioBox_->Enable();
	}
}

// returns 0 if the items are equal, negative value if the first item is less than the second one and positive value if the first one is greater than the second one 
#if wxCHECK_VERSION(2, 9, 0)
int wxCALLBACK Regard3DTriangulationDialog::TInitialImagePairListCompareFunction(wxIntPtr item1, wxIntPtr item2, wxIntPtr WXUNUSED(sortData))
#else
int wxCALLBACK Regard3DTriangulationDialog::TInitialImagePairListCompareFunction(long item1, long item2, long WXUNUSED(sortData))
#endif
{
	Regard3DTriangulationDialog *pDialog = Regard3DTriangulationDialog::pDialog_;
	wxListCtrl *pListCtrl = pDialog->pTInitialImagePairListCtrl_;
	long index1 = pListCtrl->FindItem(-1, item1);	// Find list entry with data item1
	long index2 = pListCtrl->FindItem(-1, item2);
	int col = pDialog->ipSortColumn_;
	bool reverseOrder = (pDialog->ipSortDirections_[col] == 1);

	size_t item1ImageA = 0, item1ImageB = 0, item2ImageA = 0, item2ImageB = 0;
	if(static_cast<size_t>(item1) < pDialog->imageIDList_.size())
	{
		item1ImageA = pDialog->imageIDList_[item1].first;
		item1ImageB = pDialog->imageIDList_[item1].second;
	}
	if(static_cast<size_t>(item2) < pDialog->imageIDList_.size())
	{
		item2ImageA = pDialog->imageIDList_[item2].first;
		item2ImageB = pDialog->imageIDList_[item2].second;
	}

	if(col == 0)
	{
		int cmp = 0;
		if(item1ImageA < item2ImageA)
			cmp = -1;
		else if(item1ImageA > item2ImageA)
			cmp = 1;
		if(reverseOrder)
			cmp = -cmp;
		return cmp;
	}
	else if(col == 2)
	{
		int cmp = 0;
		if(item1ImageB < item2ImageB)
			cmp = -1;
		else if(item1ImageB > item2ImageB)
			cmp = 1;
		if(reverseOrder)
			cmp = -cmp;
		return cmp;
	}
#if wxCHECK_VERSION(2, 9, 1)
	else if(col == 1)
	{
		wxString text1 = pListCtrl->GetItemText(index1, 1);	// The last parameter only works after wxWidgets 2.9.1
		wxString text2 = pListCtrl->GetItemText(index2, 1);

		int cmp = text1.Cmp(text2);
		if(reverseOrder)
			cmp = -cmp;
		return cmp;
	}
	else if(col == 3)
	{
		wxString text1 = pListCtrl->GetItemText(index1, 3);	// The last parameter only works after wxWidgets 2.9.1
		wxString text2 = pListCtrl->GetItemText(index2, 3);

		int cmp = text1.Cmp(text2);
		if(reverseOrder)
			cmp = -cmp;
		return cmp;
	}
	else if(col == 4)
	{
		wxString text1 = pListCtrl->GetItemText(index1, 4);	// The last parameter only works after wxWidgets 2.9.1
		wxString text2 = pListCtrl->GetItemText(index2, 4);
		long val1, val2;
		text1.ToCLong(&val1);
		text2.ToCLong(&val2);

		int cmp = 0;
		if(val1 < val2)
			cmp = -1;
		else if(val1 > val2)
			cmp = 1;
		if(reverseOrder)
			cmp = -cmp;
		return cmp;
	}
#endif

	return 0;
}

void Regard3DTriangulationDialog::OnTInitialImagePairColClick( wxListEvent& event )
{
	int col = event.GetColumn();
	if(0 <= col
		&& col < pTInitialImagePairListCtrl_->GetColumnCount())
	{
		ipSortColumn_ = col;
		pDialog_ = this;
		pTInitialImagePairListCtrl_->SortItems( TInitialImagePairListCompareFunction, 0 );

		ipSortDirections_[col] = 1 - ipSortDirections_[col];
	}
}

void Regard3DTriangulationDialog::OnTInitialImagePairItemDeselected( wxListEvent& event )
{
	// Clear preview
	pPreviewCanvas_->clearPreviewImage();
	previewInfoMatches_.clear();
}

void Regard3DTriangulationDialog::OnTInitialImagePairItemSelected( wxListEvent& event )
{
	// Create preview
	long i = event.GetIndex();
	int index = pTInitialImagePairListCtrl_->GetItemData(i);
	int imageID1 = 0, imageID2 = 0;
	if(index < static_cast<long>(imageIDList_.size()))
	{
		imageID1 = static_cast<int>(imageIDList_[index].first);
		imageID2 = static_cast<int>(imageIDList_[index].second);
	}

	PreviewInfo previewInfo = pProject_->prepareMatchesPreview(pComputeMatches_->id_, imageID1, imageID2);

	if(pTPreviewWithMatchesCheckBox_->GetValue())
		previewInfo.keypointType_ = PreviewInfo::PIKPTRichKeypoints;
	else
		previewInfo.keypointType_ = PreviewInfo::PIKPTNoKeypoints;

	pPreviewGeneratorThread_->addPreviewRequest(previewInfo);
}

void Regard3DTriangulationDialog::OnTPreviewWithMatchesCheckBox( wxCommandEvent& event )
{
	pPreviewCanvas_->clearPreviewImage();

	PreviewInfo previewInfo = pProject_->prepareMatchesPreview(pComputeMatches_->id_,
		previewInfoMatches_.index1_, previewInfoMatches_.index2_);
	if(event.IsChecked())
		previewInfo.keypointType_ = PreviewInfo::PIKPTRichKeypoints;
	else
		previewInfo.keypointType_ = PreviewInfo::PIKPTNoKeypoints;

	pPreviewGeneratorThread_->addPreviewRequest(previewInfo);
}

void Regard3DTriangulationDialog::OnTimer( wxTimerEvent &WXUNUSED(event) )
{
	checkForPreviewImage();
}

void Regard3DTriangulationDialog::updateInitialImagePairListCtrl()
{
	pTInitialImagePairListCtrl_->ClearAll();
	imageIDList_.clear();

	// This was checked before in isTriangulationPossible()
	if(initialImagePairListIsEmpty_)
		return;

	OpenMVGHelper::ImagePairList imgPairList = OpenMVGHelper::getBestValidatedPairs(paths_, paths_.matchesFFilename_, 0);
	if(!imgPairList.empty())
	{
		const ImageInfoVector &imageInfoVector = pPictureSet_->imageList_;
		pTInitialImagePairListCtrl_->InsertColumn(pTInitialImagePairListCtrl_->GetColumnCount(), wxT("Index 1"));
		pTInitialImagePairListCtrl_->InsertColumn(pTInitialImagePairListCtrl_->GetColumnCount(), wxT("Image 1"));
		pTInitialImagePairListCtrl_->InsertColumn(pTInitialImagePairListCtrl_->GetColumnCount(), wxT("Index 2"));
		pTInitialImagePairListCtrl_->InsertColumn(pTInitialImagePairListCtrl_->GetColumnCount(), wxT("Image 2"));
		pTInitialImagePairListCtrl_->InsertColumn(pTInitialImagePairListCtrl_->GetColumnCount(), wxT("# Matches"));

		size_t newID = 0;

		bool noPairHasFullImageInfo = true;
		for(size_t i = 0; i < imgPairList.size(); i++)
		{
			const OpenMVGHelper::ImagePair &imgPair = imgPairList[i];
			bool focalLengthKnown1 = false, focalLengthKnown2 = false;
			if(imageInfoVector.size() > imgPair.indexA_)
			{
				const ImageInfo &imageInfo = imageInfoVector[imgPair.indexA_];
				focalLengthKnown1 = ((imageInfo.sensorWidth_ != 0)
					&& (imageInfo.focalLength_ != 0));
			}
			if(imageInfoVector.size() > imgPair.indexB_)
			{
				const ImageInfo &imageInfo = imageInfoVector[imgPair.indexB_];
				focalLengthKnown2 = ((imageInfo.sensorWidth_ != 0)
					&& (imageInfo.focalLength_ != 0));
			}
			if(focalLengthKnown1 && focalLengthKnown2)
				noPairHasFullImageInfo = false;
		}

		for(size_t i = 0; i < imgPairList.size(); i++)
		{
			const OpenMVGHelper::ImagePair &imgPair = imgPairList[i];

			wxString imageFilenameA(imgPair.imageFilenameA_.c_str(), *wxConvCurrent);
			wxString imageFilenameB(imgPair.imageFilenameB_.c_str(), *wxConvCurrent);

			wxFileName imageAFN(imageFilenameA);
			wxFileName imageBFN(imageFilenameB);

			bool focalLengthKnown1 = false, focalLengthKnown2 = false;
			if(imageInfoVector.size() > imgPair.indexA_)
			{
				const ImageInfo &imageInfo = imageInfoVector[imgPair.indexA_];
				imageAFN = wxFileName(imageInfo.filename_);
				focalLengthKnown1 = ((imageInfo.sensorWidth_ != 0)
					&& (imageInfo.focalLength_ != 0));
			}
			if(imageInfoVector.size() > imgPair.indexB_)
			{
				const ImageInfo &imageInfo = imageInfoVector[imgPair.indexB_];
				imageBFN = wxFileName(imageInfo.filename_);
				focalLengthKnown2 = ((imageInfo.sensorWidth_ != 0)
					&& (imageInfo.focalLength_ != 0));
			}

			if(noPairHasFullImageInfo
				|| (focalLengthKnown1 && focalLengthKnown2))
			{
				pTInitialImagePairListCtrl_->InsertItem(newID, wxString::Format(wxT("%d"), imgPair.indexA_));
				pTInitialImagePairListCtrl_->SetItem(newID, 1, imageAFN.GetFullName());
				pTInitialImagePairListCtrl_->SetItem(newID, 2, wxString::Format(wxT("%d"), imgPair.indexB_));
				pTInitialImagePairListCtrl_->SetItem(newID, 3,  imageBFN.GetFullName());
				pTInitialImagePairListCtrl_->SetItem(newID, 4, wxString::Format(wxT("%d"), imgPair.matches_));
				pTInitialImagePairListCtrl_->SetItemData(newID, newID);	// Used in sorting

				// Store image indexes in imageIDList_
				imageIDList_.push_back( std::make_pair(imgPair.indexA_, imgPair.indexB_) );

				newID++;
			}
		}

		if(newID > 0)
		{
			int stateMask = wxLIST_STATE_SELECTED;
			pTInitialImagePairListCtrl_->SetItemState(0, stateMask, stateMask);	// Select first item
			pTInitialImagePairListCtrl_->SetColumnWidth(0, wxLIST_AUTOSIZE);
			pTInitialImagePairListCtrl_->SetColumnWidth(1, wxLIST_AUTOSIZE);
			pTInitialImagePairListCtrl_->SetColumnWidth(2, wxLIST_AUTOSIZE);
			pTInitialImagePairListCtrl_->SetColumnWidth(3, wxLIST_AUTOSIZE);
			pTInitialImagePairListCtrl_->SetColumnWidth(4, wxLIST_AUTOSIZE);
		}
	}
}

void Regard3DTriangulationDialog::updateTriangulationMethodChoice()
{
	pTriangulationMethodRadioBox_->Enable(1, isGlobalSfmAvailable_);
	pTriangulationMethodRadioBox_->Select(0);

	// Enable controls for incremental method, disable controls for global method
	pTInitialImagePairListCtrl_->Enable();
	pTPreviewWithMatchesCheckBox_->Enable();
	pTGlobalRotAvgMethodRatioBox_->Disable();
	pTGlobalTranslAvgMethodRadioBox_->Disable();

	pTriangulationPanel_->Layout();
}

void Regard3DTriangulationDialog::checkForPreviewImage()
{
	if(pPreviewGeneratorThread_ == NULL)
		return;

	wxImage previewImage;
	PreviewInfo previewInfo;
	if(pPreviewGeneratorThread_->getNewPreviewImage(previewImage, previewInfo))
	{
		pPreviewCanvas_->previewImage(previewImage);
		pPreviewCanvas_->Refresh();
		previewInfoMatches_ = previewInfo;
	}
}

BEGIN_EVENT_TABLE( Regard3DTriangulationDialog, Regard3DTriangulationDialogBase )
	EVT_TIMER(ID_R3D_TD_TIMER, Regard3DTriangulationDialog::OnTimer)
END_EVENT_TABLE()

