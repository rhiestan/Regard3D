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
#include "Regard3DMatchingResultsDialog.h"
#include "Regard3DImagePreviewDialog.h"
#include "OpenMVGHelper.h"
#include "Regard3DSettings.h"

enum
{
	ID_R3D_MRD_TIMER = 3900
};

Regard3DMatchingResultsDialog *Regard3DMatchingResultsDialog::pDialog_ = NULL;

Regard3DMatchingResultsDialog::Regard3DMatchingResultsDialog(wxWindow *pParent)
	: Regard3DMatchingResultsDialogBase(pParent),
	aTimer_(this, ID_R3D_MRD_TIMER),
	pPreviewGeneratorThread_(NULL),
	pProject_(NULL), pComputeMatches_(NULL), pPictureSet_(NULL),
	ipSortColumn_(0), skipPreviewImages_(false)
{
	for(int i = 0; i < 5; i++)
		ipSortDirections_[i] = 0;
}

Regard3DMatchingResultsDialog::~Regard3DMatchingResultsDialog()
{
}

void Regard3DMatchingResultsDialog::setPreviewGeneratorThread(PreviewGeneratorThread *pPreviewGeneratorThread)
{
	pPreviewGeneratorThread_ = pPreviewGeneratorThread;
}

void Regard3DMatchingResultsDialog::setComputeMatches(R3DProject *pProject, R3DProject::ComputeMatches *pComputeMatches)
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

void Regard3DMatchingResultsDialog::OnPreviewFinished()
{
	checkForPreviewImage();
}

void Regard3DMatchingResultsDialog::OnNewImageInfos()
{
}

void Regard3DMatchingResultsDialog::EndModal(int retCode)
{
	Regard3DSettings::getInstance().saveMatchingResultsLayoutToConfig(this);

	wxDialog::EndModal(retCode);
}

void Regard3DMatchingResultsDialog::OnInitDialog( wxInitDialogEvent& event )
{
	pImageListCtrl_->ClearAll();
	pImageListCtrl_->InsertColumn(0, wxT("Index"));
	pImageListCtrl_->InsertColumn(1, wxT("Image file name"));
	pImageListCtrl_->InsertColumn(2, wxT("Image size"));
	pImageListCtrl_->InsertColumn(3, wxT("Camera maker"));
	pImageListCtrl_->InsertColumn(4, wxT("Camera model"));
	pImageListCtrl_->InsertColumn(5, wxT("Focal length"));
	pImageListCtrl_->InsertColumn(6, wxT("Sensor width"));
	pImageListCtrl_->InsertColumn(7, wxT("Keypoints"));

	pMatchesChoice_->Clear();
	pMatchesChoice_->Append(wxT("No filter (putative matches)"));
	pMatchesChoice_->Append(wxT("Homography matrix"));
	pMatchesChoice_->Append(wxT("Fundamental matrix"));
	if(wxFileName::FileExists( wxString( paths_.matchesEFilename_.c_str(), *wxConvCurrent ) ))
		pMatchesChoice_->Append(wxT("Essential matrix"));
	pMatchesChoice_->Select(2);

	const ImageInfoVector &imageList = pPictureSet_->imageList_;
	for(size_t i = 0; i < imageList.size(); i++)
	{
		long newId = pImageListCtrl_->GetItemCount();
		const ImageInfo &imageInfo = imageList[i];

		pImageListCtrl_->InsertItem(newId, wxString::Format(wxT("%d"), static_cast<int>(i)));
		pImageListCtrl_->SetItem(i, 1, imageInfo.filename_);

		wxString imageSizeStr = wxString::Format(wxT("%d x %d"), imageInfo.imageWidth_, imageInfo.imageHeight_);
		pImageListCtrl_->SetItem(i, 2, imageSizeStr);
		pImageListCtrl_->SetItem(i, 3, imageInfo.cameraMaker_);
		pImageListCtrl_->SetItem(i, 4, imageInfo.cameraModel_);

		wxString focalLengthStr(wxT("N/A"));
		if(imageInfo.focalLength_ > 0)
			focalLengthStr = wxString::Format(wxT("%.3g mm"), imageInfo.focalLength_);
		pImageListCtrl_->SetItem(i, 5, focalLengthStr);

		wxString sensorWidthStr(wxT("N/A"));
		if(imageInfo.sensorWidth_ > 0)
			sensorWidthStr = wxString::Format(wxT("%.3g mm"), imageInfo.sensorWidth_);
		pImageListCtrl_->SetItem(i, 6, sensorWidthStr);
		pImageListCtrl_->SetItemData(i, i);

		if(pComputeMatches_ != NULL)
		{
			if(pComputeMatches_->numberOfKeypoints_.size() > i)
			{
				wxString numberOfKeypointsStr;
				if(pComputeMatches_->numberOfKeypoints_[i] > 0)
					numberOfKeypointsStr.Printf(wxT("%d"), static_cast<int>(pComputeMatches_->numberOfKeypoints_[i]));
				pImageListCtrl_->SetItem(i, 7, numberOfKeypointsStr);
			}
		}

	}
	pImageListCtrl_->SetColumnWidth(0, wxLIST_AUTOSIZE);
	pImageListCtrl_->SetColumnWidth(1, wxLIST_AUTOSIZE);
	pImageListCtrl_->SetColumnWidth(2, wxLIST_AUTOSIZE);
	pImageListCtrl_->SetColumnWidth(3, wxLIST_AUTOSIZE);
	pImageListCtrl_->SetColumnWidth(4, wxLIST_AUTOSIZE);
	pImageListCtrl_->SetColumnWidth(5, wxLIST_AUTOSIZE);
	pImageListCtrl_->SetColumnWidth(6, wxLIST_AUTOSIZE);
	if(pImageListCtrl_->GetItemCount() > 0)
	{
		int stateMask = wxLIST_STATE_SELECTED;
		pImageListCtrl_->SetItemState(0, stateMask, stateMask);	// Select first item
	}

	pIPPreviewCanvas_->clearPreviewImage();
	pIPOpenPreviewWindow_->Disable();
	pTPreviewCanvas_->clearPreviewImage();
	pTOpenPreviewWindow_->Disable();
	previewInfoKP_.clear();
	previewInfoMatches_.clear();
	previewInfoMatches_.matchesFilename_ = wxString( paths_.matchesFFilename_.c_str(), wxConvLibc );

	updateInitialImagePairListCtrl();

	Regard3DSettings::getInstance().loadMatchingResultsLayoutFromConfig(this);

	aTimer_.Start(50);
}

void Regard3DMatchingResultsDialog::OnIPImageListColClick( wxListEvent& event )
{
}

void Regard3DMatchingResultsDialog::OnIPImageListItemDeselected( wxListEvent& event )
{
	pIPPreviewCanvas_->clearPreviewImage();
	pIPOpenPreviewWindow_->Disable();
	previewInfoKP_.clear();
}

void Regard3DMatchingResultsDialog::OnIPImageListItemSelected( wxListEvent& event )
{
	long index = event.GetIndex();

	const ImageInfoVector &imageList = pPictureSet_->imageList_;
	int i = pImageListCtrl_->GetItemData(index);
	wxString filename = imageList[i].filename_;

	PreviewInfo previewInfo = pProject_->prepareKeypointPreview(pComputeMatches_->id_, filename);
	if(pIPPreviewWithKeypointsCheckBox_->GetValue())
		previewInfo.keypointType_ = PreviewInfo::PIKPTRichKeypoints;
	else
		previewInfo.keypointType_ = PreviewInfo::PIKPTNoKeypoints;

	pPreviewGeneratorThread_->addPreviewRequest(previewInfo);
}

void Regard3DMatchingResultsDialog::OnIPImageListKeyDown( wxListEvent& event )
{
}

void Regard3DMatchingResultsDialog::OnIPPreviewWithKeypointsCheckBox( wxCommandEvent& event )
{
	// Request new preview
	if(!previewInfoKP_.filename1_.IsEmpty())
	{
		PreviewInfo previewInfo = pProject_->prepareKeypointPreview(pComputeMatches_->id_, previewInfoKP_.filename1_);
		if(event.IsChecked())
			previewInfo.keypointType_ = PreviewInfo::PIKPTRichKeypoints;
		else
			previewInfo.keypointType_ = PreviewInfo::PIKPTNoKeypoints;
		pPreviewGeneratorThread_->addPreviewRequest(previewInfo);
	}
	pIPPreviewCanvas_->clearPreviewImage();
	pIPOpenPreviewWindow_->Disable();
}

void Regard3DMatchingResultsDialog::OnIPOpenPreviewWindow( wxCommandEvent& event )
{
	Regard3DImagePreviewDialog dlg(this);
	dlg.setImage(pIPPreviewCanvas_->getPreviewImage());
	dlg.setPreviewInfo(previewInfoKP_);
	dlg.setPreviewGeneratorThread(pPreviewGeneratorThread_);
	dlg.setComputeMatches(pProject_, pComputeMatches_);

	// While the dialog is shown, skip preview images here
	// They are processed in the dialog itself
	skipPreviewImages_ = true;
	try
	{
		dlg.ShowModal();
	}
	catch(...)
	{
		skipPreviewImages_ = false;
		throw;
	}
	skipPreviewImages_ = false;
}

void Regard3DMatchingResultsDialog::OnMatchesChoice( wxCommandEvent& event )
{
	if(event.GetInt() == 0)			// Putative
		previewInfoMatches_.matchesFilename_ = wxString( paths_.matchesPutitativeFilename_.c_str(), wxConvLibc );
	else if(event.GetInt() == 1)	// Homographic
		previewInfoMatches_.matchesFilename_ = wxString( paths_.matchesHFilename_.c_str(), wxConvLibc );
	else if(event.GetInt() == 2)	// Fundamental
		previewInfoMatches_.matchesFilename_ = wxString( paths_.matchesFFilename_.c_str(), wxConvLibc );
	else if(event.GetInt() == 3)	// Essential
		previewInfoMatches_.matchesFilename_ = wxString( paths_.matchesEFilename_.c_str(), wxConvLibc );

	updateInitialImagePairListCtrl();
}

void Regard3DMatchingResultsDialog::OnTInitialImagePairColClick( wxListEvent& event )
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

void Regard3DMatchingResultsDialog::OnTInitialImagePairItemDeselected( wxListEvent& event )
{
	// Clear preview
	pTPreviewCanvas_->clearPreviewImage();
	pTOpenPreviewWindow_->Disable();
//	previewInfoMatches_.clear();
}

void Regard3DMatchingResultsDialog::OnTInitialImagePairItemSelected( wxListEvent& event )
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

	previewInfo.matchesFilename_ = previewInfoMatches_.matchesFilename_;
	pPreviewGeneratorThread_->addPreviewRequest(previewInfo);
}

void Regard3DMatchingResultsDialog::OnTPreviewWithMatchesCheckBox( wxCommandEvent& event )
{
	pTPreviewCanvas_->clearPreviewImage();

	PreviewInfo previewInfo = pProject_->prepareMatchesPreview(pComputeMatches_->id_,
		previewInfoMatches_.index1_, previewInfoMatches_.index2_);
	if(event.IsChecked())
		previewInfo.keypointType_ = PreviewInfo::PIKPTRichKeypoints;
	else
		previewInfo.keypointType_ = PreviewInfo::PIKPTNoKeypoints;

	previewInfo.matchesFilename_ = previewInfoMatches_.matchesFilename_;
	pPreviewGeneratorThread_->addPreviewRequest(previewInfo);
}

void Regard3DMatchingResultsDialog::OnTOpenPreviewWindow( wxCommandEvent& event )
{
	Regard3DImagePreviewDialog dlg(this);
	dlg.setImage(pTPreviewCanvas_->getPreviewImage());
	dlg.setPreviewInfo(previewInfoMatches_);
	dlg.setPreviewGeneratorThread(pPreviewGeneratorThread_);
	dlg.setComputeMatches(pProject_, pComputeMatches_);
    
	// While the dialog is shown, skip preview images in the mainframe
	// They are processed in the dialog itself
	skipPreviewImages_ = true;
	try
	{
		dlg.ShowModal();
	}
	catch(...)
	{
		skipPreviewImages_ = false;
		throw;
	}
	skipPreviewImages_ = false;
}

// returns 0 if the items are equal, negative value if the first item is less than the second one and positive value if the first one is greater than the second one 
#if wxCHECK_VERSION(2, 9, 0)
int wxCALLBACK Regard3DMatchingResultsDialog::TInitialImagePairListCompareFunction(wxIntPtr item1, wxIntPtr item2, wxIntPtr WXUNUSED(sortData))
#else
int wxCALLBACK Regard3DMatchingResultsDialog::TInitialImagePairListCompareFunction(long item1, long item2, long WXUNUSED(sortData))
#endif
{
	Regard3DMatchingResultsDialog *pDialog = Regard3DMatchingResultsDialog::pDialog_;
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

/*void Regard3DMatchingResultsDialog::OnOK( wxCommandEvent& event )
{
	EndModal(wxID_OK);
}
*/
void Regard3DMatchingResultsDialog::OnTimer( wxTimerEvent &WXUNUSED(event) )
{
	checkForPreviewImage();
}

void Regard3DMatchingResultsDialog::updateInitialImagePairListCtrl()
{
	std::string matchesFilename;
	int selectedFilter = pMatchesChoice_->GetSelection();
	if(selectedFilter == 0)
		matchesFilename = paths_.matchesPutitativeFilename_;	// No filter
	else if(selectedFilter == 1)
		matchesFilename = paths_.matchesHFilename_;			// Homography
	else if(selectedFilter == 2)
		matchesFilename = paths_.matchesFFilename_;			// Fundamental
	else
		matchesFilename = paths_.matchesEFilename_;			// Essential

	pTInitialImagePairListCtrl_->ClearAll();
	imageIDList_.clear();
	OpenMVGHelper::ImagePairList imgPairList = OpenMVGHelper::getBestValidatedPairs(paths_, matchesFilename, 100);
	if(!imgPairList.empty())
	{
		const ImageInfoVector &imageInfoVector = pPictureSet_->imageList_;
		pTInitialImagePairListCtrl_->InsertColumn(pTInitialImagePairListCtrl_->GetColumnCount(), wxT("Index 1"));
		pTInitialImagePairListCtrl_->InsertColumn(pTInitialImagePairListCtrl_->GetColumnCount(), wxT("Image 1"));
		pTInitialImagePairListCtrl_->InsertColumn(pTInitialImagePairListCtrl_->GetColumnCount(), wxT("Index 2"));
		pTInitialImagePairListCtrl_->InsertColumn(pTInitialImagePairListCtrl_->GetColumnCount(), wxT("Image 2"));
		pTInitialImagePairListCtrl_->InsertColumn(pTInitialImagePairListCtrl_->GetColumnCount(), wxT("# Matches"));

		for(size_t i = 0; i < imgPairList.size(); i++)
		{
			const OpenMVGHelper::ImagePair &imgPair = imgPairList[i];

			wxString imageFilenameA(imgPair.imageFilenameA_.c_str(), *wxConvCurrent);
			wxString imageFilenameB(imgPair.imageFilenameB_.c_str(), *wxConvCurrent);

			wxFileName imageAFN(imageFilenameA);
			wxFileName imageBFN(imageFilenameB);

			if(imageInfoVector.size() > imgPair.indexA_)
			{
				const ImageInfo &imageInfo = imageInfoVector[imgPair.indexA_];
				imageAFN = wxFileName(imageInfo.filename_);
			}
			if(imageInfoVector.size() > imgPair.indexB_)
			{
				const ImageInfo &imageInfo = imageInfoVector[imgPair.indexB_];
				imageBFN = wxFileName(imageInfo.filename_);
			}
			pTInitialImagePairListCtrl_->InsertItem(i, wxString::Format(wxT("%d"), imgPair.indexA_));
			pTInitialImagePairListCtrl_->SetItem(i, 1, imageAFN.GetFullName());
			pTInitialImagePairListCtrl_->SetItem(i, 2, wxString::Format(wxT("%d"), imgPair.indexB_));
			pTInitialImagePairListCtrl_->SetItem(i, 3,  imageBFN.GetFullName());
			pTInitialImagePairListCtrl_->SetItem(i, 4, wxString::Format(wxT("%d"), imgPair.matches_));
			pTInitialImagePairListCtrl_->SetItemData(i, i);	// Used in sorting

			// Store image indexes in imageIDList_
			imageIDList_.push_back( std::make_pair(imgPair.indexA_, imgPair.indexB_) );
		}

		int stateMask = wxLIST_STATE_SELECTED;
		pTInitialImagePairListCtrl_->SetItemState(0, stateMask, stateMask);	// Select first item
		pTInitialImagePairListCtrl_->SetColumnWidth(0, wxLIST_AUTOSIZE);
		pTInitialImagePairListCtrl_->SetColumnWidth(1, wxLIST_AUTOSIZE);
		pTInitialImagePairListCtrl_->SetColumnWidth(2, wxLIST_AUTOSIZE);
		pTInitialImagePairListCtrl_->SetColumnWidth(3, wxLIST_AUTOSIZE);
		pTInitialImagePairListCtrl_->SetColumnWidth(4, wxLIST_AUTOSIZE);
	}
}

void Regard3DMatchingResultsDialog::checkForPreviewImage()
{
	if(pPreviewGeneratorThread_ == NULL)
		return;

	if(!skipPreviewImages_)
	{
		wxImage previewImage;
		PreviewInfo previewInfo;
		if(pPreviewGeneratorThread_->getNewPreviewImage(previewImage, previewInfo))
		{
			if(previewInfo.type_ == PreviewInfo::PITMatches)
			{
				pTPreviewCanvas_->previewImage(previewImage);
				if(!pTOpenPreviewWindow_->IsEnabled())
					pTOpenPreviewWindow_->Enable();
				previewInfoMatches_ = previewInfo;
			}
			else
			{
				pIPPreviewCanvas_->previewImage(previewImage);
				if(!pIPOpenPreviewWindow_->IsEnabled())
					pIPOpenPreviewWindow_->Enable();
				previewInfoKP_ = previewInfo;
			}
		}
	}
}

BEGIN_EVENT_TABLE( Regard3DMatchingResultsDialog, Regard3DMatchingResultsDialogBase )
	EVT_TIMER(ID_R3D_MRD_TIMER, Regard3DMatchingResultsDialog::OnTimer)
END_EVENT_TABLE()

