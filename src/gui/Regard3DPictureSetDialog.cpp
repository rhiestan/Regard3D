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
#include "Regard3DPictureSetDialog.h"
#include "PreviewGeneratorThread.h"
#include "Regard3DDropTarget.h"
#include "R3DProject.h"
#include "ImageInfoThread.h"
#include "Regard3DSettings.h"

enum
{
	ID_R3D_PSD_TIMER = 3700
};

Regard3DPictureSetDialog::Regard3DPictureSetDialog(wxWindow *pParent)
	: Regard3DPictureSetDialogBase(pParent),
	aTimer_(this, ID_R3D_PSD_TIMER),
	pPreviewGeneratorThread_(NULL), pPictureSet_(NULL),
	showOnly_(false), pRegard3DPictureSetDlgDropTarget_(NULL),
	sentImageInfoRequests_(0), receivedImageInfos_(0)
{
	aTimer_.Start(50);
}

Regard3DPictureSetDialog::~Regard3DPictureSetDialog()
{
}

void Regard3DPictureSetDialog::setPreviewGeneratorThread(PreviewGeneratorThread *pPreviewGeneratorThread)
{
	pPreviewGeneratorThread_ = pPreviewGeneratorThread;
}

void Regard3DPictureSetDialog::setImageInfoThread(ImageInfoThread *pImageInfoThread)
{
	pImageInfoThread_ = pImageInfoThread;
}

void Regard3DPictureSetDialog::setPictureSet(R3DProject::PictureSet *pPictureSet)
{
	pPictureSet_ = pPictureSet;
	showOnly_ = !(pPictureSet->computeMatches_.empty());
}

void Regard3DPictureSetDialog::EndModal(int retCode)
{
	Regard3DSettings::getInstance().savePictureSetLayoutToConfig(this);

	wxDialog::EndModal(retCode);
}

void Regard3DPictureSetDialog::OnInitDialog( wxInitDialogEvent& event )
{
	if(showOnly_)
	{
		pAddFilesButton_->Disable();
		pRemoveSelectedFilesButton_->Disable();
		pClearFileListButton_->Disable();
		pPictureSetNameTextCtrl_->Disable();	// Can't change to read-only after creation
	}
	else
	{
		pRegard3DPictureSetDlgDropTarget_ = new Regard3DPictureSetDlgDropTarget();
		pRegard3DPictureSetDlgDropTarget_->setDialog(this);
		SetDropTarget(pRegard3DPictureSetDlgDropTarget_);	// Memory responsibility goes to wxWindow
	}

	pImageListCtrl_->ClearAll();
	pImageListCtrl_->InsertColumn(0, wxT("Image file name"));
	pImageListCtrl_->InsertColumn(1, wxT("Image size"));
	pImageListCtrl_->InsertColumn(2, wxT("Camera maker"));
	pImageListCtrl_->InsertColumn(3, wxT("Camera model"));
	pImageListCtrl_->InsertColumn(4, wxT("Focal length"));
	pImageListCtrl_->InsertColumn(5, wxT("Sensor width"));

	if(pPictureSet_ != NULL)
	{
		// Turn into edit mode
		imageList_ = pPictureSet_->imageList_;
		for(size_t i = 0; i < imageList_.size(); i++)
		{
			const ImageInfo &ii = imageList_[i];
			addFileToImageList(ii.filename_, false, &ii);
		}

		pPictureSetNameTextCtrl_->SetValue( pPictureSet_->name_ );
	}

	Regard3DSettings::getInstance().loadPictureSetLayoutFromConfig(this);
}

void Regard3DPictureSetDialog::OnAddFilesButton( wxCommandEvent& event )
{
	wxFileDialog chooseFileDialog(this, wxT("Add image files"),
		wxEmptyString, wxEmptyString, wxT("JPEG files (*.jpg;*.jpeg;*.JPG;*.JPEG)|*.jpg;*.jpeg;*.JPG;*.JPEG|All files(*.*)|*.*"),
		wxFD_OPEN | wxFD_FILE_MUST_EXIST | wxFD_MULTIPLE);

	if(chooseFileDialog.ShowModal() == wxID_OK)
	{
		wxString directory = chooseFileDialog.GetDirectory();

		wxArrayString filenames;
		chooseFileDialog.GetFilenames(filenames);

		for(size_t i = 0; i < filenames.GetCount(); i++)
		{
			wxFileName fn(directory, filenames[i], wxPATH_NATIVE);
			wxString filename = fn.GetFullPath();

			addFileToImageList(filename, true);
		}
	}
}

void Regard3DPictureSetDialog::OnRemoveSelectedFilesButton( wxCommandEvent& event )
{
	removeSelectedItems( pImageListCtrl_ );
}

void Regard3DPictureSetDialog::OnClearFileListButton( wxCommandEvent& event )
{
	pImageListCtrl_->DeleteAllItems();
	pPreviewCanvas_->clearPreviewImage();
}

void Regard3DPictureSetDialog::OnImageListColClick( wxListEvent& event )
{
}

void Regard3DPictureSetDialog::OnImageListItemDeselected( wxListEvent& event )
{
	pPreviewCanvas_->clearPreviewImage();
}

void Regard3DPictureSetDialog::OnImageListItemSelected( wxListEvent& event )
{
	long index = event.GetIndex();

	wxString filename = pImageListCtrl_->GetItemText(index);

	PreviewInfo previewInfo;
	previewInfo.type_ = PreviewInfo::PITKeypoints;
	previewInfo.filename1_ = filename;
	previewInfo.keypointType_ = PreviewInfo::PIKPTNoKeypoints;

	pPreviewGeneratorThread_->addPreviewRequest(previewInfo);
}

void Regard3DPictureSetDialog::OnImageListKeyDown( wxListEvent& event )
{
	int keyCode = event.GetKeyCode();
	if(keyCode == WXK_DELETE || keyCode == WXK_BACK)
	{
		// Delete selected items
		removeSelectedItems( pImageListCtrl_ );
	}
}

void Regard3DPictureSetDialog::OnCancel( wxCommandEvent& event )
{
	EndModal(wxID_CANCEL);
}

void Regard3DPictureSetDialog::OnOK( wxCommandEvent& event )
{
	if(sentImageInfoRequests_ == receivedImageInfos_)
		EndModal(wxID_OK);
	else
		wxMessageBox(wxT("Please wait until all image infos have been listed"),
			wxT("Picture set"), wxOK | wxICON_INFORMATION | wxCENTRE, this);
}

void Regard3DPictureSetDialog::OnTimer( wxTimerEvent &WXUNUSED(event) )
{
	checkForPreviewImage();
	checkForNewImageInfos();
}

void Regard3DPictureSetDialog::removeSelectedItems( wxListCtrl *pListCtrl )
{
	if(showOnly_)
		return;

	std::vector<int> itemsToBeDeleted;
	int stateMask = wxLIST_STATE_SELECTED;
	int itemCount = pListCtrl->GetItemCount();
	for(int i = 0; i < itemCount; i++)
	{
		int itemState = pListCtrl->GetItemState(i, stateMask);
		if(itemState != 0)
		{
			// Store items to be deleted in an array
			itemsToBeDeleted.push_back(i);
		}
	}

	// First, convert size_t to signed int to allow - 1 if list is empty
	int itemsToBeDeletedSize = static_cast<int>(itemsToBeDeleted.size());
	// Iterate list, start at the back
	for(int i = 0; i < itemsToBeDeletedSize; i++)
	{
		int itemToBeDeleted = itemsToBeDeleted[itemsToBeDeletedSize - 1 - i];

		// Delete entry in imageList_
		wxString filename = pListCtrl->GetItemText(itemToBeDeleted);
		std::vector<ImageInfo>::iterator iter = imageList_.begin();
		while(iter != imageList_.end())
		{
			ImageInfo &ii = (*iter);
			if(ii.filename_.IsSameAs(filename))
			{
				imageList_.erase(iter);
				break;
			}
			iter++;
		}

		pListCtrl->DeleteItem(itemToBeDeleted);
	}
}

void Regard3DPictureSetDialog::addFileToImageList(const wxString &filename, bool addToImageList, const ImageInfo *pImageInfo)
{
	// Check whether file is already in the list
	long newId = pImageListCtrl_->GetItemCount();
	wxFileName fn(filename);
	fn.Normalize();
	wxString fnString1 = fn.GetFullPath();

	for(long i = 0; i < newId; i++)
	{
#if defined(R3D_WIN32)
		// The code below is too slow on Windows
		wxString fnString2 = pImageListCtrl_->GetItemText(i);
		if(fnString1.IsSameAs(fnString2, false))
			return;
#else
		wxFileName fn2(pImageListCtrl_->GetItemText(i));
		if(fn2.SameAs(fn))
			return;
#endif
	}

	if(fn.FileExists())
	{
		ImageInfo imageInfo;
		imageInfo.filename_ = filename;
		if(addToImageList)
			imageList_.push_back(imageInfo);

		pImageListCtrl_->InsertItem(newId, filename);

		if(pImageInfo != NULL)
		{
			wxString imageSizeStr = wxString::Format(wxT("%d x %d"), pImageInfo->imageWidth_, pImageInfo->imageHeight_);
			pImageListCtrl_->SetItem(newId, 1, imageSizeStr);
			pImageListCtrl_->SetItem(newId, 2, pImageInfo->cameraMaker_);
			pImageListCtrl_->SetItem(newId, 3, pImageInfo->cameraModel_);

			wxString focalLengthStr(wxT("N/A"));
			if(pImageInfo->focalLength_ > 0)
				focalLengthStr = wxString::Format(wxT("%.3g mm"), pImageInfo->focalLength_);
			pImageListCtrl_->SetItem(newId, 4, focalLengthStr);

			wxString sensorWidthStr(wxT("N/A"));
			if(pImageInfo->sensorWidth_ > 0)
				sensorWidthStr = wxString::Format(wxT("%.3g mm"), pImageInfo->sensorWidth_);
			pImageListCtrl_->SetItem(newId, 5, sensorWidthStr);
		}
		else
		{
			pImageInfoThread_->addImageInfoRequest(filename);
			sentImageInfoRequests_++;
		}
		pImageListCtrl_->SetColumnWidth(0, wxLIST_AUTOSIZE);
		pImageListCtrl_->SetColumnWidth(1, wxLIST_AUTOSIZE);
		pImageListCtrl_->SetColumnWidth(2, wxLIST_AUTOSIZE);
		pImageListCtrl_->SetColumnWidth(3, wxLIST_AUTOSIZE);
		pImageListCtrl_->SetColumnWidth(4, wxLIST_AUTOSIZE);
		pImageListCtrl_->SetColumnWidth(5, wxLIST_AUTOSIZE);

	}
}

void Regard3DPictureSetDialog::addDragAndDropFiles(const wxArrayString &filenames)
{
	if(showOnly_)
		return;

	for(size_t i = 0; i < filenames.GetCount(); i++)
	{
		addFileToImageList(filenames[i], true);
	}
}

int Regard3DPictureSetDialog::updateProject(R3DProject *pProject)
{
	R3DProject::PictureSet ps;
	ps.imageList_ = imageList_;
	wxString name = pPictureSetNameTextCtrl_->GetValue();
	ps.name_ = name;

	int psid = 0;
	if(pPictureSet_ == NULL)
	{
		// Add mode
		if(pProject != NULL)
			psid = pProject->addPictureSet(ps);
	}
	else
	{
		// Edit mode
		if(pProject != NULL)
			pProject->updatePictureSet(ps, pPictureSet_);

		psid = ps.id_;
	}

	return psid;
}

void Regard3DPictureSetDialog::OnPreviewFinished()
{
	checkForPreviewImage();
}

void Regard3DPictureSetDialog::OnNewImageInfos()
{
	checkForNewImageInfos();
}

void Regard3DPictureSetDialog::checkForNewImageInfos()
{
	// Check for image info thread
	wxString imageInfoFilename;
	ImageInfo imageInfo;
	while(pImageInfoThread_->getNewImageInfo(imageInfoFilename, imageInfo))
	{
		receivedImageInfos_++;
		long itemCount = pImageListCtrl_->GetItemCount();
		for(long i = 0; i < itemCount; i++)
		{
			wxString listString = pImageListCtrl_->GetItemText(i);
			if(listString.IsSameAs(imageInfoFilename))
			{
				wxString imageSizeStr = wxString::Format(wxT("%d x %d"), imageInfo.imageWidth_, imageInfo.imageHeight_);
				pImageListCtrl_->SetItem(i, 1, imageSizeStr);
				pImageListCtrl_->SetItem(i, 2, imageInfo.cameraMaker_);
				pImageListCtrl_->SetItem(i, 3, imageInfo.cameraModel_);

				wxString focalLengthStr(wxT("N/A"));
				if(imageInfo.focalLength_ > 0)
					focalLengthStr = wxString::Format(wxT("%.3g mm"), imageInfo.focalLength_);
				pImageListCtrl_->SetItem(i, 4, focalLengthStr);

				wxString sensorWidthStr(wxT("N/A"));
				if(imageInfo.sensorWidth_ > 0)
					sensorWidthStr = wxString::Format(wxT("%.3g mm"), imageInfo.sensorWidth_);
				pImageListCtrl_->SetItem(i, 5, sensorWidthStr);
			}
		}

		pImageListCtrl_->SetColumnWidth(0, wxLIST_AUTOSIZE);
		pImageListCtrl_->SetColumnWidth(1, wxLIST_AUTOSIZE);
		pImageListCtrl_->SetColumnWidth(2, wxLIST_AUTOSIZE);
		pImageListCtrl_->SetColumnWidth(3, wxLIST_AUTOSIZE);
		pImageListCtrl_->SetColumnWidth(4, wxLIST_AUTOSIZE);
		pImageListCtrl_->SetColumnWidth(5, wxLIST_AUTOSIZE);

		// Update entry in imageList_

		std::vector<ImageInfo>::iterator iter = imageList_.begin();
		while(iter != imageList_.end())
		{
			ImageInfo &ii = (*iter);
			if(ii.filename_.IsSameAs(imageInfoFilename))
			{
				ii.imageWidth_ = imageInfo.imageWidth_;
				ii.imageHeight_ = imageInfo.imageHeight_;
				ii.cameraMaker_ = imageInfo.cameraMaker_;
				ii.cameraModel_ = imageInfo.cameraModel_;
				ii.focalLength_ = imageInfo.focalLength_;
				ii.sensorWidth_ = imageInfo.sensorWidth_;
			}
			iter++;
		}

	}
}

void Regard3DPictureSetDialog::checkForPreviewImage()
{
	if(pPreviewGeneratorThread_ == NULL)
		return;

	wxImage previewImage;
	PreviewInfo previewInfo;
	if(pPreviewGeneratorThread_->getNewPreviewImage(previewImage, previewInfo))
	{
		pPreviewCanvas_->previewImage(previewImage);
		pPreviewCanvas_->Refresh();
	}
}

BEGIN_EVENT_TABLE( Regard3DPictureSetDialog, Regard3DPictureSetDialogBase )
	EVT_TIMER(ID_R3D_PSD_TIMER, Regard3DPictureSetDialog::OnTimer)
END_EVENT_TABLE()
