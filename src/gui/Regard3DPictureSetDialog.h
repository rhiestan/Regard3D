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
#ifndef REGARD3DPICTURESETDIALOG_H
#define REGARD3DPICTURESETDIALOG_H

class PreviewGeneratorThread;
class ImageInfoThread;
class Regard3DPictureSetDlgDropTarget;


#include "Regard3DMainFrameBase.h"
#include "R3DImageUpdatesInterface.h"
#include "ImageInfo.h"
#include "R3DProject.h"

class Regard3DPictureSetDialog: public Regard3DPictureSetDialogBase, public R3DImageUpdatesInterface
{
public:
	Regard3DPictureSetDialog(wxWindow *pParent);
	virtual ~Regard3DPictureSetDialog();

	void setPreviewGeneratorThread(PreviewGeneratorThread *pPreviewGeneratorThread);
	void setImageInfoThread(ImageInfoThread *pImageInfoThread);
	void setPictureSet(R3DProject::PictureSet *pPictureSet);
	void addDragAndDropFiles(const wxArrayString &filenames);
	int updateProject(R3DProject *pProject);

	virtual void OnPreviewFinished();
	virtual void OnNewImageInfos();

	virtual void EndModal(int retCode);

protected:
	virtual void OnInitDialog( wxInitDialogEvent& event );
	virtual void OnAddFilesButton( wxCommandEvent& event );
	virtual void OnRemoveSelectedFilesButton( wxCommandEvent& event );
	virtual void OnClearFileListButton( wxCommandEvent& event );
	virtual void OnImageListColClick( wxListEvent& event );
	virtual void OnImageListItemDeselected( wxListEvent& event );
	virtual void OnImageListItemSelected( wxListEvent& event );
	virtual void OnImageListKeyDown( wxListEvent& event );
	virtual void OnCancel( wxCommandEvent& event );
	virtual void OnOK( wxCommandEvent& event );

	virtual void OnTimer( wxTimerEvent &event );

	void removeSelectedItems( wxListCtrl *pListCtrl );
	void addFileToImageList(const wxString &filename, bool addToImageList, const ImageInfo *pImageInfo = NULL);
	void checkForNewImageInfos();
	void checkForPreviewImage();

private:
	wxTimer aTimer_;
	PreviewGeneratorThread *pPreviewGeneratorThread_;
	ImageInfoThread *pImageInfoThread_;
	Regard3DPictureSetDlgDropTarget *pRegard3DPictureSetDlgDropTarget_;
	R3DProject::PictureSet *pPictureSet_;
	bool showOnly_;

	int sentImageInfoRequests_, receivedImageInfos_;

	ImageInfoVector imageList_;

	DECLARE_EVENT_TABLE()
};

class Regard3DPictureSetDlgDropTarget: public wxFileDropTarget
{
public:
	Regard3DPictureSetDlgDropTarget() : pDialog_(NULL) { }
	virtual ~Regard3DPictureSetDlgDropTarget() { }
	void setDialog(Regard3DPictureSetDialog *pDialog) { pDialog_ = pDialog; }

	virtual bool OnDropFiles(wxCoord x, wxCoord y, const wxArrayString &filenames)
	{
		if(pDialog_ != NULL)
			pDialog_->addDragAndDropFiles(filenames);

		return true;
	}

private:
	Regard3DPictureSetDialog *pDialog_;
};


#endif
