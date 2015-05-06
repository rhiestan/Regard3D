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
#include "Regard3DImagePreviewDialog.h"
#include "PreviewGeneratorThread.h"
#include "R3DProject.h"
#include "Regard3DSettings.h"

#include <cmath>

// For double buffering
#include <wx/dcbuffer.h>

const double Regard3DImagePreviewDialog::zoomChange_ = 1.1892071150027210667174999705605;		// sqrt(sqrt(2))

enum
{
	ID_R3D_IPD_TIMER = 3700
};

Regard3DImagePreviewDialog::Regard3DImagePreviewDialog(wxWindow *pParent)
	: Regard3DImagePreviewDialogBase(pParent),
	aTimer_(this, ID_R3D_IPD_TIMER),
	pPreviewGeneratorThread_(NULL),
	pProject_(NULL), pComputeMatches_(NULL),
	pPictureSet_(NULL)
{
	wxAcceleratorEntry entries[3];
	entries[0].Set(wxACCEL_NORMAL, WXK_ESCAPE, ID_CLOSEBUTTON);
	entries[1].Set(wxACCEL_NORMAL, WXK_NUMPAD_ADD, ID_ZOOMINBUTTON);
	entries[2].Set(wxACCEL_NORMAL, WXK_NUMPAD_SUBTRACT, ID_ZOOMOUTBUTTON);
	wxAcceleratorTable accel(3, entries);
	
	SetAcceleratorTable(accel);

	aTimer_.Start(50);
}

Regard3DImagePreviewDialog::~Regard3DImagePreviewDialog()
{
}

void Regard3DImagePreviewDialog::setImage(const wxImage &img)
{
	pImagePanel_->setImage(img);
}

void Regard3DImagePreviewDialog::setPreviewInfo(const PreviewInfo &previewInfo)
{
	previewInfo_ = previewInfo;
}

void Regard3DImagePreviewDialog::setPreviewGeneratorThread(PreviewGeneratorThread *pPreviewGeneratorThread)
{
	pPreviewGeneratorThread_ = pPreviewGeneratorThread;
}

void Regard3DImagePreviewDialog::setComputeMatches(R3DProject *pProject, R3DProject::ComputeMatches *pComputeMatches)
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

void Regard3DImagePreviewDialog::OnClose( wxCloseEvent& event )
{
	Regard3DSettings::getInstance().saveImagePreviewLayoutToConfig(this);

	event.Skip();	// The event is not processed, continue closing the dialog
}

void Regard3DImagePreviewDialog::OnInitDialog( wxInitDialogEvent& event )
{
	pImagePanel_->setZoomFactor(1.0);
	updateZoomFactor(1.0);

	pKeypointTypeChoice_->Clear();
	pKeypointTypeChoice_->Append(wxT("No keypoints"));
	pKeypointTypeChoice_->Append(wxT("Simple keypoints"));
	pKeypointTypeChoice_->Append(wxT("Rich keypoints"));
	if(previewInfo_.keypointType_ == PreviewInfo::PIKPTNoKeypoints)
		pKeypointTypeChoice_->Select(0);
	else
		pKeypointTypeChoice_->Select(2);

	if(previewInfo_.type_ == PreviewInfo::PITKeypoints)
	{
		pShowSingleKeypointsCheckBox_->Hide();
		pMatchesChoice_->Hide();
		pEnableTrackFilterCheckBox_->Hide();
	}
	else
	{
		pShowSingleKeypointsCheckBox_->SetValue(false);
		pMatchesChoice_->Clear();
		pMatchesChoice_->Append(wxT("Putative matches"));
		pMatchesChoice_->Append(wxT("Homographic matches"));
		pMatchesChoice_->Append(wxT("Fundamental matches"));
		if(wxFileName::FileExists( wxString( paths_.matchesEFilename_.c_str(), *wxConvCurrent ) ))
			pMatchesChoice_->Append(wxT("Essential matches"));
		pMatchesChoice_->Select(2);
		pEnableTrackFilterCheckBox_->SetValue(false);
	}

	Regard3DSettings::getInstance().loadImagePreviewLayoutFromConfig(this);
	pImagePreviewDialogPanel_->Layout();
}

void Regard3DImagePreviewDialog::OnKeypointTypeChoice( wxCommandEvent& event )
{
	if(pPreviewGeneratorThread_ == NULL)
		return;

	if(event.GetInt() == 2)
		previewInfo_.keypointType_ = PreviewInfo::PIKPTRichKeypoints;
	else if(event.GetInt() == 1)
		previewInfo_.keypointType_ = PreviewInfo::PIKPTSimpleKeypoints;
	else
		previewInfo_.keypointType_ = PreviewInfo::PIKPTNoKeypoints;
	pPreviewGeneratorThread_->addPreviewRequest(previewInfo_);
}

void Regard3DImagePreviewDialog::OnShowSingleKeypointsCheckBox( wxCommandEvent& event )
{
	if(pPreviewGeneratorThread_ == NULL)
		return;

	previewInfo_.showSingleKeypoints_ = (event.IsChecked());
	pPreviewGeneratorThread_->addPreviewRequest(previewInfo_);
}

void Regard3DImagePreviewDialog::OnMatchesChoice( wxCommandEvent& event )
{
	if(pPreviewGeneratorThread_ == NULL)
		return;

	if(event.GetInt() == 0)			// Putative
		previewInfo_.matchesFilename_ = wxString( paths_.matchesPutitativeFilename_.c_str(), *wxConvCurrent );
	else if(event.GetInt() == 1)	// Homographic
		previewInfo_.matchesFilename_ = wxString( paths_.matchesHFilename_.c_str(), *wxConvCurrent );
	else if(event.GetInt() == 2)	// Fundamental
		previewInfo_.matchesFilename_ = wxString( paths_.matchesFFilename_.c_str(), *wxConvCurrent );
	else if(event.GetInt() == 3)	// Essential
		previewInfo_.matchesFilename_ = wxString( paths_.matchesEFilename_.c_str(), *wxConvCurrent );

	pPreviewGeneratorThread_->addPreviewRequest(previewInfo_);
}

void Regard3DImagePreviewDialog::OnEnableTrackFilter( wxCommandEvent& event )
{
	previewInfo_.enableTrackFilter_ = event.IsChecked();
	pPreviewGeneratorThread_->addPreviewRequest(previewInfo_);
}

void Regard3DImagePreviewDialog::OnZoomInButton( wxCommandEvent& event )
{
	double zoomFactor = pImagePanel_->getZoomFactor();
	zoomFactor *= zoomChange_;

	pImagePanel_->setZoomFactor(zoomFactor);
	updateZoomFactor(zoomFactor);
}

void Regard3DImagePreviewDialog::OnZoomOutButton( wxCommandEvent& event )
{
	double zoomFactor = pImagePanel_->getZoomFactor();
	zoomFactor /= zoomChange_;
	pImagePanel_->setZoomFactor(zoomFactor);
	updateZoomFactor(zoomFactor);
}

void Regard3DImagePreviewDialog::OnCloseButton( wxCommandEvent& event )
{
	Close();
}

void Regard3DImagePreviewDialog::OnMouseWheel(wxMouseEvent& event)
{
	int wheelRotation = event.GetWheelRotation();
	if(wheelRotation != 0
#if wxCHECK_VERSION(2, 9, 0)
		&& !event.HasModifiers())
#else
		&& !event.ShiftDown()
		&& !event.AltDown()
		&& !event.ControlDown())
#endif
	{
		double zoomFactor = pImagePanel_->getZoomFactor();

		if(wheelRotation > 0)
			zoomFactor *= zoomChange_;
		else
			zoomFactor /= zoomChange_;
		pImagePanel_->setZoomFactor(zoomFactor, event.GetX(), event.GetY(), (wheelRotation > 0));
		updateZoomFactor(zoomFactor);

		event.StopPropagation();	// Does not help, unfortunately
		event.Skip(false);			// See R3D_USE_SCROLLBAR_MOUSEWHEEL_WORKAROUND
	}
}

void Regard3DImagePreviewDialog::OnTimer( wxTimerEvent &WXUNUSED(event) )
{
	if(pPreviewGeneratorThread_ == NULL)
		return;

	wxImage previewImage;
	PreviewInfo previewInfo;
	if(pPreviewGeneratorThread_->getNewPreviewImage(previewImage, previewInfo))
	{
		pImagePanel_->setImage(previewImage);
		pImagePanel_->Refresh();
	}
}

void Regard3DImagePreviewDialog::updateZoomFactor(double zoomFactor)
{
	wxString zoomFactorStr = wxString::Format(wxT("%.1f %%"), 100.0 * zoomFactor);
	pZoomFactorTextCtrl_->SetValue(zoomFactorStr);
}

BEGIN_EVENT_TABLE(Regard3DImagePreviewDialog, Regard3DImagePreviewDialogBase)
	EVT_MOUSEWHEEL(Regard3DImagePreviewDialog::OnMouseWheel)
	EVT_TIMER(ID_R3D_IPD_TIMER, Regard3DImagePreviewDialog::OnTimer)
END_EVENT_TABLE()
