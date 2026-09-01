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
#include "Regard3DProgressDialog.h"
#include "Regard3DMainFrame.h"
#include "Regard3DConsoleOutputFrame.h"

#include <wx/utils.h>

enum
{
	ID_PROGRESS_DLG_TIMER = 4100
};

Regard3DProgressDialog::Regard3DProgressDialog(wxWindow *parent, const wxString &title)
	: Regard3DProgressDialogBase(parent, wxID_ANY, title),
	aTimer_(this, ID_PROGRESS_DLG_TIMER),
	startTime_(wxDateTime::Now()), abortRequested_(false),
	pMainFrame_(NULL), pConsoleOutputFrame_(NULL)
{
	pProgressGauge_->SetRange(100);

	// Disable the main window, but not the console window
	wxWindow *pNonDisableWindow = NULL;
	if(parent != NULL)
	{
		Regard3DMainFrame *pMainFrame = dynamic_cast<Regard3DMainFrame*>(parent);
		if(pMainFrame != NULL)
		{
			Regard3DConsoleOutputFrame *pConsoleOutputFrame = pMainFrame->getConsoleOutputFrame();
			pNonDisableWindow = pConsoleOutputFrame;

			// Store for later
			pMainFrame_ = pMainFrame;
			pConsoleOutputFrame_ = pConsoleOutputFrame;
		}
	}

	// Unfortunately, this has a bug on OS X/Cocoa: When the non-disable window
	// gets hidden, the disable becomes inactive and the progress window
	// will end up behind the main frame if the user clicks into it.
	// Workaround: Disable all windows, including the console output
#if defined(R3D_MACOSX)
	pWindowDisabler_ = new wxWindowDisabler();
#else
	pWindowDisabler_ = new wxWindowDisabler(pNonDisableWindow);
#endif


	if(pConsoleOutputFrame_ != NULL)
	{
		// Disable button if console output frame is visible
		if(pConsoleOutputFrame_->IsShown())
		{
			pShowOutputWindowButton_->Disable();
		}
	}

	aTimer_.Start(100);
}

Regard3DProgressDialog::~Regard3DProgressDialog()
{
	if(pWindowDisabler_ != NULL)
	{
		delete pWindowDisabler_;
		pWindowDisabler_ = NULL;
	}
}

void Regard3DProgressDialog::Update(int value, const wxString &newmsg)
{
	if(!newmsg.IsEmpty() && !abortRequested_)
	{
		pProgressStatusTextCtrl_->SetValue(newmsg);
		this->Fit();
		this->Layout();
	}
	pProgressGauge_->SetValue(value);
}

void Regard3DProgressDialog::Pulse(const wxString &newmsg)
{
	if(!newmsg.IsEmpty() && !abortRequested_)
	{
		pProgressStatusTextCtrl_->SetValue(newmsg);
		this->Fit();
		this->Layout();
	}
	pProgressGauge_->Pulse();
}

void Regard3DProgressDialog::EndDialog()
{
	if(pWindowDisabler_ != NULL)
	{
		delete pWindowDisabler_;
		pWindowDisabler_ = NULL;
	}

	if(this->IsModal())
		EndModal(0);
	else
		Close();
}

void Regard3DProgressDialog::OnShowOutputWindowButton( wxCommandEvent& event )
{
	if(pConsoleOutputFrame_ != NULL)
	{
		pConsoleOutputFrame_->Show();
		pShowOutputWindowButton_->Disable();
	}
}

/**
 * Stops the external program of the running step.
 *
 * The dialog stays open: the program takes a moment to die, and the step
 * closes the dialog itself once it has reported back.
 */
void Regard3DProgressDialog::OnAbortButton( wxCommandEvent& event )
{
	if(pMainFrame_ == NULL || abortRequested_)
		return;

	abortRequested_ = true;
	pAbortButton_->Disable();
	pProgressStatusTextCtrl_->SetValue(wxT("Aborting..."));

	// Nothing below this line: the step may end while we are still in here,
	// and this dialog is deleted when it does
	pMainFrame_->abortExternalProcess();
}

void Regard3DProgressDialog::OnTimer( wxTimerEvent &event )
{
	// Update elapsed time
	wxTimeSpan runTime = wxDateTime::Now() - startTime_;
	wxString runTimeStr;
	if(runTime.GetHours() > 0)
		runTimeStr = runTime.Format(wxT("%H:%M:%S"));
	else
		runTimeStr = runTime.Format(wxT("%M:%S"));

	wxString origStr = pElapsedTimeTextCtrl_->GetValue();
	if(!origStr.IsSameAs(runTimeStr))		// Compare strings to avoid flickering
	{
		pElapsedTimeTextCtrl_->SetValue(runTimeStr);
	}

	// Aborting is only possible while an external program is running.
	// The built-in engines are threads, which cannot be stopped halfway.
	if(pMainFrame_ != NULL)
	{
		const bool canAbort = (!abortRequested_
			&& pMainFrame_->isExternalProcessRunning());
		if(canAbort != pAbortButton_->IsEnabled())
			pAbortButton_->Enable(canAbort);
	}

	// Update pShowOutputWindowButton_
	if(pConsoleOutputFrame_ != NULL)
	{
		// Re-enable button if console output frame was closed by user
		if(!pConsoleOutputFrame_->IsShown()
			&& !pShowOutputWindowButton_->IsEnabled())
		{
			pShowOutputWindowButton_->Enable();
		}
	}
}

BEGIN_EVENT_TABLE( Regard3DProgressDialog, Regard3DProgressDialogBase )
	EVT_TIMER(ID_PROGRESS_DLG_TIMER, Regard3DProgressDialog::OnTimer)
END_EVENT_TABLE()
