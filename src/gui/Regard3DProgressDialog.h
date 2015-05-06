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
#ifndef REGARD3DPROGRESSDIALOG_H
#define REGARD3DPROGRESSDIALOG_H

class wxWindowDisabler;
class Regard3DMainFrame;
class Regard3DConsoleOutputFrame;

#include "Regard3DMainFrameBase.h"

class Regard3DProgressDialog: public Regard3DProgressDialogBase
{
public:
	Regard3DProgressDialog(wxWindow *parent, const wxString &title);
	virtual ~Regard3DProgressDialog();

	void Update(int value, const wxString &newmsg = wxEmptyString);
	void Pulse(const wxString &newmsg = wxEmptyString);
	void EndDialog();

protected:
	virtual void OnShowOutputWindowButton( wxCommandEvent& event );

	virtual void OnTimer( wxTimerEvent &event );

private:
	wxWindowDisabler *pWindowDisabler_;
	wxTimer aTimer_;

	wxDateTime startTime_;
	Regard3DMainFrame *pMainFrame_;
	Regard3DConsoleOutputFrame *pConsoleOutputFrame_;

	DECLARE_EVENT_TABLE()
};

#endif
