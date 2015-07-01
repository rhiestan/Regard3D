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

#ifndef REGARD3DCONSOLEOUTPUTFRAMEBASE_H 
#define REGARD3DCONSOLEOUTPUTFRAMEBASE_H

class wxStreamToTextRedirector;
class StdOutRedirect;
class minilog_mutex_impl_wx;

#include "Regard3DMainFrameBase.h"

#if defined(_WIN32)
// Does not seem to work with MinGW
//#define USE_STDOUT_REDIRECT
#endif
// Works, but is extremely slow
//#define USE_STREAM_TO_TEXT_REDIRECT

#include <wx/minifram.h>

class Regard3DConsoleOutputFrame: public wxMiniFrame //public Regard3DConsoleOutputFrameBase
{
public:
	Regard3DConsoleOutputFrame(wxWindow* parent);
	virtual ~Regard3DConsoleOutputFrame();

protected:
	virtual void OnConsoleOutputClose( wxCloseEvent& event );
	virtual void OnClearConsoleOutputButton( wxCommandEvent& event );

	virtual void OnTimer( wxTimerEvent &event );

private:
#if defined(USE_STREAM_TO_TEXT_REDIRECT)
	wxStreamToTextRedirector *pStreamToTextRedirectorStdOut_;
	wxStreamToTextRedirector *pStreamToTextRedirectorStdErr_;
#endif
#if defined(USE_STDOUT_REDIRECT) && defined(_WIN32)
	StdOutRedirect *pStdOutRedirect_;
#endif
	minilog_mutex_impl_wx *pminilog_mutex_impl_wx_;

	wxTimer aTimer_;

	wxPanel* pMainPanel_;
	wxTextCtrl* pConsoleOutputTextCtrl_;
	wxButton* pClearConsoleOutputButton_;

	DECLARE_EVENT_TABLE()
};

#endif
