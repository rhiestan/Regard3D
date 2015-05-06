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
#include "Regard3DConsoleOutputFrame.h"

#include "minilog/minilog.h"

class minilog_mutex_impl_wx: public minilog_mutex_interface
{
public:
	minilog_mutex_impl_wx(): minilog_mutex_interface()
	{
	}

	virtual ~minilog_mutex_impl_wx()
	{
	}

	virtual void lock()
	{
		mutex_.Lock();
	}

	virtual void unlock()
	{
		mutex_.Unlock();
	}

private:
	wxMutex mutex_;
};

enum
{
	ID_CONSOLE_WIN_TIMER = 3500,
	ID_REGARD3DCONSOLEOUTPUTFRAMEBASE,
	ID_CONSOLEOUTPUTTEXTCTRL,
	ID_CLEARCONSOLEOUTPUTBUTTON
};
/*
#define BUFFER_SIZE 1024

#if defined(_WIN32)
// The following code is taken from here:
// http://stackoverflow.com/questions/573724/how-can-i-redirect-stdout-to-some-visible-display-in-a-windows-application

#include <windows.h>

#include <stdio.h>
#include <fcntl.h>
#include <io.h>
#include <iostream>

#ifndef _USE_OLD_IOSTREAMS
using namespace std;
#endif

#define READ_FD 0
#define WRITE_FD 1

#define CHECK(a) if ((a)!= 0) return -1;

class StdOutRedirect
{
public:
	StdOutRedirect(int bufferSize);
	~StdOutRedirect();

	int Start();
	int Stop();
	int GetBuffer(char *buffer, int size);

private:
	int fdStdOutPipe[2];
	int fdStdOut;
};

StdOutRedirect::~StdOutRedirect()
{
	_close(fdStdOut);
	_close(fdStdOutPipe[WRITE_FD]);
	_close(fdStdOutPipe[READ_FD]);
}

StdOutRedirect::StdOutRedirect(int bufferSize)
{
	if (_pipe(fdStdOutPipe, bufferSize, O_TEXT)!=0)
	{
		//treat error eventually
	}
	fdStdOut = _dup(_fileno(stdout));
}

int StdOutRedirect::Start()
{
	fflush( stdout );
	CHECK(_dup2(fdStdOutPipe[WRITE_FD], _fileno(stdout)));
	ios::sync_with_stdio();
	setvbuf( stdout, NULL, _IONBF, 0 ); // absolutely needed
	return 0;
}

int StdOutRedirect::Stop()
{
	CHECK(_dup2(fdStdOut, _fileno(stdout)));
	ios::sync_with_stdio();
	return 0;
}

int StdOutRedirect::GetBuffer(char *buffer, int size)
{
	HANDLE rdHandle = (HANDLE)_get_osfhandle( fdStdOutPipe[READ_FD] );
	if(rdHandle != INVALID_HANDLE_VALUE)
	{
		DWORD totalBytesAvailable = 0;
		BOOL retVal = PeekNamedPipe(rdHandle, NULL, 0, NULL, &totalBytesAvailable, NULL);
		if(retVal != 0		// Success
			&& totalBytesAvailable == 0)
		{
			return 0;
		}
	}

	int nOutRead = _read(fdStdOutPipe[READ_FD], buffer, size);
	buffer[nOutRead] = '\0';
	return nOutRead;
}
#endif
*/

Regard3DConsoleOutputFrame::Regard3DConsoleOutputFrame(wxWindow* parent)
//	: Regard3DConsoleOutputFrameBase(parent),
	: wxMiniFrame(parent, wxID_ANY, wxT("Console output"), wxDefaultPosition,
		wxDefaultSize, wxCAPTION | wxRESIZE_BORDER | wxCLOSE_BOX),
	pStreamToTextRedirectorStdOut_(NULL), pStreamToTextRedirectorStdErr_(NULL),
	pStdOutRedirect_(NULL),
	aTimer_(this, ID_CONSOLE_WIN_TIMER)
{
/*#if defined(_WIN32)
	pStdOutRedirect_ = new StdOutRedirect(BUFFER_SIZE);
	pStdOutRedirect_->Start();
	aTimer_.Start(100);
#else*/
//	pStreamToTextRedirectorStdOut_ = new wxStreamToTextRedirector(pConsoleOutputTextCtrl_, &(std::cout));
//	pStreamToTextRedirectorStdErr_ = new wxStreamToTextRedirector(pConsoleOutputTextCtrl_, &(std::cerr));
//#endif

	SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer14;
	bSizer14 = new wxBoxSizer( wxVERTICAL );

	pMainPanel_ = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer54;
	bSizer54 = new wxBoxSizer( wxVERTICAL );

	pConsoleOutputTextCtrl_ = new wxTextCtrl( pMainPanel_, ID_CONSOLEOUTPUTTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxHSCROLL|wxTE_MULTILINE|wxTE_READONLY );
	pConsoleOutputTextCtrl_->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 76, 90, 90, false, wxEmptyString ) );

	bSizer54->Add( pConsoleOutputTextCtrl_, 1, wxALL|wxEXPAND, 3 );

	wxBoxSizer* bSizer15;
	bSizer15 = new wxBoxSizer( wxHORIZONTAL );
	bSizer15->Add( 0, 0, 1, wxEXPAND, 5 );

	pClearConsoleOutputButton_ = new wxButton( pMainPanel_, ID_CLEARCONSOLEOUTPUTBUTTON, wxT("Clear"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer15->Add( pClearConsoleOutputButton_, 0, wxALL, 3 );

	bSizer54->Add( bSizer15, 0, wxEXPAND, 5 );

	pMainPanel_->SetSizer( bSizer54 );
	pMainPanel_->Layout();
	bSizer54->Fit( pMainPanel_ );
	bSizer14->Add( pMainPanel_, 1, wxEXPAND, 5 );

	this->SetSizer( bSizer14 );
	this->Layout();
	this->Centre( wxBOTH );

	pminilog_mutex_impl_wx_ = new minilog_mutex_impl_wx();
	minilog::inst().set_locker(pminilog_mutex_impl_wx_);
	aTimer_.Start(100);
}

Regard3DConsoleOutputFrame::~Regard3DConsoleOutputFrame()
{
/*#if defined(_WIN32)
	if(pStdOutRedirect_ != NULL)
		pStdOutRedirect_->Stop();
	delete pStdOutRedirect_;
#else*/
//	delete pStreamToTextRedirectorStdOut_;
//	delete pStreamToTextRedirectorStdErr_;
//#endif

	minilog::inst().set_locker(NULL);
	delete pminilog_mutex_impl_wx_;
}

void Regard3DConsoleOutputFrame::OnConsoleOutputClose( wxCloseEvent& event )
{
	Hide();
}

void Regard3DConsoleOutputFrame::OnClearConsoleOutputButton( wxCommandEvent& event )
{
	pConsoleOutputTextCtrl_->Clear();
}

void Regard3DConsoleOutputFrame::OnTimer( wxTimerEvent &event )
{
/*	char buf[BUFFER_SIZE+1];	// Not necessarily same BUFFER_SIZE as in new StdOutRedirect(BUFFER_SIZE);
	int nOutRead = pStdOutRedirect_->GetBuffer(buf, BUFFER_SIZE);
	if(nOutRead > 0)
		pConsoleOutputTextCtrl_->AppendText(wxString(buf, *wxConvCurrent));*/

	std::string str = minilog::inst().getbuf();
	if(!str.empty())
	{
		pConsoleOutputTextCtrl_->AppendText(wxString(str.c_str(), *wxConvCurrent));
	}
}

BEGIN_EVENT_TABLE( Regard3DConsoleOutputFrame, wxMiniFrame)	//Regard3DConsoleOutputFrameBase )
	EVT_TIMER(ID_CONSOLE_WIN_TIMER, Regard3DConsoleOutputFrame::OnTimer)
	EVT_CLOSE( Regard3DConsoleOutputFrame::OnConsoleOutputClose )
	EVT_BUTTON( ID_CLEARCONSOLEOUTPUTBUTTON, Regard3DConsoleOutputFrame::OnClearConsoleOutputButton )

END_EVENT_TABLE()
