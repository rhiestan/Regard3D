/**
 * Copyright (C) 2026 Roman Hiestand
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
#include "R3DExportProcess.h"
#include "Regard3DMainFrame.h"
#include "R3DExternalPrograms.h"

#include <iostream>

R3DExportProcess::R3DExportProcess(Regard3DMainFrame *pMainFrame)
	: wxProcess(pMainFrame), pMainFrame_(pMainFrame),
	processId_(0), isOK_(true), wasCancelled_(false)
{
}

R3DExportProcess::~R3DExportProcess()
{
}

bool R3DExportProcess::runExportProcess(const R3DProjectPaths &paths, const wxString &name,
	const wxString &command, const wxString &expectedOutput)
{
	name_ = name;
	command_ = command;
	expectedOutput_ = expectedOutput;

#if wxCHECK_VERSION(2, 9, 0)
	env_.cwd = paths.absoluteProjectPath_;		// All paths are relative to project

	// Set PATH/LD_LIBRARY_PATH/DYLD_LIBRARY_PATH environment variable
#if defined(R3D_WIN32)
	wxString envVarName(wxT("PATH"));
#elif defined(R3D_MACOSX)
	wxString envVarName(wxT("DYLD_LIBRARY_PATH"));
#else
	wxString envVarName(wxT("LD_LIBRARY_PATH"));
#endif

	wxEnvVariableHashMap envVars;
	if(wxGetEnvMap(&envVars))
	{
		wxString concatPaths;
		const wxArrayString &exePaths = R3DExternalPrograms::getInstance().getAllPaths();
		for(size_t i = 0; i < exePaths.GetCount(); i++)
		{
			if(!concatPaths.IsEmpty())
#if defined(R3D_WIN32)
				concatPaths.Append(wxT(";"));
#else
				concatPaths.Append(wxT(":"));
#endif

			concatPaths.Append(exePaths[i]);
		}
		wxEnvVariableHashMap::iterator iter = envVars.find(envVarName);
		if(iter != envVars.end())
		{
			wxString val = iter->second;
			if(!val.IsEmpty())
#if defined(R3D_WIN32)
				val.Append(wxT(";"));
#else
				val.Append(wxT(":"));
#endif
			val.Append(concatPaths);
			envVars[envVarName] = val;
		}
		else
		{
			envVars[envVarName] = concatPaths;
		}
		env_.env = envVars;
	}
#endif

	Redirect();	// Redirect I/O, hide console window

#if wxCHECK_VERSION(2, 9, 0)
	processId_ = wxExecute(command_, wxEXEC_ASYNC, this, &env_);
#else
	processId_ = wxExecute(command_, wxEXEC_ASYNC, this);
#endif

	if(processId_ <= 0)
	{
		// OnTerminate is never called for a process that never started
		isOK_ = false;
		errorMessage_ = wxString::Format(wxT("The %s export could not be started."),
			name_.c_str());
		finish();
		return false;
	}

	return true;
}

void R3DExportProcess::readConsoleOutput()
{
	// Forwarded to std::cout/std::cerr, where the console output window picks
	// it up the same way as the OpenMVG library's own logging
	wxInputStream *pIn = GetInputStream();
	if(pIn != NULL)
	{
		std::string buf;
		while(pIn->CanRead())
		{
			int curc = pIn->GetC();
			if(curc != wxEOF)
				buf.push_back( static_cast<char>(curc) );
		}

		if(!buf.empty())
			std::cout << buf << std::flush;
	}
	wxInputStream *pErr = GetErrorStream();
	if(pErr != NULL)
	{
		std::string buf;
		while(pErr->CanRead())
		{
			int curc = pErr->GetC();
			if(curc != wxEOF)
				buf.push_back( static_cast<char>(curc) );
		}

		if(!buf.empty())
			std::cerr << buf << std::flush;
	}
}

void R3DExportProcess::cancel()
{
	if(wasCancelled_ || processId_ <= 0)
		return;

	wasCancelled_ = true;

	// wxKILL_CHILDREN in case the tool started helpers of its own
	wxProcess::Kill(processId_, wxSIGKILL, wxKILL_CHILDREN);
}

void R3DExportProcess::OnTerminate(int pid, int status)
{
	readConsoleOutput();	// Finish reading streams

	// This process is gone; cancel() must not kill a recycled pid
	processId_ = 0;

	if(wasCancelled_)
	{
		isOK_ = false;
		errorMessage_ = wxT("Aborted.");
	}
	else if(status != 0)
	{
		isOK_ = false;
		errorMessage_ = wxString::Format(
			wxT("The %s export returned with error code %d.\n\n")
			wxT("Please check the console output for details."), name_.c_str(), status);
	}
	else if(!expectedOutput_.IsEmpty()
		&& !wxFileName::Exists(expectedOutput_))
	{
		// Some of the tools report success and still write nothing, for
		// instance when a file they need beside their own executable is missing
		isOK_ = false;
		errorMessage_ = wxString::Format(
			wxT("The %s export wrote no\n%s\n\nPlease check the console output for details."),
			name_.c_str(), expectedOutput_.c_str());
	}

	finish();
}

void R3DExportProcess::finish()
{
	if(pMainFrame_ != NULL)
		pMainFrame_->sendExportFinishedEvent();
}
