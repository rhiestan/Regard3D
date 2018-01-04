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
#include "R3DExternalPrograms.h"
#include "Regard3DSettings.h"

#include <wx/stdpaths.h>

R3DExternalPrograms R3DExternalPrograms::instance_;


bool R3DExternalPrograms::initialize()
{
	if(!initialized_)
	{
		wxFileName exeFN(wxStandardPaths::Get().GetExecutablePath());
		exeFN.SetFullName(wxEmptyString);

		wxString configExePath = Regard3DSettings::getInstance().getExternalEXEPath();
		if(!configExePath.IsEmpty())
			exeFN.SetPath(configExePath);

		wxString executableExtension;
#if defined(R3D_WIN32)
		executableExtension = wxT("exe");
#endif
		bool isOK = true;
		allPaths_.Clear();
		wxFileName pmvsFN(exeFN);
		pmvsFN.AppendDir(wxT("pmvs"));
		if(pmvsFN.DirExists())
		{
			allPaths_.Add(pmvsFN.GetPath(wxPATH_GET_VOLUME));
			isOK &= checkExecutable(pmvsFN.GetPath(wxPATH_GET_VOLUME), wxT("pmvs2"), executableExtension, pmvsPath_);
			isOK &= checkExecutable(pmvsFN.GetPath(wxPATH_GET_VOLUME), wxT("cmvs"), executableExtension, cmvsPath_);
			isOK &= checkExecutable(pmvsFN.GetPath(wxPATH_GET_VOLUME), wxT("genOption"), executableExtension, genOptionPath_);
		}
		else
			isOK = false;

		wxFileName poissonFN(exeFN);
		poissonFN.AppendDir(wxT("poisson"));
		if(poissonFN.DirExists())
		{
			allPaths_.Add(poissonFN.GetPath(wxPATH_GET_VOLUME));
			isOK &= checkExecutable(poissonFN.GetPath(wxPATH_GET_VOLUME), wxT("PoissonRecon"), executableExtension, poissonReconPath_);
			isOK &= checkExecutable(poissonFN.GetPath(wxPATH_GET_VOLUME), wxT("SurfaceTrimmer"), executableExtension, surfaceTrimmerPath_);
		}
		else
			isOK = false;

		wxFileName mveFN(exeFN);
		mveFN.AppendDir(wxT("mve"));
		if(mveFN.DirExists())
		{
			allPaths_.Add(mveFN.GetPath(wxPATH_GET_VOLUME));
			isOK &= checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("makescene"), executableExtension, makescenePath_);
			isOK &= checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("texrecon"), executableExtension, texreconPath_);
			isOK &= checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("dmrecon"), executableExtension, dmreconPath_);
			isOK &= checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("scene2pset"), executableExtension, scene2psetPath_);
			isOK &= checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("fssrecon"), executableExtension, fssreconPath_);
			isOK &= checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("meshclean"), executableExtension, meshcleanPath_);
			isOK &= checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("smvsrecon"), executableExtension, smvsreconPath_);
			isOK &= checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("smvsrecon_SSE41"), executableExtension, smvsreconSSE41Path);
		}
		else
			isOK = false;

		wxFileName cmpmvsFN(exeFN);
		cmpmvsFN.AppendDir(wxT("cmpmvs"));
		if(cmpmvsFN.DirExists())
		{
			allPaths_.Add(cmpmvsFN.GetPath(wxPATH_GET_VOLUME));
			checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("CMPMVS"), executableExtension, cmpmvsPath_);
		}

		if(!isOK)
		{
			wxMessageBox(wxT("Third-party executables not found.\nPlease put them where the executable is located."),
				wxT("Third party executables not found"), wxICON_ERROR | wxOK, NULL);
		}

		initialized_ = true;
	}

	return true;
}

R3DExternalPrograms::R3DExternalPrograms()
	: initialized_(false)
{
}

R3DExternalPrograms::~R3DExternalPrograms()
{
}

bool R3DExternalPrograms::checkExecutable(const wxString &path, const wxString &name, const wxString &extension,
	wxString &outFullPath)
{
	wxFileName fn(path, name, extension);
	if(fn.IsFileExecutable())
	{
		outFullPath = fn.GetFullPath();
		return true;
	}

	return false;
}
	
