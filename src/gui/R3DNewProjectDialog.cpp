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
#include "R3DNewProjectDialog.h"
#include "Regard3DSettings.h"

#include <wx/stdpaths.h>

R3DNewProjectDialog::R3DNewProjectDialog(wxWindow *parent)
	: NewProjectDialogBase(parent)
{
	pUseDefaultProjectPathRadioButton_->SetValue(true);
	pSetProjectPathRadioButton_->SetValue(false);

	defaultProjectDir_ = Regard3DSettings::getInstance().getDefaultProjectPath();
	if(defaultProjectDir_.IsEmpty())
#if wxCHECK_VERSION(2, 9, 0)
		defaultProjectDir_ = wxStandardPaths::Get().GetAppDocumentsDir();
#else
	{
		wxFileName dpp(wxStandardPaths::Get().GetDocumentsDir(), wxEmptyString);
		dpp.AppendDir(wxTheApp->GetAppName());
		defaultProjectDir_ = dpp.GetPath(wxPATH_GET_VOLUME);
	}
#endif

	wxFileName defaultDir(defaultProjectDir_, wxT(""));
	pProjectPathDirPicker_->SetPath(defaultDir.GetFullPath());
	pProjectPathDirPicker_->Disable();
#if defined(__WXOSX__)
	// Workaround for different behaviour of OS X port
	pProjectPathDirPicker_->GetTextCtrl()->Disable();
	pProjectPathDirPicker_->GetPickerCtrl()->Disable();
#endif
}

R3DNewProjectDialog::~R3DNewProjectDialog()
{
}

void R3DNewProjectDialog::OnUseDefaultProjectPathRadioButton( wxCommandEvent& event )
{
	wxFileName defaultDir(defaultProjectDir_, wxT(""));
	pProjectPathDirPicker_->SetPath(defaultDir.GetFullPath());
	updateProjectFilename();

	pSetProjectPathRadioButton_->SetValue(false);
	pProjectPathDirPicker_->Disable();
#if defined(__WXOSX__)
	// Workaround for different behaviour of OS X port
	pProjectPathDirPicker_->GetTextCtrl()->Disable();
	pProjectPathDirPicker_->GetPickerCtrl()->Disable();
#endif
}

void R3DNewProjectDialog::OnSetProjectPathRadioButton( wxCommandEvent& event )
{
	pUseDefaultProjectPathRadioButton_->SetValue(false);
	pProjectPathDirPicker_->Enable();
#if defined(__WXOSX__)
	// Workaround for different behaviour of OS X port
	pProjectPathDirPicker_->GetTextCtrl()->Enable();
	pProjectPathDirPicker_->GetPickerCtrl()->Enable();
#endif
}

void R3DNewProjectDialog::OnProjectPathChanged( wxFileDirPickerEvent& event )
{
	updateProjectFilename();
}

void R3DNewProjectDialog::OnProjectNameText( wxCommandEvent& event )
{
	updateProjectFilename();
}

void R3DNewProjectDialog::OnOK( wxCommandEvent& event )
{
	if(Validate() && TransferDataFromWindow())
	{
		bool isOK = updateProjectFilename();
		if(!isOK)
		{
			wxMessageBox(wxT("Illegal characters in project name!"),
				wxT("Error"), wxOK | wxICON_ERROR);
			return;
		}

		// Check project file name
		wxFileName projectFN(projectFilename_);
		if(projectFN.FileExists())
		{
			wxMessageBox(wxT("Project file name already exists!"),
				wxT("Error"), wxOK | wxICON_ERROR);
			return;
		}

		// Check folder path
		wxFileName projectPath(projectFilename_);
		projectPath.SetFullName(wxT(""));
		if(!projectPath.DirExists())
		{
			// Create folder if necessary
#if wxCHECK_VERSION(2, 9, 0)
			bool isOKMkdir = projectPath.Mkdir(wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL);
#else
			bool isOKMkdir = projectPath.Mkdir(0777, wxPATH_MKDIR_FULL);
#endif
			if(!isOKMkdir)
			{
				wxMessageBox(wxT("Error while creating project directory!"),
					wxT("Error"), wxOK | wxICON_ERROR);
				return;
			}
		}

		EndModal(wxID_OK);
	}
}

bool R3DNewProjectDialog::updateProjectFilename()
{
	if(!Validate() || !TransferDataFromWindow())
	{
		pProjectFilenameTextCtrl_->Clear();
		return false;
	}

	wxString pathStr = pProjectPathDirPicker_->GetPath();
	wxString projectName = pProjectNameTextCtrl_->GetValue();
	if(pathStr.IsEmpty()
		|| projectName.IsEmpty())
	{
		pProjectFilenameTextCtrl_->Clear();
		return false;
	}

	wxString forbiddenChar = wxFileName::GetForbiddenChars();
	forbiddenChar.Append(wxT(".$%~"));	// Also disallow the dot, as this is a relative dir
										// and some other special characters
	// Check projectName for forbidden characters
	for(size_t i = 0; i < projectName.Length(); i++)
	{
		wxChar ch = (wxChar)(projectName[i]);
		if(forbiddenChar.Find(ch) != wxNOT_FOUND)
		{
			pProjectFilenameTextCtrl_->Clear();
			return false;
		}
	}
	wxFileName projectFN(pathStr, projectName, wxT("r3d"));
	projectFN.AppendDir(projectName);
	projectFilename_ = projectFN.GetFullPath();
	pProjectFilenameTextCtrl_->SetValue(projectFilename_);

	return true;
}

BEGIN_EVENT_TABLE( R3DNewProjectDialog, NewProjectDialogBase )
END_EVENT_TABLE()

