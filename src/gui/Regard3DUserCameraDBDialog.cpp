/**
 * Copyright (C) 2017 Roman Hiestand
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
#include "Regard3DUserCameraDBDialog.h"
#include "UserCameraDB.h"
#include "Regard3DSettings.h"

Regard3DUserCameraDBDialog::Regard3DUserCameraDBDialog(wxWindow *pParent)
	: Regard3DUserCameraDBDialogBase(pParent)
{

}

Regard3DUserCameraDBDialog::~Regard3DUserCameraDBDialog()
{

}

void Regard3DUserCameraDBDialog::EndModal(int retCode)
{
	Regard3DSettings::getInstance().saveUserCameraDBLayoutToConfig(this);

	wxDialog::EndModal(retCode);
}

void Regard3DUserCameraDBDialog::OnInitDialog(wxInitDialogEvent& event)
{
	std::vector<std::tuple<wxString, wxString, double> > entries;
	bool isOk = UserCameraDB::getInstance().fetchAllEntries(entries);

	if(!isOk)
	{
		wxMessageBox(wxT("Error while reading the user camera database."),
			wxT("User camera database"), wxICON_WARNING, this);
		EndModal(wxID_CANCEL);
	}

	pUserCameraDBListCtrl_->ClearAll();
	pUserCameraDBListCtrl_->InsertColumn(0, wxT("Camera maker"));
	pUserCameraDBListCtrl_->InsertColumn(1, wxT("Camera model"));
	pUserCameraDBListCtrl_->InsertColumn(2, wxT("Sensor width"));

	int newId = 0;
	for(const auto &entry : entries)
	{
		pUserCameraDBListCtrl_->InsertItem(newId, std::get<0>(entry));

		pUserCameraDBListCtrl_->SetItem(newId, 1, std::get<1>(entry));

		wxString sensorWidthStr = wxString::Format(wxT("%.3g mm"), std::get<2>(entry));
		pUserCameraDBListCtrl_->SetItem(newId, 2, sensorWidthStr);
		newId++;
	}

	Regard3DSettings::getInstance().loadUserCameraDBLayoutFromConfig(this);
//	pUserCameraDBListCtrl_->SetColumnWidth(0, wxLIST_AUTOSIZE);
//	pUserCameraDBListCtrl_->SetColumnWidth(1, wxLIST_AUTOSIZE);
//	pUserCameraDBListCtrl_->SetColumnWidth(2, wxLIST_AUTOSIZE);
}

void Regard3DUserCameraDBDialog::OnDeleteSelectedEntryButtonClicked(wxCommandEvent& event)
{
	// Find selected entries
	std::vector<int> itemsToBeDeleted;
	int stateMask = wxLIST_STATE_SELECTED;
	int itemCount = pUserCameraDBListCtrl_->GetItemCount();
	for(int i = 0; i < itemCount; i++)
	{
		int itemState = pUserCameraDBListCtrl_->GetItemState(i, stateMask);
		if(itemState != 0)
		{
			// Store items to be deleted in an array
			itemsToBeDeleted.push_back(i);
		}
	}

	// First, convert size_t to signed int to allow - 1 if list is empty
	int itemsToBeDeletedSize = static_cast<int>(itemsToBeDeleted.size());

	if(itemsToBeDeletedSize == 0)
		return;		// Nothing to do

	// Ask user
	wxString queryString;
	if(itemsToBeDeletedSize > 1)
	{
		queryString = wxString::Format(wxT("Are you sure to delete %d entries in the user camera database?"), itemsToBeDeletedSize);
	}
	else
	{
		wxString cameraMaker = pUserCameraDBListCtrl_->GetItemText(itemsToBeDeleted[0]);
		wxString cameraModel = pUserCameraDBListCtrl_->GetItemText(itemsToBeDeleted[0], 1);
		queryString = wxString(wxT("Are you sure to delete the entry: ")) + cameraMaker
			+ wxString(wxT(" ")) + cameraModel + wxString(wxT("?"));
	}
	int retVal = wxMessageBox(queryString,
		wxT("User camera database"), wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION, this);
	if(retVal != wxYES)
		return;

	// Iterate list, start at the back
	for(int i = 0; i < itemsToBeDeletedSize; i++)
	{
		int itemToBeDeleted = itemsToBeDeleted[itemsToBeDeletedSize - 1 - i];

		// Delete entry in pUserCameraDBListCtrl_
		wxString cameraMaker = pUserCameraDBListCtrl_->GetItemText(itemToBeDeleted);
		wxString cameraModel = pUserCameraDBListCtrl_->GetItemText(itemToBeDeleted, 1);

		UserCameraDB::getInstance().removeFromDB(cameraMaker, cameraModel);
		pUserCameraDBListCtrl_->DeleteItem(itemToBeDeleted);
	}

}

void Regard3DUserCameraDBDialog::OnDeleteAllEntriesButtonClicked(wxCommandEvent& event)
{
	int retVal = wxMessageBox(wxT("Are you sure to delete all entries in the user camera database?"),
		wxT("User camera database"), wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION, this);
	if(retVal == wxYES)
	{
		bool isOk = UserCameraDB::getInstance().clearDB();
		if(!isOk)
		{
			wxMessageBox(wxT("Error during deleting the user camera database"),
				wxT("User camera database"), wxICON_ERROR, this);
		}
		else
		{
			pUserCameraDBListCtrl_->ClearAll();
		}
	}
}


BEGIN_EVENT_TABLE( Regard3DUserCameraDBDialog, Regard3DUserCameraDBDialogBase )
END_EVENT_TABLE()
