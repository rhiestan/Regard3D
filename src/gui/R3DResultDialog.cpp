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
#include "R3DResultDialog.h"

enum
{
	ID_OPEN_HTML_BUTTON = 4200
};

R3DResultDialog::R3DResultDialog(wxWindow *parent, const wxString &title,
	const wxString &header, const wxArrayString &resultStrings,
	const wxString &htmlReport)
	: wxDialog(parent, wxID_ANY, title),
	htmlReport_(htmlReport)
{
	wxBoxSizer *pSizer = new wxBoxSizer(wxVERTICAL);
	wxPanel *pPanel = new wxPanel(this);
	pSizer->Add(pPanel, wxSizerFlags().Expand().Proportion(1).Border(wxALL, 3));

	wxBoxSizer *pInnerSizer = new wxBoxSizer(wxVERTICAL);
	wxStaticText *pHeaderStaticText = new wxStaticText(pPanel, wxID_ANY, header);
	pInnerSizer->Add(pHeaderStaticText, wxSizerFlags().Expand().Proportion(0).Border(wxALL, 3));

	int cols = 2, rows = resultStrings.GetCount() / 2;
	wxFlexGridSizer *pFGSizer = new wxFlexGridSizer(rows, cols, 0, 0);
	pFGSizer->AddGrowableCol(1);
	pFGSizer->SetFlexibleDirection( wxBOTH );
	pFGSizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );

	for(size_t i = 0; i < resultStrings.GetCount(); i++)
	{
		const wxString &str = resultStrings[i];
		if((i % 2) == 0)
		{
			// Left: Static text, right aligned
			wxStaticText *pStaticText = new wxStaticText(pPanel, wxID_ANY, str + wxT(":"));
			pFGSizer->Add(pStaticText, wxSizerFlags().Proportion(0).Border(wxALL, 3)
				.Align(wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT));
		}
		else
		{
			// Right: read-only text control
			wxTextCtrl *pTextCtrl = new wxTextCtrl(pPanel, wxID_ANY, str, wxDefaultPosition, wxDefaultSize, wxTE_READONLY);
			pTextCtrl->SetMinSize( wxSize( 200, -1 ) );
			pFGSizer->Add(pTextCtrl, wxSizerFlags().Expand().Proportion(1).Border(wxALL, 3)
				.Align(wxALIGN_CENTER_VERTICAL));
		}
	}
	pInnerSizer->Add(pFGSizer, wxSizerFlags().Expand().Proportion(1));

	if(!htmlReport.IsEmpty()
		&& wxFileName::FileExists(htmlReport))
	{
		// Add button to open HTML report
		wxBoxSizer *pSizer2 = new wxBoxSizer(wxHORIZONTAL);
		wxButton *pOpenHTMLReportButton = new wxButton(pPanel, ID_OPEN_HTML_BUTTON, wxT("Open HTML Report"));
		pSizer2->Add(pOpenHTMLReportButton, wxSizerFlags().Border(wxALL, 3));
		pInnerSizer->Add(pSizer2, wxSizerFlags().Expand().Proportion(0));
	}
	wxStdDialogButtonSizer *pStdDialogButtonSizer = new wxStdDialogButtonSizer();
	wxButton *pOKButton = new wxButton(pPanel, wxID_OK);
	pStdDialogButtonSizer->AddButton(pOKButton);
	pStdDialogButtonSizer->Realize();

	pInnerSizer->Add(pStdDialogButtonSizer, wxSizerFlags().Expand().Proportion(0));
	pPanel->SetSizer(pInnerSizer);

	SetSizer(pSizer);
	Fit();
	Layout();
	CenterOnParent();
}

R3DResultDialog::~R3DResultDialog()
{
}

void R3DResultDialog::OnOpenHTMLReportButton(wxCommandEvent& evt)
{
	wxLaunchDefaultBrowser(htmlReport_);
}

BEGIN_EVENT_TABLE( R3DResultDialog, wxDialog )
	EVT_BUTTON(ID_OPEN_HTML_BUTTON, R3DResultDialog::OnOpenHTMLReportButton)
END_EVENT_TABLE()
