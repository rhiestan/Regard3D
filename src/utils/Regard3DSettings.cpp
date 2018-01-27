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
#include "Regard3DSettings.h"
#include "Regard3DMainFrame.h"
#include "Regard3DConsoleOutputFrame.h"
#include "Regard3DImagePreviewDialog.h"
#include "Regard3DPictureSetDialog.h"
#include "Regard3DTriangulationDialog.h"
#include "Regard3DMatchingResultsDialog.h"
#include "Regard3DUserCameraDBDialog.h"


#include <wx/config.h>

Regard3DSettings Regard3DSettings::instance_;

Regard3DSettings::Regard3DSettings()
{
	configWindowDimensions_ = wxT("WindowDimensions");
	configConsoleOutputFrameDimensions_ = wxT("ConsoleOutputFrameDimensions");
	configImagePreviewDimensions_ = wxT("ImagePreviewDimensions");
	configPictureSetDimensions_ = wxT("PictureSetDimensions");
	configTriangulationDimensions_ = wxT("TriangulationDimensions");
	configMatchingResultDimensions_ = wxT("MatchingResultDimensions");
	configUserCameraDBDimensions_ = wxT("UserCameraDBDimensions");
	configFontFilename_ = wxT("FontFilename");
	configCameraDBFilename_ = wxT("CameraDBFilename");
	configExternalEXEPath_ = wxT("ExternalEXEPath");
	configDefaultProjectPath_ = wxT("DefaultProjectPath");
	configUserCameraDBFilename_ = wxT("UserCameraDBFilename");
	configIsMouseButtonSwitched_ = wxT("IsMouseButtonSwitched");
	configIsMouseWheelSwitched_ = wxT("IsMouseWheelSwitched");
}

Regard3DSettings::~Regard3DSettings()
{
}

void Regard3DSettings::loadWindowLayoutFromConfig(Regard3DMainFrame *pMainFrame,
	Regard3DConsoleOutputFrame *pConsoleOutputFrame)
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));

	{
		wxPoint pt;
		wxSize sz;
		bool isMaximized = false;
		bool isFullScreen = false;
		int mainSplitterPos = 0;
		int projectSplitterPos = 0;

		wxString dimensions;
		if(config.Read(configWindowDimensions_, &dimensions))
		{
			std::wistringstream istr(std::wstring(dimensions.c_str()));
			istr >> pt.x >> pt.y >> sz.x >> sz.y >> isMaximized >> isFullScreen >> mainSplitterPos >> projectSplitterPos;

			pMainFrame->SetPosition(pt);
			pMainFrame->SetSize(sz);
			if(isMaximized)
				pMainFrame->Maximize();
			if(isFullScreen)
				pMainFrame->ShowFullScreen(true, 0);
			pMainFrame->getMainSplitter()->SetSashPosition(mainSplitterPos);
			pMainFrame->getProjectSplitter()->SetSashPosition(projectSplitterPos);
		}
	}

	{
		wxPoint pt;
		wxSize sz;
		bool isMaximized = false;
		bool isShown = false;

		wxString dimensions;
		if(config.Read(configConsoleOutputFrameDimensions_, &dimensions))
		{
			std::wistringstream istr(std::wstring(dimensions.c_str()));
			istr >> pt.x >> pt.y >> sz.x >> sz.y >> isMaximized >> isShown;

			pConsoleOutputFrame->SetPosition(pt);
			pConsoleOutputFrame->SetSize(sz);
			if(isMaximized)
				pConsoleOutputFrame->Maximize();

			pConsoleOutputFrame->Show(isShown);
		}
	}

}

void Regard3DSettings::saveWindowLayoutToConfig(Regard3DMainFrame *pMainFrame,
	Regard3DConsoleOutputFrame *pConsoleOutputFrame)
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));

	{
		wxPoint pt = pMainFrame->GetPosition();
		wxSize sz = pMainFrame->GetSize();
		bool isIconized = pMainFrame->IsIconized();
		bool isMaximized = pMainFrame->IsMaximized();
		bool isFullScreen = pMainFrame->IsFullScreen();

		int mainSplitterPos = pMainFrame->getMainSplitter()->GetSashPosition();
		int projectSplitterPos = pMainFrame->getProjectSplitter()->GetSashPosition();
		if(isIconized || isMaximized || isFullScreen)
		{
			// Do not overwrite window size and position in one of those cases
			wxString dimensions;
			if(config.Read(configWindowDimensions_, &dimensions))
			{
				std::wistringstream istr(std::wstring(dimensions.c_str()));
				istr >> pt.x >> pt.y >> sz.x >> sz.y;
			}
		}

		std::wostringstream ostr;
		ostr << pt.x << L" " << pt.y << L" " << sz.x << L" " << sz.y << L" " << isMaximized
			<< L" " << isFullScreen << L" " << mainSplitterPos << L" " << projectSplitterPos;
		wxString dimensions(ostr.str().c_str());

		config.Write(configWindowDimensions_, dimensions);
	}

	{
		wxPoint pt = pConsoleOutputFrame->GetPosition();
		wxSize sz = pConsoleOutputFrame->GetSize();
		bool isIconized = pConsoleOutputFrame->IsIconized();
		bool isMaximized = pConsoleOutputFrame->IsMaximized();
		bool isShown = pConsoleOutputFrame->IsShown();

		if(isIconized || isMaximized || !isShown)
		{
			// Do not overwrite window size and position in one of those cases
			wxString dimensions;
			if(config.Read(configConsoleOutputFrameDimensions_, &dimensions))
			{
				std::wistringstream istr(std::wstring(dimensions.c_str()));
				istr >> pt.x >> pt.y >> sz.x >> sz.y;
			}
		}

		std::wostringstream ostr;
		ostr << pt.x << L" " << pt.y << L" " << sz.x << L" " << sz.y << L" " << isMaximized
			<< L" " << isShown;
		wxString dimensions(ostr.str().c_str());

		config.Write(configConsoleOutputFrameDimensions_, dimensions);
	}
	
}


void Regard3DSettings::loadImagePreviewLayoutFromConfig(Regard3DImagePreviewDialog *pDialog)
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));

	{
		wxPoint pt;
		wxSize sz;
		bool isMaximized = false;
		bool isFullScreen = false;

		wxString dimensions;
		if(config.Read(configImagePreviewDimensions_, &dimensions))
		{
			std::wistringstream istr(std::wstring(dimensions.c_str()));
			istr >> pt.x >> pt.y >> sz.x >> sz.y >> isMaximized >> isFullScreen;

			pDialog->SetPosition(pt);
			pDialog->SetSize(sz);
			if(isMaximized)
				pDialog->Maximize();
			if(isFullScreen)
				pDialog->ShowFullScreen(true, 0);
		}
	}

}

void Regard3DSettings::saveImagePreviewLayoutToConfig(Regard3DImagePreviewDialog *pDialog)
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));

	{
		wxPoint pt = pDialog->GetPosition();
		wxSize sz = pDialog->GetSize();
		bool isIconized = pDialog->IsIconized();
		bool isMaximized = pDialog->IsMaximized();
		bool isFullScreen = pDialog->IsFullScreen();

		if(isIconized || isMaximized || isFullScreen)
		{
			// Do not overwrite window size and position in one of those cases
			wxString dimensions;
			if(config.Read(configImagePreviewDimensions_, &dimensions))
			{
				std::wistringstream istr(std::wstring(dimensions.c_str()));
				istr >> pt.x >> pt.y >> sz.x >> sz.y;
			}
		}

		std::wostringstream ostr;
		ostr << pt.x << L" " << pt.y << L" " << sz.x << L" " << sz.y << L" " << isMaximized
			<< L" " << isFullScreen;
		wxString dimensions(ostr.str().c_str());

		config.Write(configImagePreviewDimensions_, dimensions);
	}
}

void Regard3DSettings::loadPictureSetLayoutFromConfig(Regard3DPictureSetDialog *pDialog)
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));

	{
		wxPoint pt;
		wxSize sz;
		bool isMaximized = false;
		bool isFullScreen = false;
		int splitterPos = 0;

		wxString dimensions;
		if(config.Read(configPictureSetDimensions_, &dimensions))
		{
			std::wistringstream istr(std::wstring(dimensions.c_str()));
			istr >> pt.x >> pt.y >> sz.x >> sz.y >> isMaximized >> isFullScreen >> splitterPos;

			pDialog->SetPosition(pt);
			pDialog->SetSize(sz);
			if(isMaximized)
				pDialog->Maximize();
			if(isFullScreen)
				pDialog->ShowFullScreen(true, 0);

			pDialog->pCentralSplitter_->SetSashPosition(splitterPos);
		}
	}
}

void Regard3DSettings::savePictureSetLayoutToConfig(Regard3DPictureSetDialog *pDialog)
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));

	{
		wxPoint pt = pDialog->GetPosition();
		wxSize sz = pDialog->GetSize();
		bool isIconized = pDialog->IsIconized();
		bool isMaximized = pDialog->IsMaximized();
		bool isFullScreen = pDialog->IsFullScreen();
		int splitterPos = pDialog->pCentralSplitter_->GetSashPosition();

		if(isIconized || isMaximized || isFullScreen)
		{
			// Do not overwrite window size and position in one of those cases
			wxString dimensions;
			if(config.Read(configPictureSetDimensions_, &dimensions))
			{
				std::wistringstream istr(std::wstring(dimensions.c_str()));
				istr >> pt.x >> pt.y >> sz.x >> sz.y;
			}
		}

		std::wostringstream ostr;
		ostr << pt.x << L" " << pt.y << L" " << sz.x << L" " << sz.y << L" " << isMaximized
			<< L" " << isFullScreen << L" " << splitterPos;
		wxString dimensions(ostr.str().c_str());

		config.Write(configPictureSetDimensions_, dimensions);
	}
}

void Regard3DSettings::loadTriangulationLayoutFromConfig(Regard3DTriangulationDialog *pDialog)
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));

	{
		wxPoint pt;
		wxSize sz;
		bool isMaximized = false;
		bool isFullScreen = false;
		int splitterPos = 0;

		wxString dimensions;
		if(config.Read(configTriangulationDimensions_, &dimensions))
		{
			std::wistringstream istr(std::wstring(dimensions.c_str()));
			istr >> pt.x >> pt.y >> sz.x >> sz.y >> isMaximized >> isFullScreen >> splitterPos;

			pDialog->SetPosition(pt);
			pDialog->SetSize(sz);
			if(isMaximized)
				pDialog->Maximize();
			if(isFullScreen)
				pDialog->ShowFullScreen(true, 0);

			pDialog->pIncrementalMethodBoxSplitter_->SetSashPosition(splitterPos);
		}
	}
}

void Regard3DSettings::saveTriangulationLayoutToConfig(Regard3DTriangulationDialog *pDialog)
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));

	{
		wxPoint pt = pDialog->GetPosition();
		wxSize sz = pDialog->GetSize();
		bool isIconized = pDialog->IsIconized();
		bool isMaximized = pDialog->IsMaximized();
		bool isFullScreen = pDialog->IsFullScreen();
		int splitterPos = pDialog->pIncrementalMethodBoxSplitter_->GetSashPosition();

		if(isIconized || isMaximized || isFullScreen)
		{
			// Do not overwrite window size and position in one of those cases
			wxString dimensions;
			if(config.Read(configTriangulationDimensions_, &dimensions))
			{
				std::wistringstream istr(std::wstring(dimensions.c_str()));
				istr >> pt.x >> pt.y >> sz.x >> sz.y;
			}
		}

		std::wostringstream ostr;
		ostr << pt.x << L" " << pt.y << L" " << sz.x << L" " << sz.y << L" " << isMaximized
			<< L" " << isFullScreen << L" " << splitterPos;
		wxString dimensions(ostr.str().c_str());

		config.Write(configTriangulationDimensions_, dimensions);
	}
}

void Regard3DSettings::loadMatchingResultsLayoutFromConfig(Regard3DMatchingResultsDialog *pDialog)
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));

	{
		wxPoint pt;
		wxSize sz;
		bool isMaximized = false;
		bool isFullScreen = false;
		int splitterHPos = 0, splitterV1Pos = 0, splitterV2Pos = 0;

		wxString dimensions;
		if(config.Read(configMatchingResultDimensions_, &dimensions))
		{
			std::wistringstream istr(std::wstring(dimensions.c_str()));
			istr >> pt.x >> pt.y >> sz.x >> sz.y >> isMaximized >> isFullScreen >> splitterHPos
				>> splitterV1Pos >> splitterV2Pos;

			pDialog->SetPosition(pt);
			pDialog->SetSize(sz);
			if(isMaximized)
				pDialog->Maximize();
			if(isFullScreen)
				pDialog->ShowFullScreen(true, 0);

			pDialog->pHorizontalSplitter_->SetSashPosition(splitterHPos);
			pDialog->pUpperVerticalSplitter_->SetSashPosition(splitterV1Pos);
			pDialog->pLowerVerticalSplitter_->SetSashPosition(splitterV2Pos);
		}
	}
}

void Regard3DSettings::saveMatchingResultsLayoutToConfig(Regard3DMatchingResultsDialog *pDialog)
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));

	{
		wxPoint pt = pDialog->GetPosition();
		wxSize sz = pDialog->GetSize();
		bool isIconized = pDialog->IsIconized();
		bool isMaximized = pDialog->IsMaximized();
		bool isFullScreen = pDialog->IsFullScreen();
		int splitterHPos = pDialog->pHorizontalSplitter_->GetSashPosition();
		int splitterV1Pos = pDialog->pUpperVerticalSplitter_->GetSashPosition();
		int splitterV2Pos = pDialog->pLowerVerticalSplitter_->GetSashPosition();

		if(isIconized || isMaximized || isFullScreen)
		{
			// Do not overwrite window size and position in one of those cases
			wxString dimensions;
			if(config.Read(configMatchingResultDimensions_, &dimensions))
			{
				std::wistringstream istr(std::wstring(dimensions.c_str()));
				istr >> pt.x >> pt.y >> sz.x >> sz.y;
			}
		}

		std::wostringstream ostr;
		ostr << pt.x << L" " << pt.y << L" " << sz.x << L" " << sz.y << L" " << isMaximized
			<< L" " << isFullScreen << L" " << splitterHPos
			 << L" " << splitterV1Pos  << L" " << splitterV2Pos;
		wxString dimensions(ostr.str().c_str());

		config.Write(configMatchingResultDimensions_, dimensions);
	}
}

void Regard3DSettings::loadUserCameraDBLayoutFromConfig(Regard3DUserCameraDBDialog *pDialog)
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));

	{
		wxPoint pt;
		wxSize sz;
		bool isMaximized = false;
		bool isFullScreen = false;
		int column1Width = 0, column2Width = 0, column3Width = 0;

		wxString dimensions;
		if(config.Read(configUserCameraDBDimensions_, &dimensions))
		{
			std::wistringstream istr(std::wstring(dimensions.c_str()));
			istr >> pt.x >> pt.y >> sz.x >> sz.y >> isMaximized >> isFullScreen >> column1Width
				>> column2Width >> column3Width;

			pDialog->SetPosition(pt);
			pDialog->SetSize(sz);
			if(isMaximized)
				pDialog->Maximize();
			if(isFullScreen)
				pDialog->ShowFullScreen(true, 0);

			if(column1Width > 0)
				pDialog->pUserCameraDBListCtrl_->SetColumnWidth(0, column1Width);
			else
				pDialog->pUserCameraDBListCtrl_->SetColumnWidth(0, wxLIST_AUTOSIZE);
			if(column2Width > 0)
				pDialog->pUserCameraDBListCtrl_->SetColumnWidth(1, column2Width);
			else
				pDialog->pUserCameraDBListCtrl_->SetColumnWidth(1, wxLIST_AUTOSIZE);
			if(column3Width > 0)
				pDialog->pUserCameraDBListCtrl_->SetColumnWidth(2, column3Width);
			else
				pDialog->pUserCameraDBListCtrl_->SetColumnWidth(2, wxLIST_AUTOSIZE);
		}
	}
}

void Regard3DSettings::saveUserCameraDBLayoutToConfig(Regard3DUserCameraDBDialog *pDialog)
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));

	{
		wxPoint pt = pDialog->GetPosition();
		wxSize sz = pDialog->GetSize();
		bool isIconized = pDialog->IsIconized();
		bool isMaximized = pDialog->IsMaximized();
		bool isFullScreen = pDialog->IsFullScreen();
		int column1Width = pDialog->pUserCameraDBListCtrl_->GetColumnWidth(0);
		int column2Width = pDialog->pUserCameraDBListCtrl_->GetColumnWidth(1);
		int column3Width = pDialog->pUserCameraDBListCtrl_->GetColumnWidth(2);

		if(isIconized || isMaximized || isFullScreen)
		{
			// Do not overwrite window size and position in one of those cases
			wxString dimensions;
			if(config.Read(configUserCameraDBDimensions_, &dimensions))
			{
				std::wistringstream istr(std::wstring(dimensions.c_str()));
				istr >> pt.x >> pt.y >> sz.x >> sz.y;
			}
		}

		std::wostringstream ostr;
		ostr << pt.x << L" " << pt.y << L" " << sz.x << L" " << sz.y << L" " << isMaximized
			<< L" " << isFullScreen << L" " << column1Width
			 << L" " << column2Width  << L" " << column3Width;
		wxString dimensions(ostr.str().c_str());

		config.Write(configUserCameraDBDimensions_, dimensions);
	}
}

wxString Regard3DSettings::getFontFilename()
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));
	wxString str;
	if(config.Read(configFontFilename_, &str))
		return str;

	return wxEmptyString;
}

wxString Regard3DSettings::getCameraDBFilename()
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));
	wxString str;
	if(config.Read(configCameraDBFilename_, &str))
		return str;

	return wxEmptyString;
}

wxString Regard3DSettings::getExternalEXEPath()
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));
	wxString str;
	if(config.Read(configExternalEXEPath_, &str))
		return str;

	return wxEmptyString;
}

wxString Regard3DSettings::getDefaultProjectPath()
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));
	wxString str;
	if(config.Read(configDefaultProjectPath_, &str))
		return str;

	return wxEmptyString;
}

wxString Regard3DSettings::getUserCameraDBFilename()
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));
	wxString str;
	if(config.Read(configUserCameraDBFilename_, &str))
		return str;

	return wxEmptyString;
}

bool Regard3DSettings::getIsMouseButtonSwitched()
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));
	bool val;
	if(config.Read(configIsMouseButtonSwitched_, &val))
		return val;

	return false;
}

bool Regard3DSettings::getIsMouseWheelSwitched()
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));
	bool val;
	if(config.Read(configIsMouseWheelSwitched_, &val))
		return val;

	return false;
}

void Regard3DSettings::setDefaultProjectPath(const wxString &defaultProjectPath)
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));
	config.Write(configDefaultProjectPath_, defaultProjectPath);
}

void Regard3DSettings::setUserCameraDBFilename(const wxString &cameraUserDBFilename)
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));
	config.Write(configUserCameraDBFilename_, cameraUserDBFilename);
}

void Regard3DSettings::setIsMouseButtonSwitched(bool isSwitched)
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));
	config.Write(configIsMouseButtonSwitched_, isSwitched);
}

void Regard3DSettings::setIsMouseWheelSwitched(bool isSwitched)
{
	wxConfig config;	// AppName and VendorName are set in Regard3DApp::OnInit()
	config.SetPath(wxT("/"));
	config.Write(configIsMouseWheelSwitched_, isSwitched);
}
