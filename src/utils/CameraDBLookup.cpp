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
#include "CameraDBLookup.h"
#include "Regard3DSettings.h"

#include <wx/textfile.h>
#include <wx/tokenzr.h>
#include <wx/stdpaths.h>

CameraDBLookup CameraDBLookup::instance_;

CameraDBLookup::CameraDBLookup() :
	isInitialized_(false)
{
}

CameraDBLookup::~CameraDBLookup()
{
}

void CameraDBLookup::initialize()
{
	wxString camFileName(wxT("sensor_database.csv"));
	wxString usedfilename = Regard3DSettings::getInstance().getCameraDBFilename();
	bool fileExists = wxFileName::FileExists(usedfilename);
	if(!fileExists)
	{
		wxFileName fn(wxStandardPaths::Get().GetResourcesDir(), camFileName);
		fileExists = fn.FileExists();
		usedfilename = fn.GetFullPath();
	}
	if(!fileExists)
	{
		wxFileName fn(wxStandardPaths::Get().GetExecutablePath());
		fn.SetFullName(camFileName);
		fileExists = fn.FileExists();
		usedfilename = fn.GetFullPath();
	}
	if(!fileExists)
	{
		wxMessageBox(wxT("The sensor database was not found.\nPlease put the file sensor_database.csv where the executable is located."),
			wxT("Camera DB not found"), wxICON_ERROR | wxOK, NULL);
		return;
	}
	wxTextFile cameraDBFile;
	if(!cameraDBFile.Open(usedfilename))
		return;

	wxString line = cameraDBFile.GetFirstLine();
	while(!cameraDBFile.Eof())
	{
		parseLine(line);
		line = cameraDBFile.GetNextLine();
	}
	parseLine(line);


	isInitialized_ = true;
}

bool CameraDBLookup::lookupCamera(const wxString &cameraMaker, const wxString &cameraModel,
	double &sensorWidth)
{
	// First, try to find an exact match
	for(size_t i = 0; i < cameraDB_.size(); i++)
	{
		const CameraDBEntry &entry = cameraDB_[i];
		if(matchesExactly(cameraMaker, cameraModel,entry))
		{
			sensorWidth = entry.sensorWidth_;
			return true;
		}
	}

	// No exact match found, try to find a partial match
	int partlyMatches = 0;
	for(size_t i = 0; i < cameraDB_.size(); i++)
	{
		const CameraDBEntry &entry = cameraDB_[i];
		if(matchesPartly(cameraMaker, cameraModel,entry))
		{
			sensorWidth = entry.sensorWidth_;
			partlyMatches++;
		}
	}

	// We were successfull if we fond exactly one match
	return (partlyMatches == 1);
}

bool CameraDBLookup::matchesExactly(const wxString &cameraMaker, const wxString &cameraModel, const CameraDBLookup::CameraDBEntry &cam)
{
	return (cameraMaker.IsSameAs(cam.cameraMaker_, false)
		&& cameraModel.IsSameAs(cam.cameraModel_, false));
}

bool CameraDBLookup::matchesPartly(const wxString &cameraMaker, const wxString &cameraModel, const CameraDBLookup::CameraDBEntry &cam)
{
	// Remove all blanks, retry exact match
	{
		wxString cameraMakerNB = cameraMaker, cameraModelNB = cameraModel;
		cameraMakerNB.Replace(wxT(" "), wxEmptyString, true);
		cameraModelNB.Replace(wxT(" "), wxEmptyString, true);
		wxString cameraMakerNBDB = cam.cameraMaker_, cameraModelNBDB = cam.cameraModel_;
		cameraMakerNBDB.Replace(wxT(" "), wxEmptyString, true);
		cameraModelNBDB.Replace(wxT(" "), wxEmptyString, true);
		if(cameraMakerNB.IsSameAs(cameraMakerNBDB, false)
			&& cameraModelNB.IsSameAs(cameraModelNBDB, false))
			return true;
	}

	// Split camera maker from JPEG into individual words
	wxStringTokenizer tokenizer(cameraMaker, wxT(" "));
	while(tokenizer.HasMoreTokens())
	{
		wxString tokenCameraMaker = tokenizer.GetNextToken();
		if(tokenCameraMaker.IsSameAs(cam.cameraMaker_, false))	// Compare ignoring case
		{
			// Found a word of the JPEG camera maker in the database
			// Try to match all parts of the JPEG camera model with digits in them with the database camera model
			bool foundAllParts = true;
			wxStringTokenizer toknzrModelJPG(cameraModel, wxT(" -"));
			while(toknzrModelJPG.HasMoreTokens())
			{
				wxString tokenModelJPG = toknzrModelJPG.GetNextToken().Lower();
				if(CameraDBLookup::containsDigit(tokenModelJPG))
				{
					bool found = false;
					wxStringTokenizer toknzrModelDB(cam.cameraModel_, wxT(" -"));
					while(toknzrModelDB.HasMoreTokens())
					{
						wxString tokenModelDB = toknzrModelDB.GetNextToken().Lower();
						if(tokenModelDB.IsSameAs(tokenModelJPG))
						{
							found = true;
						}
					}
					if(!found)
					{
						foundAllParts = false;
						break;
					}
				}
			}
			if(foundAllParts)
				return true;
		}
	}

	return false;
}

void CameraDBLookup::parseLine(const wxString &line)
{
	if(!line.IsEmpty())
	{
		wxStringTokenizer tokenizer(line, wxT(";"));
		wxString cameraMaker, cameraModel, sensorWidthStr;
		if(tokenizer.HasMoreTokens())
			cameraMaker = tokenizer.GetNextToken();
		if(tokenizer.HasMoreTokens())
			cameraModel = tokenizer.GetNextToken();
		if(tokenizer.HasMoreTokens())
			sensorWidthStr = tokenizer.GetNextToken();

		double sensorWidth = 0;
		if(!cameraMaker.IsEmpty()
			&& !cameraModel.IsEmpty()
			&& !sensorWidthStr.IsEmpty()
			&& sensorWidthStr.ToDouble(&sensorWidth))	// Convert string to double
		{
			CameraDBEntry newEntry;
			newEntry.cameraMaker_ = cameraMaker;
			newEntry.cameraModel_ = cameraModel;
			newEntry.sensorWidth_ = sensorWidth;
			cameraDB_.push_back(newEntry);
		}
	}
}

bool CameraDBLookup::containsDigit(const wxString &str)
{
	for(size_t i = 0; i < str.Length(); i++)
	{
		if(wxIsdigit( str.GetChar(i) ))
			return true;
	}
	return false;
}
