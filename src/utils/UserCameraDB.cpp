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
#include "UserCameraDB.h"

// wxWidgets
#include <wx/stdpaths.h>
#include <wx/filename.h>

// sqlite
#include "sqlite3.h"

// The static instance
UserCameraDB UserCameraDB::instance_;


UserCameraDB::UserCameraDB()
	: isInitialized_(false), dbError_(false)
{
}

UserCameraDB::~UserCameraDB()
{
}

void UserCameraDB::initialize()
{
	wxString userLocalDataDir = wxStandardPaths::Get().GetUserLocalDataDir();
	wxFileName userCameraDBFileName(userLocalDataDir, wxT("usercamera.db"));
	fileName_ = userCameraDBFileName.GetFullPath();

	if(!userCameraDBFileName.Exists())
	{
		// Create database file
		sqlite3 *pDb = NULL;
		char *errmsg;
		int err = sqlite3_open(fileName_.ToUTF8(), &pDb);
		if(err == SQLITE_OK)
		{
			wxString sqlString(wxT("CREATE TABLE CameraDB (cameraMaker TEXT, cameraModel TEXT, sensorWidth TEXT);"));
			err = sqlite3_exec(pDb, sqlString.ToUTF8(), NULL, NULL, &errmsg);
			if(err != SQLITE_OK)
			{
				// We have an error, see errmsg
				sqlite3_free(errmsg);
				sqlite3_close(pDb);
				dbError_ = true;
				return;
			}

			sqlite3_close(pDb);
		}
	}
}

bool UserCameraDB::queryDB(const wxString &cameraMaker, const wxString &cameraModel, double &sensorWidth)
{
	sqlite3 *pDb = NULL;
	int err = sqlite3_open(fileName_.ToUTF8(), &pDb);
	if(err == SQLITE_OK)
	{
		wxString sqlString(wxT("SELECT sensorWidth FROM CameraDB WHERE cameraMaker=?1 AND cameraModel=?2;"));
		sqlite3_stmt *pStmt = NULL;
		err = sqlite3_prepare_v2(pDb, sqlString.ToUTF8(), -1, &pStmt, NULL);
		if(err != SQLITE_OK)
		{
			// We have an error
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}
		err = sqlite3_bind_text(pStmt, 1, cameraMaker.ToUTF8(), -1, SQLITE_TRANSIENT);
		if(err != SQLITE_OK)
		{
			// We have an error
			sqlite3_finalize(pStmt);
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}
		err = sqlite3_bind_text(pStmt, 2, cameraModel.ToUTF8(), -1, SQLITE_TRANSIENT);
		if(err != SQLITE_OK)
		{
			// We have an error
			sqlite3_finalize(pStmt);
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}
		err = sqlite3_step(pStmt);
		if(err != SQLITE_DONE && err != SQLITE_ROW)
		{
			// We have an error
			sqlite3_finalize(pStmt);
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}
		bool found = false;
		if(err == SQLITE_ROW)
		{
			found = true;
			int sensorWidthTextBytes = sqlite3_column_bytes(pStmt, 0);
			const unsigned char *sensorWidthText = sqlite3_column_text(pStmt, 0);
			wxString sensorWidthStr = wxString::FromUTF8(reinterpret_cast<const char *>(sensorWidthText), sensorWidthTextBytes);

			sensorWidthStr.ToDouble(&sensorWidth);
		}

		sqlite3_finalize(pStmt);
		sqlite3_close(pDb);

		return found;
	}
	else
		return false;

	return true;
}

bool UserCameraDB::addToDB(const wxString &cameraMaker, const wxString &cameraModel, double sensorWidth)
{
	sqlite3 *pDb = NULL;
	int err = sqlite3_open(fileName_.ToUTF8(), &pDb);
	if(err == SQLITE_OK)
	{
		wxString sqlString(wxT("INSERT INTO CameraDB (cameraMaker, cameraModel, sensorWidth) VALUES (?1, ?2, ?3);"));
		sqlite3_stmt *pStmt = NULL;
		err = sqlite3_prepare_v2(pDb, sqlString.ToUTF8(), -1, &pStmt, NULL);
		if(err != SQLITE_OK)
		{
			// We have an error
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}
		err = sqlite3_bind_text(pStmt, 1, cameraMaker.ToUTF8(), -1, SQLITE_TRANSIENT);
		if(err != SQLITE_OK)
		{
			// We have an error
			sqlite3_finalize(pStmt);
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}
		err = sqlite3_bind_text(pStmt, 2, cameraModel.ToUTF8(), -1, SQLITE_TRANSIENT);
		if(err != SQLITE_OK)
		{
			// We have an error
			sqlite3_finalize(pStmt);
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}
		err = sqlite3_bind_double(pStmt, 3, sensorWidth);
		if(err != SQLITE_OK)
		{
			// We have an error
			sqlite3_finalize(pStmt);
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}
		err = sqlite3_step(pStmt);
		if(err != SQLITE_DONE && err != SQLITE_ROW)
		{
			// We have an error
			sqlite3_finalize(pStmt);
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}
		err = sqlite3_finalize(pStmt);
		if(err != SQLITE_OK)
		{
			// We have an error
			sqlite3_finalize(pStmt);
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}
		sqlite3_close(pDb);
	}
	else
		return false;

	return true;
}

bool UserCameraDB::fetchAllEntries(std::vector<std::tuple<wxString, wxString, double> > &entries)
{
	sqlite3 *pDb = NULL;
	entries.clear();
	int err = sqlite3_open(fileName_.ToUTF8(), &pDb);
	if(err == SQLITE_OK)
	{
		wxString sqlString(wxT("SELECT cameraMaker, cameraModel, sensorWidth FROM CameraDB;"));
		sqlite3_stmt *pStmt = NULL;
		err = sqlite3_prepare_v2(pDb, sqlString.ToUTF8(), -1, &pStmt, NULL);
		if(err != SQLITE_OK)
		{
			// We have an error
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}
		err = sqlite3_step(pStmt);
		while(err == SQLITE_ROW)
		{
			int cameraMakerTextBytes = sqlite3_column_bytes(pStmt, 0);
			const unsigned char *cameraMakerText = sqlite3_column_text(pStmt, 0);
			wxString cameraMakerStr = wxString::FromUTF8(reinterpret_cast<const char *>(cameraMakerText), cameraMakerTextBytes);
			int cameraModelTextBytes = sqlite3_column_bytes(pStmt, 1);
			const unsigned char *cameraModelText = sqlite3_column_text(pStmt, 1);
			wxString cameraModelStr = wxString::FromUTF8(reinterpret_cast<const char *>(cameraModelText), cameraModelTextBytes);
			int sensorWidthTextBytes = sqlite3_column_bytes(pStmt, 2);
			const unsigned char *sensorWidthText = sqlite3_column_text(pStmt, 2);
			wxString sensorWidthStr = wxString::FromUTF8(reinterpret_cast<const char *>(sensorWidthText), sensorWidthTextBytes);
			double sensorWidth = 0;
			sensorWidthStr.ToDouble(&sensorWidth);

			entries.push_back( std::make_tuple<>(cameraMakerStr, cameraModelStr, sensorWidth));

			err = sqlite3_step(pStmt);
		}
		if(err != SQLITE_DONE && err != SQLITE_ROW)
		{
			// We have an error
			sqlite3_finalize(pStmt);
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}

		sqlite3_finalize(pStmt);
		sqlite3_close(pDb);
	}
	else
		return false;

	return true;
}

bool UserCameraDB::removeFromDB(const wxString &cameraMaker, const wxString &cameraModel)
{
	sqlite3 *pDb = NULL;
	int err = sqlite3_open(fileName_.ToUTF8(), &pDb);
	if(err == SQLITE_OK)
	{
		wxString sqlString(wxT("DELETE FROM CameraDB WHERE cameraMaker=?1 AND cameraModel=?2;"));
		sqlite3_stmt *pStmt = NULL;
		err = sqlite3_prepare_v2(pDb, sqlString.ToUTF8(), -1, &pStmt, NULL);
		if(err != SQLITE_OK)
		{
			// We have an error
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}
		err = sqlite3_bind_text(pStmt, 1, cameraMaker.ToUTF8(), -1, SQLITE_TRANSIENT);
		if(err != SQLITE_OK)
		{
			// We have an error
			sqlite3_finalize(pStmt);
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}
		err = sqlite3_bind_text(pStmt, 2, cameraModel.ToUTF8(), -1, SQLITE_TRANSIENT);
		if(err != SQLITE_OK)
		{
			// We have an error
			sqlite3_finalize(pStmt);
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}
		err = sqlite3_step(pStmt);
		if(err != SQLITE_DONE && err != SQLITE_ROW)
		{
			// We have an error
			sqlite3_finalize(pStmt);
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}

		sqlite3_finalize(pStmt);
		sqlite3_close(pDb);
	}
	else
		return false;

	return true;
}

bool UserCameraDB::clearDB()
{
	sqlite3 *pDb = NULL;
	int err = sqlite3_open(fileName_.ToUTF8(), &pDb);
	if(err == SQLITE_OK)
	{
		wxString sqlString(wxT("DELETE FROM CameraDB;"));
		sqlite3_stmt *pStmt = NULL;
		err = sqlite3_prepare_v2(pDb, sqlString.ToUTF8(), -1, &pStmt, NULL);
		if(err != SQLITE_OK)
		{
			// We have an error
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}
		err = sqlite3_step(pStmt);
		if(err != SQLITE_DONE && err != SQLITE_ROW)
		{
			// We have an error
			sqlite3_finalize(pStmt);
			sqlite3_close(pDb);
			dbError_ = true;
			return false;
		}

		sqlite3_finalize(pStmt);
		sqlite3_close(pDb);
	}
	else
		return false;

	return true;
}
