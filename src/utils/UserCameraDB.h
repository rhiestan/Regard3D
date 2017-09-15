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
#ifndef USER_CAMERA_DB_H
#define USER_CAMERA_DB_H

#include <wx/string.h>
#include <vector>
#include <tuple>

class UserCameraDB
{
public:
	void initialize();

	bool queryDB(const wxString &cameraMaker, const wxString &cameraModel, double &sensorWidth);
	bool addToDB(const wxString &cameraMaker, const wxString &cameraModel, double sensorWidth);
	bool fetchAllEntries(std::vector<std::tuple<wxString, wxString, double> > &entries);
	bool removeFromDB(const wxString &cameraMaker, const wxString &cameraModel);
	bool clearDB();

	static UserCameraDB &getInstance() { return instance_; }

private:
	UserCameraDB();
	virtual ~UserCameraDB();

	bool isInitialized_;
	bool dbError_;
	wxString fileName_;

	static UserCameraDB instance_;

};
#endif // !USER_CAMERA_DB_H
