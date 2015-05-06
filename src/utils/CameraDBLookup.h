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

#ifndef CAMERADBLOOKUP_H
#define CAMERADBLOOKUP_H

class CameraDBLookup
{
public:

	void initialize();

	bool lookupCamera(const wxString &cameraMaker, const wxString &cameraModel,
		double &sensorWidth);

	static CameraDBLookup &getInstance() { return instance_; }

private:
	CameraDBLookup();
	virtual ~CameraDBLookup();
	void parseLine(const wxString &line);

	class CameraDBEntry
	{
	public:
		CameraDBEntry() { }
		CameraDBEntry(const CameraDBEntry &o) { copy(o); }
		virtual ~CameraDBEntry() { }
		CameraDBEntry &copy(const CameraDBEntry &o)
		{
			cameraMaker_ = o.cameraMaker_;
			cameraModel_ = o.cameraModel_;
			sensorWidth_ = o.sensorWidth_;
			return *this;
		}
		CameraDBEntry & operator=(const CameraDBEntry & o)
		{
			return copy(o);
		}

		wxString cameraMaker_, cameraModel_;
		double sensorWidth_;
	};

	static bool matchesExactly(const wxString &cameraMaker, const wxString &cameraModel, const CameraDBEntry &cam);
	static bool matchesPartly(const wxString &cameraMaker, const wxString &cameraModel, const CameraDBEntry &cam);
	static bool containsDigit(const wxString &str);

	bool isInitialized_;
	std::vector<CameraDBEntry> cameraDB_;

	static CameraDBLookup instance_;
};

#endif
