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
#ifndef REGARD3DCOMPUTEMATCHESDIALOG_H
#define REGARD3DCOMPUTEMATCHESDIALOG_H

#include "Regard3DMainFrameBase.h"

class Regard3DComputeMatchesDialog: public Regard3DComputeMatchesDialogBase
{
public:
	Regard3DComputeMatchesDialog(wxWindow *pParent);
	virtual ~Regard3DComputeMatchesDialog();

	void setMaxPixelCount(long long maxPixelCount);

	void getResults(float &keypointSensitivity,
		float &keypointMatchingRatio);

protected:
	virtual void OnInitDialog( wxInitDialogEvent& event );
	virtual void OnKeypointSensitivitySlider( wxScrollEvent& event );
	virtual void OnKeypointMatchingRatioSlider( wxScrollEvent& event );

	void updateICKeypointSensitivityText();
	void updateICKeypointMatchingRatioText();

private:
	float keypointSensitivity_;
	float keypointMatchingRatio_;
	long long maxPixelCount_;

	DECLARE_EVENT_TABLE()
};

#endif
