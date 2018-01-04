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
#ifndef REGARD3DDENSIFICATIONDIALOG_H
#define REGARD3DDENSIFICATIONDIALOG_H

#include "Regard3DMainFrameBase.h"
#include "R3DProject.h"

class Regard3DDensificationDialog: public Regard3DDensificationDialogBase
{
public:
	Regard3DDensificationDialog(wxWindow *pParent);
	virtual ~Regard3DDensificationDialog();

	void getResults(R3DProject::Densification *pDensification);

protected:
	virtual void OnInitDialog( wxInitDialogEvent& event );
	virtual void OnUseCMVSCheckBox( wxCommandEvent& event );
	virtual void OnPMVSLevelSliderScroll( wxScrollEvent& event );
	virtual void OnPMVSCellSizeSliderScroll( wxScrollEvent& event );
	virtual void OnPMVSThresholdSliderScroll( wxScrollEvent& event );
	virtual void OnPMVSWSizeSliderScroll( wxScrollEvent& event );
	virtual void OnPMVSMinImageNumSliderScroll( wxScrollEvent& event );
	virtual void OnMVEScaleSliderScroll( wxScrollEvent& event );
	virtual void OnMVEFilterWidthSliderScroll( wxScrollEvent& event );
	virtual void OnSMVSInputScaleSliderScroll( wxScrollEvent& event );
	virtual void OnSMVSOutputScaleSliderScroll( wxScrollEvent& event );
	virtual void OnSMVSSurfaceSmoothingFactorSliderScroll( wxScrollEvent& event );

	void updatePMVSLevelText();
	void updatePMVSCellSizeText();
	void updatePMVSThresholdText();
	void updatePMVSWSizeText();
	void updatePMVSMinImageNumText();
	void updateMVEScaleText();
	void updateMVEFilterWidthText();
	void updateSMVSInputScaleText();
	void updateSMVSOutputScaleText();
	void updateSMVSSurfaceSmoothingFactorText();

private:
	DECLARE_EVENT_TABLE()
};

#endif
