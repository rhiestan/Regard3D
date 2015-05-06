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
#ifndef REGARD3DSURFACEDIALOG_H
#define REGARD3DSURFACEDIALOG_H

#include "Regard3DMainFrameBase.h"
#include "R3DProject.h"

class Regard3DSurfaceDialog: public Regard3DSurfaceDialogBase
{
public:
	Regard3DSurfaceDialog(wxWindow *pParent);
	virtual ~Regard3DSurfaceDialog();

	void setParams(R3DProject::Densification *pDensification);
	void getResults(R3DProject::Surface *pSurface);

protected:
	virtual void OnInitDialog( wxInitDialogEvent& event );
	virtual void OnSurfaceGenerationMethodRadioBox( wxCommandEvent& event );
	virtual void OnPoissonDepthSliderScroll( wxScrollEvent& event );
	virtual void OnPoissonSamplesPerNodeSliderScroll( wxScrollEvent& event );
	virtual void OnPoissonPointWeightSliderScroll( wxScrollEvent& event );
	virtual void OnPoissonTrimThresholdSliderScroll( wxScrollEvent& event );
	virtual void OnFSSRRefineOctreeLevelsSliderScroll( wxScrollEvent& event );
	virtual void OnFSSRScaleFactorMultiplierSliderScroll( wxScrollEvent& event );
	virtual void OnFSSRConfidenceThresholdSliderScroll( wxScrollEvent& event );
	virtual void OnFSSRMinComponentSizeSliderScroll( wxScrollEvent& event );
	virtual void OnColorizationMethodRadioBox( wxCommandEvent& event );
	virtual void OnColVertNumberOfNeighboursSliderScroll( wxScrollEvent& event );

	void enableSurfaceGenWidgets();
	void enableColorizationWidgets();
	void updatePoissonDepthText();
	void updateSamplesPerNodeText();
	void updatePoissonPointWeightText();
	void updatePoissonTrimThresholdText();
	void updateFSSRRefineOctreeLevelsText();
	void updateFSSRScaleFactorMultiplierText();
	void updateFSSRConfidenceThresholdText();
	void updateFSSRMinComponentSizeText();
	void updateColVertNumberOfNeighboursText();

private:
	bool enablePoisson_, enableFSSR_;
	int fssrRefineOctreeLevels_;
	float fssrScaleFactorMultiplier_;
	float fssrConfidenceThreshold_;
	int fssrMinComponentSize_;

	DECLARE_EVENT_TABLE()
};

#endif
