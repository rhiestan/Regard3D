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
#ifndef REGARD3DTRIANGULATIONDIALOG_H
#define REGARD3DTRIANGULATIONDIALOG_H

#include "Regard3DMainFrameBase.h"
#include "R3DImageUpdatesInterface.h"
#include "PreviewGeneratorThread.h"
#include "R3DProject.h"

class Regard3DTriangulationDialog: public Regard3DTriangulationDialogBase, public R3DImageUpdatesInterface
{
public:
	Regard3DTriangulationDialog(wxWindow *pParent);
	virtual ~Regard3DTriangulationDialog();

	void setPreviewGeneratorThread(PreviewGeneratorThread *pPreviewGeneratorThread);
	void setComputeMatches(R3DProject *pProject, R3DProject::ComputeMatches *pComputeMatches);

	bool isTriangulationPossible();

	void getResults(bool &global, size_t &initialImageID1, size_t &initialImageID2,
		int &rotAveraging, int &transAveraging,
		bool &refineIntrinsics);

	virtual void OnPreviewFinished();
	virtual void OnNewImageInfos();

	virtual void EndModal(int retCode);

protected:
	virtual void OnInitDialog( wxInitDialogEvent& event );
	virtual void OnTriangulationMethodRadioBox( wxCommandEvent& event );
	virtual void OnTInitialImagePairColClick( wxListEvent& event );
	virtual void OnTInitialImagePairItemDeselected( wxListEvent& event );
	virtual void OnTInitialImagePairItemSelected( wxListEvent& event );
	virtual void OnTPreviewWithMatchesCheckBox( wxCommandEvent& event );

	virtual void OnTimer( wxTimerEvent &event );

#if wxCHECK_VERSION(2, 9, 0)
	static int wxCALLBACK TInitialImagePairListCompareFunction(wxIntPtr item1, wxIntPtr item2, wxIntPtr sortData);
#else
	static int wxCALLBACK TInitialImagePairListCompareFunction(long item1, long item2, long sortData);
#endif

	void updateInitialImagePairListCtrl();
	void updateTriangulationMethodChoice();
	void checkForPreviewImage();

private:
	wxTimer aTimer_;
	PreviewGeneratorThread *pPreviewGeneratorThread_;
	R3DProject *pProject_;
	R3DProject::ComputeMatches *pComputeMatches_;
	R3DProject::PictureSet *pPictureSet_;
	R3DProjectPaths paths_;
	PreviewInfo previewInfoMatches_;

	bool isGlobalSfmAvailable_, initialImagePairListIsEmpty_;

	std::vector< std::pair<size_t, size_t> > imageIDList_;
	int ipSortColumn_, ipSortDirections_[5];
	static Regard3DTriangulationDialog *pDialog_;

	DECLARE_EVENT_TABLE()
};

#endif
