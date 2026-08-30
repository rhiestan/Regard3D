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

/**
 * Everything the triangulation dialog collects.
 *
 * engine_ decides who does the work: 0 the built-in library, 1
 * openMVG_main_SfM. Everything above openMVG_ applies to both, because the
 * three pages of the method choicebook are the three OpenMVG SfM engines.
 */
struct R3DTriangulationDialogResults
{
	R3DTriangulationDialogResults()
		: engine_(1), algorithm_(R3DProject::R3DTA_Incremental2),
		initialImageID1_(0), initialImageID2_(0),
		rotAveraging_(2), transAveraging_(1),
		refineIntrinsics_(true), useGPSInfo_(false),
		triInitialization_(R3DProject::R3DTI_MaxPair) { }

	int engine_;

	R3DProject::R3DTriangulationAlgorithm algorithm_;
	size_t initialImageID1_, initialImageID2_;
	int rotAveraging_, transAveraging_;
	bool refineIntrinsics_;
	bool useGPSInfo_;
	R3DProject::R3DTriangulationInitialization triInitialization_;

	// openMVG_main_SfM only
	R3DOpenMVGTriangulationParams openMVG_;
};

class Regard3DTriangulationDialog: public Regard3DTriangulationDialogBase, public R3DImageUpdatesInterface
{
public:
	Regard3DTriangulationDialog(wxWindow *pParent);
	virtual ~Regard3DTriangulationDialog();

	void setPreviewGeneratorThread(PreviewGeneratorThread *pPreviewGeneratorThread);
	void setComputeMatches(R3DProject *pProject, R3DProject::ComputeMatches *pComputeMatches);

	bool isTriangulationPossible();

	void getResults(R3DTriangulationDialogResults &results);

	virtual void OnPreviewFinished();
	virtual void OnNewImageInfos();

	virtual void EndModal(int retCode);

protected:
	virtual void OnInitDialog( wxInitDialogEvent& event );
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

	bool isOpenMVGSfMPossible(wxString &reason);
	void setOpenMVGToolTips();
	void initializeOpenMVGOptions();
	void readOpenMVGOptions();
	void updateEngineDependencies();

private:
	wxTimer aTimer_;
	PreviewGeneratorThread *pPreviewGeneratorThread_;
	R3DProject *pProject_;
	R3DProject::ComputeMatches *pComputeMatches_;
	R3DProject::PictureSet *pPictureSet_;
	R3DProjectPaths paths_;
	PreviewInfo previewInfoMatches_;

	bool isGlobalSfmAvailable_, initialImagePairListIsEmpty_;
	bool isOpenMVGSfMAvailable_;
	R3DTriangulationDialogResults results_;

	std::vector< std::pair<size_t, size_t> > imageIDList_;
	int ipSortColumn_, ipSortDirections_[5];
	static Regard3DTriangulationDialog *pDialog_;

	DECLARE_EVENT_TABLE()
};

#endif
