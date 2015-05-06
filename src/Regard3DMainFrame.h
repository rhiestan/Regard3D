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
#ifndef REGARD3DMAINFRAME_H
#define REGARD3DMAINFRAME_H

class OSGGLCanvas;
class ViewerOptionsPanel;
class ConvertModelToOSG;
class Regard3DDropTarget;
class R3DComputeMatchesThread;
class R3DTriangulationThread;
class Regard3DConsoleOutputFrame;
class Regard3DProgressDialog;
class Regard3DModelViewHelper;
class R3DDensificationProcess;
class R3DSurfaceGenProcess;
class R3DSmallTasksThread;
class R3DImageUpdatesInterface;

class wxProgressDialog;

#include <vector>
#include <utility>

#include <wx/process.h>

#include "Regard3DMainFrameBase.h"
#include "PreviewGeneratorThread.h"
#include "ImageInfoThread.h"
#include "R3DProject.h"

#include <osgViewer/Viewer>

// Workaround for X.h defining Success
#undef Success

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

class Regard3DMainFrame: public Regard3DMainFrameBase
{
public:
	Regard3DMainFrame(wxWindow* parent);
	virtual ~Regard3DMainFrame();

	OSGGLCanvas *getGLCanvas();
	void SetViewer(osgViewer::Viewer *viewer);

	bool addDragAndDropFiles(const wxArrayString &filenames);

	void sendNewPreviewImageEvent();
	void sendNewImageInfoEvent();

	void sendUpdateProgressBarEvent(float progress,
		const wxString &message);

	void sendComputeMatchesFinishedEvent();
	void sendTriangulationFinishedEvent();
	void sendDensificationFinishedEvent();
	void sendSurfaceGenFinishedEvent();
	void sendSmallTaskFinishedEvent();

	Regard3DConsoleOutputFrame *getConsoleOutputFrame() const { return pRegard3DConsoleOutputFrame_; }
	wxSplitterWindow *getMainSplitter() const { return pMainSplitter_; }
	wxSplitterWindow *getProjectSplitter() const { return pProjectSplitter_; }

protected:
	virtual void OnMainFrameClose( wxCloseEvent& event );
	virtual void OnMainFrameIdle( wxIdleEvent& event );
	virtual void OnNewProject( wxCommandEvent& event );
	virtual void OnOpenProject( wxCommandEvent& event );
	virtual void OnCloseProject( wxCommandEvent& event );
	virtual void OnMainFrameExitMenuItem( wxCommandEvent& event );
	virtual void OnPropertiesMenuItem( wxCommandEvent& event );
	virtual void OnViewConsoleOutputFrameMenuItem( wxCommandEvent& event );
	virtual void OnViewConsoleOutputFrameUpdate( wxUpdateUIEvent& event );
	virtual void OnAboutMenuItem( wxCommandEvent& event );
	virtual void OnProjectTreeItemActivated( wxTreeEvent& event );
	virtual void OnProjectTreeItemMenu( wxTreeEvent& event );
	virtual void OnProjectTreeSelChanged( wxTreeEvent& event );
	virtual void OnProjectTreeSelChanging( wxTreeEvent& event );
	virtual void OnProjectAddPictureSetButton( wxCommandEvent& event );
	virtual void OnPictureSetComputeMatchesButton( wxCommandEvent& event );
	virtual void OnEditPictureSetButton( wxCommandEvent& event );
	virtual void OnClonePictureSetButton( wxCommandEvent& event );
	virtual void OnProjectDeletePictureSetButton( wxCommandEvent& event );
	virtual void OnShowMatchingResultsButton( wxCommandEvent& event );
	virtual void OnTriangulationButton( wxCommandEvent& event );
	virtual void OnProjectDeleteComputeMatchesButton( wxCommandEvent& event );
	virtual void OnCreateDensePointcloudButton( wxCommandEvent& event );
	virtual void OnShowTriangulatedPoints( wxCommandEvent& event );
	virtual void OnExportTriangulationToCMPMVSButton( wxCommandEvent& event );
	virtual void OnProjectDeleteTriangulationButton( wxCommandEvent& event );
	virtual void OnCreateSurfaceButton( wxCommandEvent& event );
	virtual void OnShowDensePointCloudButton( wxCommandEvent& event );
	virtual void OnProjectDeleteDensificationButton( wxCommandEvent& event );
	virtual void OnExportDensificationButton( wxCommandEvent& event );
	virtual void OnExportDensificationToMeshLabButton( wxCommandEvent& event );
	virtual void OnShowSurfaceButton( wxCommandEvent& event );
	virtual void OnExportSurfaceButton( wxCommandEvent& event );
	virtual void OnDeleteSurfaceButton( wxCommandEvent& event );
	virtual void OnShowTrackballCheckBox( wxCommandEvent& event );
	virtual void OnPointSizeScroll( wxScrollEvent& event );
	virtual void OnShowTextureCheckBox( wxCommandEvent& event );
	virtual void OnEnableLightingCheckBox( wxCommandEvent& event );
	virtual void OnPolygonModeRadioBox( wxCommandEvent& event );
	virtual void OnShadingModelRadioBox( wxCommandEvent& event );
	virtual void OnResetViewButton( wxCommandEvent& event );

	virtual void OnTempDevToolClicked( wxCommandEvent& event );
	virtual void OnPreviewFinished( wxCommandEvent &event );
	virtual void OnNewImageInfos( wxCommandEvent &event );
	virtual void OnUpdateProgressBar( wxCommandEvent &event );
	virtual void OnComputeMatchesFinished( wxCommandEvent &event );
	virtual void OnTriangulationFinished( wxCommandEvent &event );
	virtual void OnDensificationFinished( wxCommandEvent &event );
	virtual void OnSurfaceGenFinished( wxCommandEvent &event );
	virtual void OnSmallTaskFinished( wxCommandEvent &event );
	virtual void OnContextMenuAddPictureSet( wxCommandEvent &event );
	virtual void OnContextMenuComputeMatches( wxCommandEvent &event );
	virtual void OnContextMenuEditPictureSet( wxCommandEvent &event );
	virtual void OnContextMenuClonePictureSet( wxCommandEvent &event );
	virtual void OnContextMenuShowMatches( wxCommandEvent &event );
	virtual void OnContextMenuTriangulation( wxCommandEvent &event );
	virtual void OnContextMenuCreateDensePointcloud( wxCommandEvent &event );
	virtual void OnContextMenuShowTriangulatedPoints( wxCommandEvent &event );
	virtual void OnContextMenuDeletePictureSet( wxCommandEvent &event );
	virtual void OnContextMenuDeleteComputeMatches( wxCommandEvent &event );
	virtual void OnContextMenuDeleteTriangulation( wxCommandEvent &event );
	virtual void OnContextMenuCreateSurface( wxCommandEvent &event );
	virtual void OnContextMenuShowDensePoints( wxCommandEvent &event );
	virtual void OnContextMenuDeleteDensification( wxCommandEvent &event );
	virtual void OnContextMenuShowSurface( wxCommandEvent &event );
	virtual void OnContextMenuDeleteSurface( wxCommandEvent &event );
	virtual void OnContextMenuExportTriangulationToCMPMVS( wxCommandEvent &event );
	virtual void OnContextMenuExportPointCloud( wxCommandEvent &event );
	virtual void OnContextMenuExportDensificationToMeshLab( wxCommandEvent &event );
	virtual void OnContextMenuExportSurface( wxCommandEvent &event );

	virtual void OnTimer( wxTimerEvent &event );

	wxIcon getIcon();
	void updateUIFromProject(bool newProject);

	void clear3DModel();
	void load3DModel(const wxString &filename);
	void loadSurfaceModel(const wxString &filename);

	bool getProjectTreeItemDetails(const wxTreeItemId &treeId, R3DProject::R3DTreeItem::R3DTreeItemType &type, int &id);
	void selectProjectTreeItem(int id);
	wxTreeItemId findRecursiveProjectTreeItem(const wxTreeItemId &startId, int id);
	void setProjectTreeItemBold(int id);
	void updateProjectDetails();
	void editPictureSet(R3DProject::PictureSet *pPictureSet);
	void clonePictureSet(R3DProject::PictureSet *pPictureSet);
	void deletePictureSet(R3DProject::PictureSet *pPictureSet);
	void addComputeMatches(R3DProject::PictureSet *pPictureSet);
	void showMatches(R3DProject::ComputeMatches *pComputeMatches);
	void triangulate(R3DProject::ComputeMatches *pComputeMatches);
	void deleteComputeMatches(R3DProject::ComputeMatches *pComputeMatches);
	void createDensePointcloud(R3DProject::Triangulation *pTriangulation);
	void showTriangulatedPoints(R3DProject::Triangulation *pTriangulation);
	void deleteTriangulation(R3DProject::Triangulation *pTriangulation);
	void createSurface(R3DProject::Densification *pDensification);
	void showDensePoints(R3DProject::Densification *pDensification);
	void deleteDensification(R3DProject::Densification *pDensification);
	void showSurface(R3DProject::Surface *pSurface);
	void deleteSurface(R3DProject::Surface *pSurface);
	void exportTriangulationToCMPMVS(R3DProject::Triangulation *pTriangulation);
	void exportPointCloud(R3DProject::Densification *pDensification);
	void exportDensificationToMeshLab(R3DProject::Densification *pDensification);
	void exportSurface(R3DProject::Surface *pSurface);

	void saveOrientation();
	void restoreOrientation();

	// Utility template function
	template<class C>
	C *getSpecializedProjectItem(const wxTreeItemId &selId, R3DProject::R3DTreeItem::R3DTreeItemType expectedType)
	{
		R3DProject::R3DTreeItem::R3DTreeItemType type = R3DProject::R3DTreeItem::TypeInvalid;
		int id = 0;
		if(!getProjectTreeItemDetails(selId, type, id))
			return NULL;

		if(type != expectedType)
			return NULL;

		R3DProject::Object *pObject = project_.getObjectByTypeAndID(type, id);
		if(pObject == NULL)
			return NULL;
		C *pSpecializedObject = dynamic_cast<C *>(pObject);

		return pSpecializedObject;	// Will be NULL if dynamic_cast failed
	}

private:
	bool isFirstTimeIdle_, continuousUpdate_;
	Regard3DDropTarget *pDropTarget_;
	wxTimer aTimer_;
	wxMenu *pTreeListPopupMenu_;
	wxTreeItemId contextMenuItemId_;

	// OpenSceneGraph viewer
	osg::ref_ptr<osgViewer::Viewer> viewer_;
	Regard3DModelViewHelper *pRegard3DModelViewHelper_;

	PreviewGeneratorThread previewGeneratorThread_;
	ImageInfoThread imageInfoThread_;

	R3DImageUpdatesInterface *pImageUpdatesDlg_;

	R3DComputeMatchesThread *pR3DComputeMatchesThread_;
	R3DTriangulationThread *pR3DTriangulationThread_;
	R3DDensificationProcess *pDensificationProcess_;
	R3DSurfaceGenProcess *pR3DSurfaceGenProcess_;
	R3DSmallTasksThread *pR3DSmallTasksThread_;

	R3DProject project_;

	Regard3DProgressDialog *pProgressDialog_;
	Regard3DConsoleOutputFrame *pRegard3DConsoleOutputFrame_;
	wxProgressDialog *pStdProgressDialog_;

	// Special case for OnTriangulationFinished
	wxArrayString resultStrings_;
	wxString htmlReportFilename_;

	DECLARE_EVENT_TABLE()
};

#endif
