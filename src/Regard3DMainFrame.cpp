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

// Enable the define to speed up the compilation of this file. Drawback: The About window has less information.
//#define R3D_SPEEDUP_COMPILATION

#include "CommonIncludes.h"

#include "Regard3DMainFrame.h"
#include "version.h"
#include "config.h"
#include "PreviewGeneratorThread.h"
#include "ImageInfoThread.h"
#include "Regard3DDropTarget.h"
#include "R3DComputeMatches.h"
#include "SharedGLContext.h"
#include "GraphicsWindowWX.h"
#include "R3DComputeMatchesThread.h"
#include "R3DTriangulationThread.h"
#include "OpenMVGHelper.h"
#include "R3DProject.h"
#include "R3DNewProjectDialog.h"
#include "Regard3DConsoleOutputFrame.h"
#include "R3DResultDialog.h"
#include "Regard3DProgressDialog.h"
#include "Regard3DModelViewHelper.h"
#include "Regard3DImagePreviewDialog.h"
#include "Regard3DPictureSetDialog.h"
#include "Regard3DComputeMatchesDialog.h"
#include "Regard3DTriangulationDialog.h"
#include "Regard3DMatchingResultsDialog.h"
#include "Regard3DDensificationDialog.h"
#include "R3DDensificationProcess.h"
#include "Regard3DSurfaceDialog.h"
#include "R3DSurfaceGenProcess.h"
#include "R3DModelOperations.h"
#include "R3DSmallTasksThread.h"
#include "Regard3DSettings.h"
#include "Regard3DPropertiesDialog.h"


// wxWidgets
#include <wx/mstream.h>
#include <wx/progdlg.h>
#include <wx/stdpaths.h>

// OpenSceneGraph
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#include <osg/Version>

#if !defined(R3D_SPEEDUP_COMPILATION)
// Eigen
#include <Eigen/Core>

// PointCloudLibrary
#include <pcl/pcl_config.h>

// boost
#include <boost/version.hpp>
#include <boost/thread.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// SuiteSparse
#include "SuiteSparse_config.h"

// Ceres
#include "ceres/version.h"

#endif

#include "res/png/icons_png.h"

enum
{
	ID_TIMER = 2000,
	ID_NEWPROJECTTOOL,
	ID_OPENPROJECTTOOL,
	ID_TEMP_DEV_TOOL,	// TODO: This is only a temporary development tool
	ID_PREVIEW_FINISHED,
	ID_NEW_IMAGE_INFOS,
	ID_UPDATE_PROGRESS_BAR,
	ID_COMPUTE_MATCHES_FINISHED,
	ID_TRIANGULATION_FINISHED,
	ID_DENSIFICATION_FINISHED,
	ID_SURFACE_GEN_FINISHED,
	ID_SMALL_TASK_FINISHED,
	ID_CTX_ADD_PICTURESET,
	ID_CTX_COMPUTE_MATCHES,
	ID_CTX_EDIT_PICTURESET,
	ID_CTX_CLONE_PICTURESET,
	ID_CTX_SHOW_MATCHES,
	ID_CTX_TRIANGULATION,
	ID_CTX_CREATE_DENSE_POINTCLOUD,
	ID_CTX_SHOW_TRIANGULATED_POINTS,
	ID_CTX_DELETE_PICTURESET,
	ID_CTX_DELETE_COMPUTE_MATCHES,
	ID_CTX_DELETE_TRIANGULATION,
	ID_CTX_CREATE_SURFACE,
	ID_CTX_SHOW_DENSE_POINTS,
	ID_CTX_DELETE_DENSIFICATION,
	ID_CTX_SHOW_SURFACE,
	ID_CTX_DELETE_SURFACE,
	ID_CTX_EXPORT_TRIANGULATION_TO_CMPMVS,
	ID_CTX_EXPORT_POINT_CLOUD,
	ID_CTX_EXPORT_DENSIFICATION_TO_MESHLAB,
	ID_CTX_EXPORT_SURFACE
};
   

const wxEventType myCustomEventType = wxNewEventType();

Regard3DMainFrame::Regard3DMainFrame(wxWindow* parent)
	: Regard3DMainFrameBase(parent),
	isFirstTimeIdle_(true), continuousUpdate_(false),
	aTimer_(this, ID_TIMER), pTreeListPopupMenu_(NULL),
	pImageUpdatesDlg_(NULL), pR3DComputeMatchesThread_(NULL),
	pR3DTriangulationThread_(NULL), pDensificationProcess_(NULL),
	pR3DSurfaceGenProcess_(NULL), pR3DSmallTasksThread_(NULL),
	pProgressDialog_(NULL), pStdProgressDialog_(NULL)
{
	GraphicsWindowWX* gw = new GraphicsWindowWX(pOSGGLCanvas_);
	pOSGGLCanvas_->setGraphicsWindow(gw);

	osgViewer::Viewer *viewer = new osgViewer::Viewer;
	viewer->getCamera()->setGraphicsContext(gw);
	pOSGGLCanvas_->setViewer(viewer);

	int width, height;
	pOSGGLCanvas_->GetSize(&width, &height);
	viewer->getCamera()->setViewport(0, 0, width, height);
	viewer->addEventHandler(new osgViewer::StatsHandler);
	viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);

	osgGA::TrackballManipulator *cameraManip = new osgGA::TrackballManipulator;

	// Disable "throwing": If the mouse is still moving while the button is released, the object keeps on moving
	cameraManip->setAllowThrow(false);

	// Make trackball smaller, i.e. object turns more quickly on mouse movements
	double trackballSize = cameraManip->getTrackballSize();
	cameraManip->setTrackballSize(trackballSize/2.0);

	viewer->setCameraManipulator(cameraManip);

	// Adds some shorcuts to enable/disable texturing, lighting, back-face culling etc.
//	viewer->addEventHandler( new osgGA::StateSetManipulator(viewer->getCamera()->getOrCreateStateSet()) );

	pRegard3DModelViewHelper_ = new Regard3DModelViewHelper();
	pRegard3DModelViewHelper_->setViewer(viewer);

	SetViewer(viewer);
	pOSGGLCanvas_->setMainFrame(this);
	pOSGGLCanvas_->setModelViewHelper(pRegard3DModelViewHelper_);


	previewGeneratorThread_.startPreviewThread();
	imageInfoThread_.startImageInfoThread();
	previewGeneratorThread_.setMainFrame(this);
	imageInfoThread_.setMainFrame(this);

	wxBitmap documentNewBitmap, documentOpenBitmap;
	{
		wxMemoryInputStream istream(document_new_png, sizeof(document_new_png));
		wxImage img(istream, wxBITMAP_TYPE_PNG);
		documentNewBitmap = wxBitmap(img);
	}
	{
		wxMemoryInputStream istream(document_open_png, sizeof(document_open_png));
		wxImage img(istream, wxBITMAP_TYPE_PNG);
		documentOpenBitmap = wxBitmap(img);
	}

	pMainFrameToolBar_->AddTool( ID_NEWPROJECTTOOL, wxT("New project"), documentNewBitmap, wxNullBitmap, wxITEM_NORMAL, wxT("Create new project..."), wxT("Create new project...") );
	pMainFrameToolBar_->AddTool( ID_OPENPROJECTTOOL, wxT("Open project"), documentOpenBitmap, wxNullBitmap, wxITEM_NORMAL, wxT("Open project..."), wxT("Open project...") ); 
//	pMainFrameToolBar_->AddTool( ID_TEMP_DEV_TOOL, wxT("Dev tool"), documentOpenBitmap, wxNullBitmap, wxITEM_NORMAL, wxT("Dev tool"), wxT("Dev tool") ); 

	pMainFrameToolBar_->Realize();

	SetIcon(getIcon());

/*
	pDropTarget_ = new Regard3DDropTarget();
	pDropTarget_->setMainFrame(this);
	SetDropTarget(pDropTarget_);	// Memory responsibility goes to wxWindow
*/
	R3DProject::setInstance(&project_);

	pRegard3DConsoleOutputFrame_ = new Regard3DConsoleOutputFrame(this);
	Regard3DSettings::getInstance().loadWindowLayoutFromConfig(this, pRegard3DConsoleOutputFrame_);
	updateUIFromProject(true);

	aTimer_.Start(10);
}

Regard3DMainFrame::~Regard3DMainFrame()
{
	delete pRegard3DModelViewHelper_;
	pRegard3DModelViewHelper_ = NULL;

	if(pTreeListPopupMenu_ != NULL)
	{
		delete pTreeListPopupMenu_;
		pTreeListPopupMenu_ = NULL;
	}
}

OSGGLCanvas *Regard3DMainFrame::getGLCanvas()
{
	return pOSGGLCanvas_;
}

void Regard3DMainFrame::SetViewer(osgViewer::Viewer *viewer)
{
    viewer_ = viewer;
}

bool Regard3DMainFrame::addDragAndDropFiles(const wxArrayString &filenames)
{
	return true;
}

void Regard3DMainFrame::sendNewPreviewImageEvent()
{
#if wxCHECK_VERSION(2, 9, 0)
	wxCommandEvent* newEvent = new wxCommandEvent(myCustomEventType, ID_PREVIEW_FINISHED);
	wxQueueEvent(this, newEvent);
#else
	wxCommandEvent newEvent(myCustomEventType, ID_PREVIEW_FINISHED);
	AddPendingEvent(newEvent);
#endif
}

void Regard3DMainFrame::sendNewImageInfoEvent()
{
#if wxCHECK_VERSION(2, 9, 0)
	wxCommandEvent* newEvent = new wxCommandEvent(myCustomEventType, ID_NEW_IMAGE_INFOS);
	wxQueueEvent(this, newEvent);
#else
	wxCommandEvent newEvent(myCustomEventType, ID_NEW_IMAGE_INFOS);
	AddPendingEvent(newEvent);
#endif
}

void Regard3DMainFrame::sendUpdateProgressBarEvent(float progress,
	const wxString &message)
{
#if wxCHECK_VERSION(2, 9, 0)
	wxCommandEvent* newEvent = new wxCommandEvent(myCustomEventType, ID_UPDATE_PROGRESS_BAR);
	newEvent->SetExtraLong( static_cast<long>(100.0f * progress) );
	newEvent->SetString(message);
	wxQueueEvent(this, newEvent);
#else
	wxCommandEvent newEvent(myCustomEventType, ID_UPDATE_PROGRESS_BAR);
	newEvent.SetExtraLong( static_cast<long>(100.0f * progress) );
	newEvent.SetString(message);
	AddPendingEvent(newEvent);
#endif
}

void Regard3DMainFrame::sendComputeMatchesFinishedEvent()
{
#if wxCHECK_VERSION(2, 9, 0)
	wxCommandEvent* newEvent = new wxCommandEvent(myCustomEventType, ID_COMPUTE_MATCHES_FINISHED);
	wxQueueEvent(this, newEvent);
#else
	wxCommandEvent newEvent(myCustomEventType, ID_COMPUTE_MATCHES_FINISHED);
	AddPendingEvent(newEvent);
#endif
}

void Regard3DMainFrame::sendTriangulationFinishedEvent()
{
#if wxCHECK_VERSION(2, 9, 0)
	wxCommandEvent* newEvent = new wxCommandEvent(myCustomEventType, ID_TRIANGULATION_FINISHED);
	wxQueueEvent(this, newEvent);
#else
	wxCommandEvent newEvent(myCustomEventType, ID_TRIANGULATION_FINISHED);
	AddPendingEvent(newEvent);
#endif
}

void Regard3DMainFrame::sendDensificationFinishedEvent()
{
#if wxCHECK_VERSION(2, 9, 0)
	wxCommandEvent* newEvent = new wxCommandEvent(myCustomEventType, ID_DENSIFICATION_FINISHED);
	wxQueueEvent(this, newEvent);
#else
	wxCommandEvent newEvent(myCustomEventType, ID_DENSIFICATION_FINISHED);
	AddPendingEvent(newEvent);
#endif
}

void Regard3DMainFrame::sendSurfaceGenFinishedEvent()
{
#if wxCHECK_VERSION(2, 9, 0)
	wxCommandEvent* newEvent = new wxCommandEvent(myCustomEventType, ID_SURFACE_GEN_FINISHED);
	wxQueueEvent(this, newEvent);
#else
	wxCommandEvent newEvent(myCustomEventType, ID_SURFACE_GEN_FINISHED);
	AddPendingEvent(newEvent);
#endif
}

void Regard3DMainFrame::sendSmallTaskFinishedEvent()
{
#if wxCHECK_VERSION(2, 9, 0)
	wxCommandEvent* newEvent = new wxCommandEvent(myCustomEventType, ID_SMALL_TASK_FINISHED);
	wxQueueEvent(this, newEvent);
#else
	wxCommandEvent newEvent(myCustomEventType, ID_SMALL_TASK_FINISHED);
	AddPendingEvent(newEvent);
#endif
}

void Regard3DMainFrame::OnMainFrameClose( wxCloseEvent& event )
{
	Regard3DSettings::getInstance().saveWindowLayoutToConfig(this, pRegard3DConsoleOutputFrame_);
	pRegard3DConsoleOutputFrame_->Destroy();

	saveOrientation();
	project_.save();
	viewer_->stopThreading();
	pOSGGLCanvas_->cleanup();

	aTimer_.Stop();

	previewGeneratorThread_.stopPreviewThread();
	imageInfoThread_.stopImageInfoThread();

	Destroy();
}

void Regard3DMainFrame::OnNewProject( wxCommandEvent& WXUNUSED(event) )
{
	R3DNewProjectDialog dlg(this);
	int retVal = dlg.ShowModal();
	if(retVal == wxID_OK)
	{
		bool isOK = project_.newProject(dlg.getProjectFilename());
		if(!isOK)
		{
			wxMessageBox(wxT("Error while creating the project!"),
				wxT("Error"), wxOK | wxICON_ERROR);
			return;
		}
		isOK = project_.save();
		if(!isOK)
		{
			wxMessageBox(wxT("Error while saving the project!"),
				wxT("Error"), wxOK | wxICON_ERROR);
			return;
		}

		updateUIFromProject(true);
		selectProjectTreeItem(0);
		updateProjectDetails();
	}
}

void Regard3DMainFrame::OnOpenProject( wxCommandEvent& WXUNUSED(event) )
{
	wxFileDialog chooseFileDialog(this, wxT("Choose Regard3D project"),
		wxEmptyString, wxEmptyString, wxT("Regard3D projects (*.r3d;*.R3D)|*.r3d;*.R3D|All files(*.*)|*.*"),
		wxFD_OPEN | wxFD_FILE_MUST_EXIST);
	if(chooseFileDialog.ShowModal() == wxID_OK)
	{
		bool isOK = project_.loadProject(chooseFileDialog.GetPath());
		if(!isOK)
		{
			wxMessageBox(wxT("Error while loading the project!"),
				wxT("Error"), wxOK | wxICON_ERROR);
			return;
		}

		if(!project_.ensureImageFilesArePresent())
		{
			wxMessageBox(wxT("Not all images of the project were found!"),
				wxT("Error"), wxOK | wxICON_ERROR);
			return;
		}
		updateUIFromProject(true);
		selectProjectTreeItem(0);
		updateProjectDetails();
	}
}

void Regard3DMainFrame::OnCloseProject( wxCommandEvent& WXUNUSED(event) )
{
	saveOrientation();
	project_.save();
	project_.closeProject();

	updateUIFromProject(true);
	updateProjectDetails();
}

void Regard3DMainFrame::OnMainFrameExitMenuItem( wxCommandEvent& WXUNUSED(event) )
{
	Close();
}

void Regard3DMainFrame::OnPropertiesMenuItem( wxCommandEvent& event )
{
	wxString defaultProjectPath = Regard3DSettings::getInstance().getDefaultProjectPath();
	if(defaultProjectPath.IsEmpty())
#if wxCHECK_VERSION(2, 9, 0)
		defaultProjectPath = wxStandardPaths::Get().GetAppDocumentsDir();
#else
	{
		wxFileName dpp(wxStandardPaths::Get().GetDocumentsDir(), wxEmptyString);
		dpp.AppendDir(wxTheApp->GetAppName());
		defaultProjectPath = dpp.GetPath(wxPATH_GET_VOLUME);
	}
#endif
	int mouseButtonType = (Regard3DSettings::getInstance().getIsMouseButtonSwitched() ? 1 : 0);
	int mouseWheelType = (Regard3DSettings::getInstance().getIsMouseWheelSwitched() ? 1 : 0);

	Regard3DPropertiesDialog dlg(this);
	dlg.pDefaultProjectPathDirPicker_->SetPath(defaultProjectPath);
	dlg.pMouseButtonRadioBox_->SetSelection(mouseButtonType);
	dlg.pMouseWheelRadioBox_->SetSelection(mouseWheelType);

	int retVal = dlg.ShowModal();
	if(retVal == wxID_OK)
	{
		defaultProjectPath = dlg.pDefaultProjectPathDirPicker_->GetPath();
		mouseButtonType = dlg.pMouseButtonRadioBox_->GetSelection();
		mouseWheelType = dlg.pMouseWheelRadioBox_->GetSelection();
		Regard3DSettings::getInstance().setDefaultProjectPath(defaultProjectPath);
		bool isMouseButtonSwitched = (mouseButtonType == 1);
		Regard3DSettings::getInstance().setIsMouseButtonSwitched( isMouseButtonSwitched );
		pOSGGLCanvas_->setIsMouseButtonSwitched(isMouseButtonSwitched);

		bool isMouseWheelSwitched = (mouseWheelType == 1);
		Regard3DSettings::getInstance().setIsMouseWheelSwitched( isMouseWheelSwitched );
		pOSGGLCanvas_->setIsMouseWheelSwitched(isMouseWheelSwitched);
	}
}

void Regard3DMainFrame::OnViewConsoleOutputFrameMenuItem( wxCommandEvent& event )
{
	pRegard3DConsoleOutputFrame_->Show(event.IsChecked());
}

void Regard3DMainFrame::OnViewConsoleOutputFrameUpdate( wxUpdateUIEvent& event )
{
	event.Check(pRegard3DConsoleOutputFrame_->IsShown());
}

//#define R3D_GET_OPENBLAS_INFO
#if defined(R3D_GET_OPENBLAS_INFO)
/*Get the number of threads on runtime.*/
extern "C" int openblas_get_num_threads(void);

/*Get the number of physical processors (cores).*/
extern "C" int openblas_get_num_procs(void);

/*Get the build configure on runtime.*/
extern "C" char* openblas_get_config(void);

/*Get the CPU corename on runtime.*/
extern "C" char* openblas_get_corename(void);

/* Get the parallelization type which is used by OpenBLAS */
extern "C" int openblas_get_parallel(void); 
/* OpenBLAS is compiled for sequential use  */
#define OPENBLAS_SEQUENTIAL  0
/* OpenBLAS is compiled using normal threading model */
#define OPENBLAS_THREAD  1 
/* OpenBLAS is compiled using OpenMP threading model */
#define OPENBLAS_OPENMP 2

#endif

void Regard3DMainFrame::OnAboutMenuItem( wxCommandEvent& event )
{
	wxAboutDialogInfo info;
	info.SetName(wxT(REGARD3D_NAME) wxT(" ") wxT(REGARD3D_ARCHITECTURE_STRING));
	info.SetVersion(wxString(REGARD3D_VERSION_STRING, *wxConvCurrent));
	info.SetIcon(getIcon());

	wxString openblasStr;

#if defined(R3D_GET_OPENBLAS_INFO)
#	if defined(_MSC_VER)
	typedef int (*openblas_get_num_procs_ptr)(void);
	typedef char * (*openblas_get_config_ptr)(void);
	typedef char * (*openblas_get_corename_ptr)(void);
	typedef int (*openblas_get_parallel_ptr)(void);
	openblas_get_num_procs_ptr openblas_get_num_procs_m;
	openblas_get_config_ptr openblas_get_config_m;
	openblas_get_corename_ptr openblas_get_corename_m;
	openblas_get_parallel_ptr openblas_get_parallel_m;
	openblas_get_num_procs_m = (openblas_get_num_procs_ptr)::GetProcAddress(GetModuleHandle(NULL), "openblas_get_num_procs");
	if(openblas_get_num_procs_m == NULL)
		openblas_get_num_procs_m = (openblas_get_num_procs_ptr)::GetProcAddress(GetModuleHandle(L"libopenblas.dll"), "openblas_get_num_procs");
	openblas_get_config_m = (openblas_get_config_ptr)::GetProcAddress(GetModuleHandle(NULL), "openblas_get_config");
	if(openblas_get_config_m == NULL)
		openblas_get_config_m = (openblas_get_config_ptr)::GetProcAddress(GetModuleHandle(L"libopenblas.dll"), "openblas_get_config");
	openblas_get_corename_m = (openblas_get_corename_ptr)::GetProcAddress(GetModuleHandle(NULL), "openblas_get_corename");
	if(openblas_get_corename_m == NULL)
		openblas_get_corename_m = (openblas_get_corename_ptr)::GetProcAddress(GetModuleHandle(L"libopenblas.dll"), "openblas_get_corename");
	openblas_get_parallel_m = (openblas_get_parallel_ptr)::GetProcAddress(GetModuleHandle(NULL), "openblas_get_parallel");
	if(openblas_get_parallel_m == NULL)
		openblas_get_parallel_m = (openblas_get_parallel_ptr)::GetProcAddress(GetModuleHandle(L"libopenblas.dll"), "openblas_get_parallel");
	if(openblas_get_config_m != NULL)
	{
		const char *config = (*openblas_get_config_m)();
		openblasStr = wxT("\n OpenBLAS (");
		openblasStr.Append( wxString(config, wxConvLibc) );
		openblasStr.Append( wxT(")") );
	}
	if(openblas_get_parallel_m != NULL
		&& openblas_get_num_procs_m != NULL
		&& openblas_get_corename_m != NULL)
	{
		int par = (*openblas_get_parallel_m)();
		char buf[100];
		sprintf(buf, "parallel: %d, procs: %d CPU corename: %s\n", par,
			(*openblas_get_num_procs_m)(), (*openblas_get_corename_m)());
		OutputDebugStringA(buf);
	}
#	else
	const char *config = openblas_get_config();
	int par = openblas_get_parallel();
	char buf[100];
	sprintf(buf, "parallel: %d, threads: %d procs: %d CPU corename: %s\n", par,
		openblas_get_num_threads(), openblas_get_num_procs(), openblas_get_corename());
	openblasStr = wxT("\n OpenBLAS (");
	openblasStr.Append( wxString(config, wxConvLibc) );
	openblasStr.Append( wxT(")") );
#	endif
#endif

	wxString descrString;
	descrString = wxT("This program creates 3D models from 2D photographs.\n\n");
	descrString.Append(wxT("Based on the OpenMVG Structure from Motion engine.\n"));
	descrString.Append(wxT("Uses AKAZE and LIOP for feature detection and descriptor extraction.\n\n"));
	descrString.Append(wxT("Built with: ") wxT(REGARD3D_COMPILER) wxT(" ") wxT(REGARD3D_COMPILER_VERSION) wxT("\n"));
#if defined(R3D_HAVE_OPENMP)
	descrString.Append(wxT("OpenMP enabled\n"));
#endif
	descrString.Append(wxT("Linked with:\n ") wxVERSION_STRING wxT("\n"));
	descrString.Append(wxT(" "));
#if !defined(R3D_SPEEDUP_COMPILATION)
	descrString.Append(wxString::Format(wxT("Boost %d.%d.%d"),
		BOOST_VERSION / 100000,
		BOOST_VERSION / 100 % 1000,
		BOOST_VERSION % 100));
	descrString.Append(wxT("\n "));
	descrString.Append(wxString::Format(wxT("Eigen %d.%d.%d"),
		EIGEN_WORLD_VERSION,
		EIGEN_MAJOR_VERSION,
		EIGEN_MINOR_VERSION));
	descrString.Append(wxT("\n "));
	descrString.Append(wxString(osgGetLibraryName(), *wxConvCurrent));
	descrString.Append(wxT(" "));
	descrString.Append(wxString(osgGetVersion(), *wxConvCurrent));
	descrString.Append(wxT("\n "));
	descrString.Append( wxT("OpenCV ") );
	descrString.Append( wxString(CV_VERSION, *wxConvCurrent) );
	descrString.Append(wxT("\n "));
	descrString.Append(wxString("PointCloudLibrary " PCL_VERSION_PRETTY, *wxConvCurrent));
	descrString.Append(wxT("\n SuiteSparse "));
#if defined(SUITESPARSE_HAS_VERSION_FUNCTION)
	int ss_version[3];
	SuiteSparse_version(ss_version);
	descrString.Append(wxString::Format(wxT("%d.%d.%d"), ss_version[0], ss_version[1], ss_version[2]));
#else
	descrString.Append(wxString::Format(wxT("%d.%d.%d"), SUITESPARSE_MAIN_VERSION, SUITESPARSE_SUB_VERSION, SUITESPARSE_SUBSUB_VERSION));
#endif
	descrString.Append(wxT("\n Ceres ") );
	descrString.Append( wxString(CERES_VERSION_STRING, *wxConvCurrent) );
#endif
	descrString.Append(openblasStr);
	info.SetDescription(descrString);
	info.SetCopyright(wxString(wxT("\u00A9 ")) + wxString(wxT(REGARD3D_COPYRIGHT_YEAR))
		+ wxString(wxT(" ")) + wxString(wxT(REGARD3D_COPYRIGHT_NAME)));

	wxAboutBox(info);
}

void Regard3DMainFrame::OnProjectTreeItemActivated( wxTreeEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = event.GetItem();

	// Try out the different types
	R3DProject::PictureSet *pPictureSet = getSpecializedProjectItem<R3DProject::PictureSet>(selId,
		R3DProject::R3DTreeItem::TypePictureSet);
	if(pPictureSet != NULL)
		editPictureSet(pPictureSet);

	R3DProject::ComputeMatches *pComputeMatches = getSpecializedProjectItem<R3DProject::ComputeMatches>(selId,
		R3DProject::R3DTreeItem::TypeComputeMatches);
	if(pComputeMatches != NULL)
		showMatches(pComputeMatches);

	R3DProject::Triangulation *pTriangulation = getSpecializedProjectItem<R3DProject::Triangulation>(selId,
		R3DProject::R3DTreeItem::TypeTriangulation);
	if(pTriangulation != NULL)
		showTriangulatedPoints(pTriangulation);

	R3DProject::Densification *pDensification = getSpecializedProjectItem<R3DProject::Densification>(selId,
		R3DProject::R3DTreeItem::TypeDensification);
	if(pDensification != NULL)
		showDensePoints(pDensification);

	R3DProject::Surface *pSurface = getSpecializedProjectItem<R3DProject::Surface>(selId,
		R3DProject::R3DTreeItem::TypeSurface);
	if(pSurface != NULL)
		showSurface(pSurface);
}

void Regard3DMainFrame::OnProjectTreeItemMenu( wxTreeEvent& event )
{
	// Get tree item
	contextMenuItemId_ = event.GetItem();
	if(!contextMenuItemId_.IsOk())
		return;
	R3DProject::R3DTreeItem::R3DTreeItemType type = R3DProject::R3DTreeItem::TypeInvalid;
	int id = 0;
	wxTreeItemData *pData = pProjectTreeCtrl_->GetItemData(contextMenuItemId_);
	if(pData != NULL)
	{
		R3DProject::R3DTreeItem *pTreeItem = dynamic_cast<R3DProject::R3DTreeItem*>(pData);
		if(pTreeItem != NULL)
		{
			type = pTreeItem->getType();
			id = pTreeItem->getID();
		}
	}

	R3DProject::Object *pObject = project_.getObjectByTypeAndID(type, id);

	if(pTreeListPopupMenu_ != NULL)
	{
		delete pTreeListPopupMenu_;
		pTreeListPopupMenu_ = NULL;
	}

	if(type == R3DProject::R3DTreeItem::TypeProject)
	{
		pTreeListPopupMenu_ = new wxMenu();
		pTreeListPopupMenu_->Append(ID_CTX_ADD_PICTURESET, wxT("Add Picture Set..."));
	}
	else if(type == R3DProject::R3DTreeItem::TypePictureSet)
	{
		bool showOnly = false;
		R3DProject::PictureSet *pPictureSet = dynamic_cast<R3DProject::PictureSet *>(pObject);
		if(pPictureSet != NULL)
			showOnly = !(pPictureSet->computeMatches_.empty());
		pTreeListPopupMenu_ = new wxMenu();
		pTreeListPopupMenu_->Append(ID_CTX_COMPUTE_MATCHES, wxT("Compute matches..."));
		if(showOnly)
			pTreeListPopupMenu_->Append(ID_CTX_EDIT_PICTURESET, wxT("Show picture set..."));
		else
			pTreeListPopupMenu_->Append(ID_CTX_EDIT_PICTURESET, wxT("Edit picture set..."));
		pTreeListPopupMenu_->Append(ID_CTX_CLONE_PICTURESET, wxT("Clone picture set"));
		pTreeListPopupMenu_->Append(ID_CTX_DELETE_PICTURESET, wxT("Delete"));
	}
	else if(type == R3DProject::R3DTreeItem::TypeComputeMatches)
	{
		pTreeListPopupMenu_ = new wxMenu();
		pTreeListPopupMenu_->Append(ID_CTX_SHOW_MATCHES, wxT("Show matching results..."));
		pTreeListPopupMenu_->Append(ID_CTX_TRIANGULATION, wxT("Triangulation..."));
		pTreeListPopupMenu_->Append(ID_CTX_DELETE_COMPUTE_MATCHES, wxT("Delete"));
	}
	else if(type == R3DProject::R3DTreeItem::TypeTriangulation)
	{
		pTreeListPopupMenu_ = new wxMenu();
		pTreeListPopupMenu_->Append(ID_CTX_CREATE_DENSE_POINTCLOUD, wxT("Create dense pointcloud..."));
		pTreeListPopupMenu_->Append(ID_CTX_SHOW_TRIANGULATED_POINTS, wxT("Show triangulated points"));
		pTreeListPopupMenu_->Append(ID_CTX_EXPORT_TRIANGULATION_TO_CMPMVS, wxT("Export project to CMPMVS"));
		pTreeListPopupMenu_->Append(ID_CTX_DELETE_TRIANGULATION, wxT("Delete"));
	}
	else if(type == R3DProject::R3DTreeItem::TypeDensification)
	{
		pTreeListPopupMenu_ = new wxMenu();
		pTreeListPopupMenu_->Append(ID_CTX_CREATE_SURFACE, wxT("Create Surface..."));
		pTreeListPopupMenu_->Append(ID_CTX_SHOW_DENSE_POINTS, wxT("Show point cloud"));
		pTreeListPopupMenu_->Append(ID_CTX_EXPORT_POINT_CLOUD, wxT("Export point cloud"));
		pTreeListPopupMenu_->Append(ID_CTX_EXPORT_DENSIFICATION_TO_MESHLAB, wxT("Export scene to MeshLab"));
		pTreeListPopupMenu_->Append(ID_CTX_DELETE_DENSIFICATION, wxT("Delete"));
	}
	else if(type == R3DProject::R3DTreeItem::TypeSurface)
	{
		pTreeListPopupMenu_ = new wxMenu();
		pTreeListPopupMenu_->Append(ID_CTX_SHOW_SURFACE, wxT("Show surface"));
		pTreeListPopupMenu_->Append(ID_CTX_EXPORT_SURFACE, wxT("Export surface"));
		pTreeListPopupMenu_->Append(ID_CTX_DELETE_SURFACE, wxT("Delete"));
	}

	if(pTreeListPopupMenu_ != NULL)
		PopupMenu(pTreeListPopupMenu_);

}

void Regard3DMainFrame::OnProjectTreeSelChanged( wxTreeEvent& event )
{
	updateProjectDetails();
}

void Regard3DMainFrame::OnProjectTreeSelChanging( wxTreeEvent& event )
{
	// Possibility to veto a selection event
}

void Regard3DMainFrame::OnProjectAddPictureSetButton( wxCommandEvent& event )
{
	Regard3DPictureSetDialog dlg(this);
	dlg.setPreviewGeneratorThread(&previewGeneratorThread_);
	dlg.setImageInfoThread(&imageInfoThread_);
	pImageUpdatesDlg_ = &dlg;

	int retVal = wxID_CANCEL;
	try
	{
		retVal = dlg.ShowModal();
	}
	catch(...)
	{
		pImageUpdatesDlg_ = NULL;
		throw;
	}
	pImageUpdatesDlg_ = NULL;

	if(retVal == wxID_OK)
	{
		int psid = dlg.updateProject(&project_);
		project_.populateTreeControl(pProjectTreeCtrl_);
		project_.save();
		selectProjectTreeItem(psid);
	}
}

void Regard3DMainFrame::OnPictureSetComputeMatchesButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::PictureSet *pPictureSet = getSpecializedProjectItem<R3DProject::PictureSet>(selId,
		R3DProject::R3DTreeItem::TypePictureSet);
	if(pPictureSet != NULL)
		addComputeMatches(pPictureSet);
}

void Regard3DMainFrame::OnEditPictureSetButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::PictureSet *pPictureSet = getSpecializedProjectItem<R3DProject::PictureSet>(selId,
		R3DProject::R3DTreeItem::TypePictureSet);
	if(pPictureSet != NULL)
		editPictureSet(pPictureSet);
}

void Regard3DMainFrame::OnClonePictureSetButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::PictureSet *pPictureSet = getSpecializedProjectItem<R3DProject::PictureSet>(selId,
		R3DProject::R3DTreeItem::TypePictureSet);
	if(pPictureSet != NULL)
		clonePictureSet(pPictureSet);
}

void Regard3DMainFrame::OnProjectDeletePictureSetButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::PictureSet *pPictureSet = getSpecializedProjectItem<R3DProject::PictureSet>(selId,
		R3DProject::R3DTreeItem::TypePictureSet);
	if(pPictureSet != NULL)
		deletePictureSet(pPictureSet);
}

void Regard3DMainFrame::OnShowMatchingResultsButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::ComputeMatches *pComputeMatches = getSpecializedProjectItem<R3DProject::ComputeMatches>(selId,
		R3DProject::R3DTreeItem::TypeComputeMatches);
	if(pComputeMatches != NULL)
		showMatches(pComputeMatches);
}

void Regard3DMainFrame::OnTriangulationButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::ComputeMatches *pComputeMatches = getSpecializedProjectItem<R3DProject::ComputeMatches>(selId,
		R3DProject::R3DTreeItem::TypeComputeMatches);
	if(pComputeMatches != NULL)
		triangulate(pComputeMatches);
}

void Regard3DMainFrame::OnProjectDeleteComputeMatchesButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::ComputeMatches *pComputeMatches = getSpecializedProjectItem<R3DProject::ComputeMatches>(selId,
		R3DProject::R3DTreeItem::TypeComputeMatches);
	if(pComputeMatches != NULL)
		deleteComputeMatches(pComputeMatches);
}

void Regard3DMainFrame::OnCreateDensePointcloudButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::Triangulation *pTriangulation = getSpecializedProjectItem<R3DProject::Triangulation>(selId,
		R3DProject::R3DTreeItem::TypeTriangulation);
	if(pTriangulation != NULL)
		createDensePointcloud(pTriangulation);
}

void Regard3DMainFrame::OnShowTriangulatedPoints( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::Triangulation *pTriangulation = getSpecializedProjectItem<R3DProject::Triangulation>(selId,
		R3DProject::R3DTreeItem::TypeTriangulation);
	if(pTriangulation != NULL)
		showTriangulatedPoints(pTriangulation);
}

void Regard3DMainFrame::OnExportTriangulationToCMPMVSButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::Triangulation *pTriangulation = getSpecializedProjectItem<R3DProject::Triangulation>(selId,
		R3DProject::R3DTreeItem::TypeTriangulation);
	if(pTriangulation != NULL)
		exportTriangulationToCMPMVS(pTriangulation);
}

void Regard3DMainFrame::OnProjectDeleteTriangulationButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::Triangulation *pTriangulation = getSpecializedProjectItem<R3DProject::Triangulation>(selId,
		R3DProject::R3DTreeItem::TypeTriangulation);
	if(pTriangulation != NULL)
		deleteTriangulation(pTriangulation);
}

void Regard3DMainFrame::OnCreateSurfaceButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::Densification *pDensification = getSpecializedProjectItem<R3DProject::Densification>(selId,
		R3DProject::R3DTreeItem::TypeDensification);
	if(pDensification != NULL)
		createSurface(pDensification);
}

void Regard3DMainFrame::OnShowDensePointCloudButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::Densification *pDensification = getSpecializedProjectItem<R3DProject::Densification>(selId,
		R3DProject::R3DTreeItem::TypeDensification);
	if(pDensification != NULL)
		showDensePoints(pDensification);
}

void Regard3DMainFrame::OnProjectDeleteDensificationButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::Densification *pDensification = getSpecializedProjectItem<R3DProject::Densification>(selId,
		R3DProject::R3DTreeItem::TypeDensification);
	if(pDensification != NULL)
		deleteDensification(pDensification);
}

void Regard3DMainFrame::OnExportDensificationButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::Densification *pDensification = getSpecializedProjectItem<R3DProject::Densification>(selId,
		R3DProject::R3DTreeItem::TypeDensification);
	if(pDensification != NULL)
		exportPointCloud(pDensification);
}

void Regard3DMainFrame::OnExportDensificationToMeshLabButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::Densification *pDensification = getSpecializedProjectItem<R3DProject::Densification>(selId,
		R3DProject::R3DTreeItem::TypeDensification);
	if(pDensification != NULL)
		exportDensificationToMeshLab(pDensification);
}

void Regard3DMainFrame::OnShowSurfaceButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::Surface *pSurface = getSpecializedProjectItem<R3DProject::Surface>(selId,
		R3DProject::R3DTreeItem::TypeSurface);
	if(pSurface != NULL)
		showSurface(pSurface);
}

void Regard3DMainFrame::OnExportSurfaceButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::Surface *pSurface = getSpecializedProjectItem<R3DProject::Surface>(selId,
		R3DProject::R3DTreeItem::TypeSurface);
	if(pSurface != NULL)
		exportSurface(pSurface);
}

void Regard3DMainFrame::OnDeleteSurfaceButton( wxCommandEvent& event )
{
	// Get selected TreeViewCtrl item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::Surface *pSurface = getSpecializedProjectItem<R3DProject::Surface>(selId,
		R3DProject::R3DTreeItem::TypeSurface);
	if(pSurface != NULL)
		deleteSurface(pSurface);
}

void Regard3DMainFrame::OnMainFrameIdle( wxIdleEvent& event )
{
	if(isFirstTimeIdle_)
	{
		if(!pOSGGLCanvas_->setup())
		{
			wxMessageBox(wxT("Setting up the OpenGL resources failed"),
				wxT("Application error"), wxICON_ERROR | wxOK, this);
			Close();
		}

		// Set no context as active (to allow context to be used in other thread)
		SharedGLContext::setNoActiveContext(pOSGGLCanvas_);

		viewer_->realize();

		// Correct aspect ratio
		double fovy, aspectRatio, zNear, zFar;
		if(viewer_->getCamera()->getProjectionMatrixAsPerspective(fovy,
			aspectRatio, zNear, zFar))
		{
			int w = 0, h = 0;
			pOSGGLCanvas_->GetSize(&w, &h);
			viewer_->getCamera()->setProjectionMatrixAsPerspective(fovy,
				static_cast<double>(w) / static_cast<double>(h), zNear, zFar);
		}
		pOSGGLCanvas_->setOK();
		pOSGGLCanvas_->Refresh();
		pOSGGLCanvas_->setIsMouseButtonSwitched(Regard3DSettings::getInstance().getIsMouseButtonSwitched());
		pOSGGLCanvas_->setIsMouseWheelSwitched(Regard3DSettings::getInstance().getIsMouseWheelSwitched());

		updateProjectDetails();

		isFirstTimeIdle_ = false;
	}

	if (!viewer_->isRealized())
		return;
}

void Regard3DMainFrame::OnShowTrackballCheckBox( wxCommandEvent& event )
{
	bool showTrackball = (event.IsChecked());
	if(pRegard3DModelViewHelper_ != NULL)
	{
		pRegard3DModelViewHelper_->showTrackball(showTrackball);
		pOSGGLCanvas_->Refresh();
	}
}

void Regard3DMainFrame::OnPointSizeScroll( wxScrollEvent& event )
{
	int sliderValue = pPointSizeSlider_->GetValue();
	double pointSize = static_cast<double>(sliderValue);
	pPointSizeTextCtrl_->SetValue( wxString::Format(wxT("%.1g"), pointSize) );

	if(pRegard3DModelViewHelper_ != NULL)
	{
		pRegard3DModelViewHelper_->setPointSize(pointSize);
		pOSGGLCanvas_->Refresh();
	}
}

void Regard3DMainFrame::OnShowTextureCheckBox( wxCommandEvent& event )
{
	bool showTexture = (event.IsChecked());
	if(pRegard3DModelViewHelper_ != NULL)
	{
		pRegard3DModelViewHelper_->showTexture(showTexture);
		pOSGGLCanvas_->Refresh();
	}
}

void Regard3DMainFrame::OnEnableLightingCheckBox( wxCommandEvent& event )
{
	bool enableLighting = (event.IsChecked());
	if(pRegard3DModelViewHelper_ != NULL)
	{
		pRegard3DModelViewHelper_->enableLighting(enableLighting);
		pOSGGLCanvas_->Refresh();
	}
}

void Regard3DMainFrame::OnPolygonModeRadioBox( wxCommandEvent& event )
{
	int polygonMode = (event.GetInt());
	if(pRegard3DModelViewHelper_ != NULL)
	{
		pRegard3DModelViewHelper_->polygonMode(polygonMode);
		pOSGGLCanvas_->Refresh();
	}
}

void Regard3DMainFrame::OnShadingModelRadioBox( wxCommandEvent& event )
{
	int shadingModel = (event.GetInt());
	if(pRegard3DModelViewHelper_ != NULL)
	{
		pRegard3DModelViewHelper_->shadingModel(shadingModel);
		pOSGGLCanvas_->Refresh();
	}
}

void Regard3DMainFrame::OnResetViewButton( wxCommandEvent& event )
{
	viewer_->getCameraManipulator()->home(0);
	pOSGGLCanvas_->Refresh();
}

/*
// OpenSceneGraph
#include <osgViewer/Viewer>
//#include <osg/ShapeDrawable>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Point>
#include <osg/Drawable>
#include <osg/NodeCallback>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReaderWriter>
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
*/
void Regard3DMainFrame::OnTempDevToolClicked( wxCommandEvent& event )
{
/*	wxArrayString strings;
	strings.Add(wxT("str1"));
	strings.Add(wxT("str2"));
	strings.Add(wxT("str3"));
	strings.Add(wxT("str4"));
	strings.Add(wxT("str5"));
	strings.Add(wxT("str6"));
	R3DResultDialog dlg(this, wxT("Gugus"),
		wxT("Triangulation was successful!\n\nStatistics:"), strings);
	dlg.ShowModal();
	*/
	//OpenMVGHelper::exportToPMVS(project_.getRelativeSfMOutPath(), "PMVS", 1, 4);
/*
	wxString fname(wxT("F:\\Projects\\libs\\openMVG\\run\\results\\cmpmvs_OUT\\meshAvImgCol.wrl"));
	osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile(std::string(fname.mb_str()));
	if (!loadedModel)
	{
		//std::cout << argv[0] <<": No data loaded." << std::endl;
		//return false;
	}
	else
	{

		// optimize the scene graph, remove redundant nodes and state etc.
		osgUtil::Optimizer optimizer;
		optimizer.optimize(loadedModel.get());

		viewer_->setSceneData(loadedModel.get());
	}
	*/
}

void Regard3DMainFrame::OnPreviewFinished( wxCommandEvent& WXUNUSED(event) )
{
	if(pImageUpdatesDlg_ != NULL)
		pImageUpdatesDlg_->OnPreviewFinished();
}

void Regard3DMainFrame::OnNewImageInfos( wxCommandEvent& WXUNUSED(event) )
{
	if(pImageUpdatesDlg_ != NULL)
		pImageUpdatesDlg_->OnNewImageInfos();
}

void Regard3DMainFrame::OnUpdateProgressBar( wxCommandEvent &event )
{
	if(pProgressDialog_ != NULL)
	{
		int progress = static_cast<int>(event.GetExtraLong());
		wxString progressStr = event.GetString();
		if(progress >= 0)
			pProgressDialog_->Update(event.GetExtraLong(), progressStr);
		else
			pProgressDialog_->Pulse(progressStr);

		pProgressDialog_->Fit();
	}
}

void Regard3DMainFrame::OnComputeMatchesFinished( wxCommandEvent &event )
{
	wxArrayString resultStrings;

	if(pProgressDialog_ != NULL)
	{
		pProgressDialog_->EndDialog();

		delete pProgressDialog_;
		pProgressDialog_ = NULL;
	}

	bool isOK = false;
	wxString errorMessage;
	Regard3DFeatures::R3DFParams params;
	R3DProject::ComputeMatches *pComputeMatches = NULL;
	if(pR3DComputeMatchesThread_ != NULL)
	{
		pR3DComputeMatchesThread_->stopComputeMatchesThread();
		isOK = pR3DComputeMatchesThread_->getIsOK();
		errorMessage = pR3DComputeMatchesThread_->getErrorMessage();
		resultStrings = pR3DComputeMatchesThread_->getResultStrings();
		params = pR3DComputeMatchesThread_->getParameters();
		pComputeMatches = pR3DComputeMatchesThread_->getComputeMatches();
		delete pR3DComputeMatchesThread_;
		pR3DComputeMatchesThread_ = NULL;
	}

	if(isOK)
	{
		pComputeMatches->state_ = R3DProject::OSFinished;
		project_.save();
		updateProjectDetails();

		R3DResultDialog dlg(this, wxT("Compute matches results"),
			wxT("Computing matches was successful!\n\nStatistics:"),
			resultStrings, wxEmptyString);
		dlg.ShowModal();
	}
	else
	{
		project_.removeComputeMatches(pComputeMatches);
		project_.save();
		project_.populateTreeControl(pProjectTreeCtrl_);

		wxMessageBox(errorMessage,
			wxT("Compute matches"), wxICON_ERROR | wxOK, this);
	}
}

void Regard3DMainFrame::OnTriangulationFinished( wxCommandEvent &event )
{
	bool isOK = true;
	wxString errorMessage;
	resultStrings_.Clear();
	R3DProject::Triangulation *pTriangulation = NULL;
	if(pR3DTriangulationThread_ != NULL)
	{
		pR3DTriangulationThread_->stopTriangulationThread();
		isOK = pR3DTriangulationThread_->getIsOK();
		errorMessage = pR3DTriangulationThread_->getErrorMessage();
		resultStrings_ = pR3DTriangulationThread_->getResultStrings();
		pTriangulation = pR3DTriangulationThread_->getTriangulation();
		delete pR3DTriangulationThread_;
		pR3DTriangulationThread_ = NULL;
	}

	if(pProgressDialog_ != NULL)
	{
		pProgressDialog_->EndDialog();
		delete pProgressDialog_;
		pProgressDialog_ = NULL;
	}

	R3DProjectPaths paths;
	project_.getProjectPathsTri(paths, pTriangulation);

	if(isOK)
	{
		pTriangulation->state_ = R3DProject::OSFinished;
		project_.save();

		// Load model if it exists
		wxFileName modelName(wxString(paths.relativeOutPath_.c_str(), *wxConvCurrent), wxT("FinalColorized.ply"));
//		wxFileName modelName(project_.getAbsoluteOutPath(), wxT("FinalColorized.ply"));
		if(modelName.FileExists())
		{
			load3DModel(modelName.GetFullPath());
			setProjectTreeItemBold(pTriangulation->id_);
		}
		else
		{
			setProjectTreeItemBold(-1);
			wxMessageBox(wxT("Error: Result file not found"),
				wxT("Show triangulated points"), wxICON_ERROR | wxOK, this);
		}
		updateProjectDetails();
		wxFileName htmlReport(paths.absoluteOutPath_, wxT("Reconstruction_Report.html"));
		htmlReportFilename_ = htmlReport.GetFullPath();
	}
	else
	{
		project_.removeTriangulation(pTriangulation);
		project_.save();
		project_.populateTreeControl(pProjectTreeCtrl_);

		wxMessageBox(errorMessage,
			wxT("Triangulation"), wxICON_ERROR | wxOK, this);
	}

}

void Regard3DMainFrame::OnDensificationFinished( wxCommandEvent &event )
{
	R3DProject::Densification *pDensification = NULL;
	int numberOfClusters = 1;
	if(pDensificationProcess_ != NULL)
	{
		pDensificationProcess_->readConsoleOutput();	// Consume all output
		pDensification = pDensificationProcess_->getDensification();
		pDensification->runningTime_ = pDensificationProcess_->getRuntimeStr();
		numberOfClusters = pDensificationProcess_->getNumberOfClusters();

		delete pDensificationProcess_;
		pDensificationProcess_ = NULL;
	}

	if(numberOfClusters > 1)
	{
		if(pProgressDialog_ != NULL)
			pProgressDialog_->Update(80, wxT("Combining generated models"));

		// Combine models into one, update pDensification->finalDenseModelName_

		if(pR3DSmallTasksThread_ != NULL)
			delete pR3DSmallTasksThread_;

		pR3DSmallTasksThread_ = new R3DSmallTasksThread();
		pR3DSmallTasksThread_->setMainFrame(this);
		pR3DSmallTasksThread_->combineDenseModels(pDensification, numberOfClusters);

		// ...to be continued in OnSmallTaskFinished
		return;
	}

	if(pProgressDialog_ != NULL)
	{
		pProgressDialog_->EndDialog();
		delete pProgressDialog_;
		pProgressDialog_ = NULL;
	}


	// Load generated model
	R3DProjectPaths paths;
	project_.getProjectPathsDns(paths, pDensification);

	wxFileName denseModelFN(wxString(paths.relativeDenseModelName_.c_str(), wxConvLibc));
	if(denseModelFN.FileExists())
	{
		pDensification->state_ = R3DProject::OSFinished;
		project_.save();

		load3DModel(denseModelFN.GetFullPath());
		setProjectTreeItemBold(pDensification->id_);
		updateProjectDetails();
	}
	else
	{
		setProjectTreeItemBold(-1);
		wxMessageBox(wxT("No generated model found.\nPlease check console output for errors."),
			wxT("Densification"), wxICON_ERROR | wxOK, this);

		project_.removeDensification(pDensification);
		project_.save();
		project_.populateTreeControl(pProjectTreeCtrl_);
	}

}

void Regard3DMainFrame::OnSurfaceGenFinished( wxCommandEvent &event )
{
	R3DProject::Surface *pSurface = NULL;
	if(pR3DSurfaceGenProcess_ != NULL)
	{
		pR3DSurfaceGenProcess_->readConsoleOutput();	// Consume all output
		pSurface = pR3DSurfaceGenProcess_->getSurface();
		pSurface->runningTime_ = pR3DSurfaceGenProcess_->getRuntimeStr();

		delete pR3DSurfaceGenProcess_;
		pR3DSurfaceGenProcess_ = NULL;
	}

	wxString surfaceModelFilename(pSurface->finalSurfaceFilename_);
	if(pSurface->colorizationType_ == R3DProject::CTColoredVertices
		&& pSurface->surfaceType_ == R3DProject::STPoissonRecon)
	{
		if(pProgressDialog_ != NULL)
			pProgressDialog_->Update(80, wxT("Colorizing vertices"));

		if(pR3DSmallTasksThread_ != NULL)
			delete pR3DSmallTasksThread_;

		pR3DSmallTasksThread_ = new R3DSmallTasksThread();
		pR3DSmallTasksThread_->setMainFrame(this);
		pR3DSmallTasksThread_->colorizeSurface(pSurface);

		// ...to be continued in OnSmallTaskFinished
		return;
	}

	if(pProgressDialog_ != NULL)
	{
		pProgressDialog_->EndDialog();
		delete pProgressDialog_;
		pProgressDialog_ = NULL;
	}

	R3DProjectPaths paths;
	project_.getProjectPathsSrf(paths, pSurface);

	// Load generated model
	wxFileName surfaceModelFN(wxString(paths.relativeSurfacePath_.c_str(), wxConvLibc), surfaceModelFilename);
	if(surfaceModelFN.FileExists())
	{
		pSurface->state_ = R3DProject::OSFinished;
		project_.save();
		updateProjectDetails();

		loadSurfaceModel(surfaceModelFN.GetFullPath());
		setProjectTreeItemBold(pSurface->id_);
	}
	else
	{
		wxMessageBox(wxT("No generated model found.\nPlease check console output for errors."),
			wxT("Surface generation"), wxICON_ERROR | wxOK, this);

		project_.removeSurface(pSurface);
		project_.save();
		project_.populateTreeControl(pProjectTreeCtrl_);
	}
}

void Regard3DMainFrame::OnSmallTaskFinished( wxCommandEvent &event )
{
	if(pR3DSmallTasksThread_ == NULL)
		return;

	bool endProgressDialog = true;
	R3DProject::Surface *pSurface = NULL;	// Special case, see below STTColorizeSurface
	R3DProject::Densification *pDensification = NULL;	// Dito, see STTCombineDenseModels
	pR3DSmallTasksThread_->stopThread();
	
	R3DSmallTasksThread::SmallTaskType type = pR3DSmallTasksThread_->getType();
	if(type == R3DSmallTasksThread::STTLoadModel
		|| type == R3DSmallTasksThread::STTLoadSurfaceModel)
	{
		osg::ref_ptr<osg::Node> model = pR3DSmallTasksThread_->getLoadedModel();
		if(model.get() != NULL)
		{
			viewer_->setSceneData(model.get());
			restoreOrientation();
		}
		else
		{
			setProjectTreeItemBold(-1);
			wxMessageBox(wxT("Error: Could not load model"),
				wxT("Load model"), wxICON_ERROR | wxOK, this);
		}

		pOSGGLCanvas_->Refresh();
		endProgressDialog = true;
	}
	else if(type == R3DSmallTasksThread::STTExportToPMVS)
	{
		R3DProject::Densification *pDensification = pR3DSmallTasksThread_->getDensification();
		pDensificationProcess_ = new R3DDensificationProcess(this);
		pDensificationProcess_->runDensificationProcess(pDensification);
		endProgressDialog = false;
	}
	else if(type == R3DSmallTasksThread::STTExportToMeshLab
		|| type == R3DSmallTasksThread::STTExportToCMPMVS
		|| type == R3DSmallTasksThread::STTExportToPointCloud
		|| type == R3DSmallTasksThread::STTExportSurface)
	{
		// Nothing to do in thes cases
		endProgressDialog = true;
	}
	else if(type == R3DSmallTasksThread::STTColorizeSurface)
	{
		// Special case: Since it will call loadSurfaceModel which itself creates a R3DSmallTasksThread,
		// this case is handled in two steps.
		pSurface = pR3DSmallTasksThread_->getSurface();
		endProgressDialog = true;
	}
	else if(type == R3DSmallTasksThread::STTCombineDenseModels)
	{
		// Special case as STTColorizeSurface
		pDensification = pR3DSmallTasksThread_->getDensification();
		endProgressDialog = true;
	}

	delete pR3DSmallTasksThread_;
	pR3DSmallTasksThread_ = NULL;

	if(endProgressDialog)
	{
		if(pProgressDialog_ != NULL)
		{
			pProgressDialog_->EndDialog();

			delete pProgressDialog_;
			pProgressDialog_ = NULL;
		}
		if(pStdProgressDialog_ != NULL)
		{
			pStdProgressDialog_->Hide();

			delete pStdProgressDialog_;
			pStdProgressDialog_ = NULL;
		}
	}

	// From OnTriangulationFinished
	if(!resultStrings_.IsEmpty())
	{
		R3DResultDialog dlg(this, wxT("Triangulation results"),
			wxT("Triangulation was successful!\n\nStatistics:"),
			resultStrings_, htmlReportFilename_);
		dlg.ShowModal();

		resultStrings_.Clear();
		htmlReportFilename_.Clear();
	}

	// From OnSurfaceGenFinished
	if(type == R3DSmallTasksThread::STTColorizeSurface)
	{
		wxString surfaceModelFilename = pSurface->finalSurfaceFilename_;	// Was changed to "model_surface_col.ply"

		R3DProjectPaths paths;
		project_.getProjectPathsSrf(paths, pSurface);

		// Load generated model
		wxFileName surfaceModelFN(wxString(paths.relativeSurfacePath_.c_str(), wxConvLibc), surfaceModelFilename);
		if(surfaceModelFN.FileExists())
		{
			pSurface->state_ = R3DProject::OSFinished;
			project_.save();
			updateProjectDetails();

			loadSurfaceModel(surfaceModelFN.GetFullPath());
			setProjectTreeItemBold(pSurface->id_);
		}
		else
		{
			wxMessageBox(wxT("No generated model found.\nPlease check console output for errors."),
				wxT("Surface generation"), wxICON_ERROR | wxOK, this);

			project_.removeSurface(pSurface);
			project_.save();
			project_.populateTreeControl(pProjectTreeCtrl_);
		}
	}

	// From OnDensificationFinished
	if(type == R3DSmallTasksThread::STTCombineDenseModels)
	{
		// Load generated model
		R3DProjectPaths paths;
		project_.getProjectPathsDns(paths, pDensification);

		wxFileName denseModelFN(wxString(paths.relativeDenseModelName_.c_str(), wxConvLibc));
		if(denseModelFN.FileExists())
		{
			pDensification->state_ = R3DProject::OSFinished;
			project_.save();

			load3DModel(denseModelFN.GetFullPath());
			setProjectTreeItemBold(pDensification->id_);
			updateProjectDetails();
		}
		else
		{
			setProjectTreeItemBold(-1);
			wxMessageBox(wxT("No generated model found.\nPlease check console output for errors."),
				wxT("Densification"), wxICON_ERROR | wxOK, this);

			project_.removeDensification(pDensification);
			project_.save();
			project_.populateTreeControl(pProjectTreeCtrl_);
		}
	}
}

void Regard3DMainFrame::OnContextMenuAddPictureSet( wxCommandEvent &event )
{
	OnProjectAddPictureSetButton(event);
}

void Regard3DMainFrame::OnContextMenuComputeMatches( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::PictureSet *pPictureSet = getSpecializedProjectItem<R3DProject::PictureSet>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypePictureSet);
	if(pPictureSet != NULL)
		addComputeMatches(pPictureSet);
}

void Regard3DMainFrame::OnContextMenuEditPictureSet( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::PictureSet *pPictureSet = getSpecializedProjectItem<R3DProject::PictureSet>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypePictureSet);
	if(pPictureSet != NULL)
		editPictureSet(pPictureSet);
}

void Regard3DMainFrame::OnContextMenuClonePictureSet( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::PictureSet *pPictureSet = getSpecializedProjectItem<R3DProject::PictureSet>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypePictureSet);
	if(pPictureSet != NULL)
		clonePictureSet(pPictureSet);
}

void Regard3DMainFrame::OnContextMenuShowMatches( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::ComputeMatches *pComputeMatches = getSpecializedProjectItem<R3DProject::ComputeMatches>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypeComputeMatches);
	if(pComputeMatches != NULL)
		showMatches(pComputeMatches);
}

void Regard3DMainFrame::OnContextMenuTriangulation( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::ComputeMatches *pComputeMatches = getSpecializedProjectItem<R3DProject::ComputeMatches>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypeComputeMatches);
	if(pComputeMatches != NULL)
		triangulate(pComputeMatches);
}

void Regard3DMainFrame::OnContextMenuCreateDensePointcloud( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::Triangulation *pTriangulation = getSpecializedProjectItem<R3DProject::Triangulation>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypeTriangulation);
	if(pTriangulation != NULL)
		createDensePointcloud(pTriangulation);
}

void Regard3DMainFrame::OnContextMenuShowTriangulatedPoints( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::Triangulation *pTriangulation = getSpecializedProjectItem<R3DProject::Triangulation>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypeTriangulation);
	if(pTriangulation != NULL)
		showTriangulatedPoints(pTriangulation);
}

void Regard3DMainFrame::OnContextMenuDeletePictureSet( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::PictureSet *pPictureSet = getSpecializedProjectItem<R3DProject::PictureSet>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypePictureSet);
	if(pPictureSet != NULL)
		deletePictureSet(pPictureSet);
}

void Regard3DMainFrame::OnContextMenuDeleteComputeMatches( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::ComputeMatches *pComputeMatches = getSpecializedProjectItem<R3DProject::ComputeMatches>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypeComputeMatches);
	if(pComputeMatches != NULL)
		deleteComputeMatches(pComputeMatches);
}

void Regard3DMainFrame::OnContextMenuDeleteTriangulation( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::Triangulation *pTriangulation = getSpecializedProjectItem<R3DProject::Triangulation>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypeTriangulation);
	if(pTriangulation != NULL)
		deleteTriangulation(pTriangulation);
}

void Regard3DMainFrame::OnContextMenuCreateSurface( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::Densification *pDensification = getSpecializedProjectItem<R3DProject::Densification>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypeDensification);
	if(pDensification != NULL)
		createSurface(pDensification);
}

void Regard3DMainFrame::OnContextMenuShowDensePoints( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::Densification *pDensification = getSpecializedProjectItem<R3DProject::Densification>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypeDensification);
	if(pDensification != NULL)
		showDensePoints(pDensification);
}

void Regard3DMainFrame::OnContextMenuDeleteDensification( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::Densification *pDensification = getSpecializedProjectItem<R3DProject::Densification>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypeDensification);
	if(pDensification != NULL)
		deleteDensification(pDensification);
}

void Regard3DMainFrame::OnContextMenuShowSurface( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::Surface *pSurface = getSpecializedProjectItem<R3DProject::Surface>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypeSurface);
	if(pSurface != NULL)
		showSurface(pSurface);
}

void Regard3DMainFrame::OnContextMenuDeleteSurface( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::Surface *pSurface = getSpecializedProjectItem<R3DProject::Surface>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypeSurface);
	if(pSurface != NULL)
		deleteSurface(pSurface);
}

void Regard3DMainFrame::OnContextMenuExportTriangulationToCMPMVS( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::Triangulation *pTriangulation = getSpecializedProjectItem<R3DProject::Triangulation>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypeTriangulation);
	if(pTriangulation != NULL)
		exportTriangulationToCMPMVS(pTriangulation);
}

void Regard3DMainFrame::OnContextMenuExportPointCloud( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::Densification *pDensification = getSpecializedProjectItem<R3DProject::Densification>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypeDensification);
	if(pDensification != NULL)
		exportPointCloud(pDensification);
}

void Regard3DMainFrame::OnContextMenuExportDensificationToMeshLab( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::Densification *pDensification = getSpecializedProjectItem<R3DProject::Densification>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypeDensification);
	if(pDensification != NULL)
		exportDensificationToMeshLab(pDensification);
}

void Regard3DMainFrame::OnContextMenuExportSurface( wxCommandEvent &event )
{
	// Get right-clicked TreeViewCtrl item
	R3DProject::Surface *pSurface = getSpecializedProjectItem<R3DProject::Surface>(contextMenuItemId_,
		R3DProject::R3DTreeItem::TypeSurface);
	if(pSurface != NULL)
		exportSurface(pSurface);
}

void Regard3DMainFrame::OnTimer( wxTimerEvent &WXUNUSED(event) )
{
	if(continuousUpdate_)
	{
		pOSGGLCanvas_->Refresh();
	}

	if(pDensificationProcess_ != NULL)
		pDensificationProcess_->readConsoleOutput();
	if(pR3DSurfaceGenProcess_ != NULL)
		pR3DSurfaceGenProcess_->readConsoleOutput();
}

// Icon: From resource on Win32, from PNG otherwise
#if !defined(R3D_WIN32)
#include "res/png/icons_png.h"
#endif

wxIcon Regard3DMainFrame::getIcon()
{
#if defined(R3D_WIN32)
	wxIcon icon(wxT("aaaaaaaa"));
#else
	wxMemoryInputStream istream(regard3d_icon_png, sizeof(regard3d_icon_png));
	wxImage myimage_img(istream, wxBITMAP_TYPE_PNG);
	wxBitmap myimage_bmp(myimage_img);
	wxIcon icon;
	icon.CopyFromBitmap(myimage_bmp);
#endif

	return icon;
}

void Regard3DMainFrame::updateUIFromProject(bool newProject)
{
	wxString title(wxT(REGARD3D_NAME));

	if(project_.isValidProject())
	{
		title.Append(wxT(" - "));
		title.Append(project_.getProjectName());
	}
	SetTitle(title);

	project_.populateTreeControl(pProjectTreeCtrl_);
	clear3DModel();
	setProjectTreeItemBold(-1);
}

void Regard3DMainFrame::clear3DModel()
{
	saveOrientation();

//	osg::ref_ptr<osg::Geode> geode(new osg::Geode());
//	viewer_->setSceneData(geode.get());
	osg::ref_ptr<osg::Node> model;
	
	if(project_.isValidProject())
		model = pRegard3DModelViewHelper_->createEmptyModel();
	else
		model = pRegard3DModelViewHelper_->createRegard3DTextModel();

	viewer_->setSceneData(model.get());
	pOSGGLCanvas_->Refresh();
}

void Regard3DMainFrame::load3DModel(const wxString &filename)
{
	saveOrientation();

	clear3DModel();

	if(pR3DSmallTasksThread_ != NULL)
		delete pR3DSmallTasksThread_;

	pStdProgressDialog_ = new wxProgressDialog(wxT("Loading model"), wxT("Loading model"), -1, this, wxPD_APP_MODAL);
	pStdProgressDialog_->Show();
	pStdProgressDialog_->Pulse();	// Run in indeterminate mode

	pR3DSmallTasksThread_ = new R3DSmallTasksThread();
	pR3DSmallTasksThread_->setMainFrame(this);
	pR3DSmallTasksThread_->loadModel(filename, pRegard3DModelViewHelper_);
}

void Regard3DMainFrame::loadSurfaceModel(const wxString &filename)
{
	saveOrientation();

	clear3DModel();

	if(pR3DSmallTasksThread_ != NULL)
		delete pR3DSmallTasksThread_;

	pStdProgressDialog_ = new wxProgressDialog(wxT("Loading surface model"), wxT("Loading surface model"), -1, this, wxPD_APP_MODAL);
	pStdProgressDialog_->Show();
	pStdProgressDialog_->Pulse();	// Run in indeterminate mode

	pR3DSmallTasksThread_ = new R3DSmallTasksThread();
	pR3DSmallTasksThread_->setMainFrame(this);
	pR3DSmallTasksThread_->loadSurfaceModel(filename, pRegard3DModelViewHelper_);
}

bool Regard3DMainFrame::getProjectTreeItemDetails(const wxTreeItemId &treeId, R3DProject::R3DTreeItem::R3DTreeItemType &type, int &id)
{
	if(!treeId.IsOk())
		return false;
	type = R3DProject::R3DTreeItem::TypeInvalid;
	id = 0;
	wxTreeItemData *pData = pProjectTreeCtrl_->GetItemData(treeId);
	if(pData != NULL)
	{
		R3DProject::R3DTreeItem *pTreeItem = dynamic_cast<R3DProject::R3DTreeItem*>(pData);
		if(pTreeItem != NULL)
		{
			type = pTreeItem->getType();
			id = pTreeItem->getID();

			return true;
		}
	}

	return false;
}

void Regard3DMainFrame::selectProjectTreeItem(int id)
{
	wxTreeItemId rootId = pProjectTreeCtrl_->GetRootItem();
	if(rootId.IsOk())
	{
		wxTreeItemId selectItemId = findRecursiveProjectTreeItem(rootId, id);
		if(selectItemId.IsOk())
		{
			pProjectTreeCtrl_->SelectItem(selectItemId);
		}
	}
}

wxTreeItemId Regard3DMainFrame::findRecursiveProjectTreeItem(const wxTreeItemId &startId, int id)
{
	wxTreeItemId treeItemId;
	if(startId.IsOk())
	{
		R3DProject::R3DTreeItem::R3DTreeItemType type = R3DProject::R3DTreeItem::TypeInvalid;
		int oid = 0;
		getProjectTreeItemDetails(startId, type, oid);
		if(oid == id)
			return startId;

		wxTreeItemIdValue cookie;
		wxTreeItemId cur = pProjectTreeCtrl_->GetFirstChild(startId, cookie);
		while(cur.IsOk())
		{
			// Recursive call
			treeItemId = findRecursiveProjectTreeItem(cur, id);
			if(treeItemId.IsOk())
				return treeItemId;

			cur = pProjectTreeCtrl_->GetNextChild(startId, cookie);
		}
	}

	return treeItemId;
}

void Regard3DMainFrame::setProjectTreeItemBold(int id)
{
	wxTreeItemId rootId = pProjectTreeCtrl_->GetRootItem();
	if(rootId.IsOk())
	{
		int oldBoldId = project_.getShownItemId();
		wxTreeItemId oldBoldItemId = findRecursiveProjectTreeItem(rootId, oldBoldId);
		if(oldBoldItemId.IsOk())
			pProjectTreeCtrl_->SetItemBold(oldBoldItemId, false);
		project_.setShownItemId(id);
		wxTreeItemId itemId = findRecursiveProjectTreeItem(rootId, id);
		if(itemId.IsOk())
			pProjectTreeCtrl_->SetItemBold(itemId, true);
	}
}

void Regard3DMainFrame::updateProjectDetails()
{
	// Get selected item
	wxTreeItemId selId = pProjectTreeCtrl_->GetSelection();
	R3DProject::R3DTreeItem::R3DTreeItemType type = R3DProject::R3DTreeItem::TypeInvalid;
	int id = 0;

	// Also handle empty project tree gracefully
	if(selId.IsOk())
		getProjectTreeItemDetails(selId, type, id);

	R3DProject::Object *pObject = project_.getObjectByTypeAndID(type, id);
	if(type == R3DProject::R3DTreeItem::TypeProject)
	{
		pProjectLowerSizer_->Show(pProjectProjectSizer_, true);
		pProjectNameTextCtrl_->SetValue(project_.getProjectName());
		pProjectPathTextCtrl_->SetValue(project_.getProjectPath());
	}
	else
		pProjectLowerSizer_->Hide(pProjectProjectSizer_, true);

	if(type == R3DProject::R3DTreeItem::TypePictureSet)
	{
		pProjectLowerSizer_->Show(pProjectPictureSetSizer_, true);
		bool showOnly = false;
		wxString name;
		int numberOfPictures = 0;
		if(pObject != NULL)
		{
			R3DProject::PictureSet *pPictureSet = dynamic_cast<R3DProject::PictureSet *>(pObject);
			if(pPictureSet != NULL)
			{
				name = pPictureSet->name_;
				numberOfPictures = static_cast<int>(pPictureSet->getImageInfoVector().size());
				showOnly = !(pPictureSet->computeMatches_.empty());
			}
		}
		pPictureSetNameTextCtrl_->SetValue(name);
		pPictureSetIdTextCtrl_->SetValue(wxString::Format(wxT("%d"), pObject->runningId_));
		pPictureSetSizeTextCtrl_->SetValue(wxString::Format(wxT("%d"), numberOfPictures));
		if(showOnly)
			pEditPictureSetButton_->SetLabel(wxT("Show picture set..."));
		else
			pEditPictureSetButton_->SetLabel(wxT("Edit picture set..."));
	}
	else
		pProjectLowerSizer_->Hide(pProjectPictureSetSizer_, true);

	if(type == R3DProject::R3DTreeItem::TypeComputeMatches)
	{
		pProjectLowerSizer_->Show(pProjectComputeMatchesSizer_, true);
		wxString name, params, results, runningTime;
		if(pObject != NULL)
		{
			R3DProject::ComputeMatches *pComputeMatches = dynamic_cast<R3DProject::ComputeMatches *>(pObject);
			if(pComputeMatches != NULL)
			{
				name = pComputeMatches->name_;
				params = wxString::Format(wxT("Threshold: %3g/Dist ratio: %3g"), pComputeMatches->threshold_, pComputeMatches->distRatio_);
				if(!pComputeMatches->numberOfKeypoints_.empty())
				{
					using namespace boost::accumulators;
					accumulator_set<int, stats<tag::median, tag::min, tag::max, tag::mean> > acc;
					for(size_t i = 0; i < pComputeMatches->numberOfKeypoints_.size(); i++)
					{
						acc(pComputeMatches->numberOfKeypoints_[i]);
					}

					results = wxString::Format(wxT("%d/%d/%d/%d"), 
						(min)(acc), (max)(acc),
						static_cast<int>(mean(acc)),
						static_cast<int>(median(acc)));
				}
				runningTime = pComputeMatches->runningTime_;
			}
		}
		pComputeMatchesIdTextCtrl_->SetValue(wxString::Format(wxT("%d"), pObject->runningId_));
		pComputeMatchesParametersTextCtrl_->SetValue(params);
		pComputeMatchesParametersTextCtrl_->SetToolTip(params);
		pComputeMatchesResultsTextCtrl_->SetValue(results);
		pComputeMatchesResultsTextCtrl_->SetToolTip(results);
		pComputeMatchesTimeTextCtrl_->SetValue(runningTime);
	}
	else
		pProjectLowerSizer_->Hide(pProjectComputeMatchesSizer_, true);

	if(type == R3DProject::R3DTreeItem::TypeTriangulation)
	{
		pProjectLowerSizer_->Show(pProjectTriangulationSizer_, true);
		wxString name, params, resultsCameras, resultNumberOfTracks, resultResidualErrors, runningTime;
		if(pObject != NULL)
		{
			R3DProject::Triangulation *pTriangulation = dynamic_cast<R3DProject::Triangulation *>(pObject);
			if(pTriangulation != NULL)
			{
				name = pTriangulation->name_;
				if(pTriangulation->global_)
				{
					params = wxString(wxT("Global ("));
					if(pTriangulation->globalMSTBasedRot_)
						params.Append(wxT("MST based rotation + L1 rotation averaging)"));
					else
						params.Append(wxT("Dense L2 global rotation computation)"));
				}
				else
				{
					params = wxString::Format(wxT("Incremental, initial pair %d/%d"), pTriangulation->initialImageIndexA_,
						pTriangulation->initialImageIndexB_);
				}
				resultsCameras = pTriangulation->resultCameras_;
				resultNumberOfTracks = pTriangulation->resultNumberOfTracks_;
				resultResidualErrors = pTriangulation->resultResidualErrors_;
				runningTime = pTriangulation->runningTime_;
			}
		}
		pTriangulationIdTextCtrl_->SetValue(wxString::Format(wxT("%d"), pObject->runningId_));
		pTriangulationParametersTextCtrl_->SetValue(params);
		pTriangulationParametersTextCtrl_->SetToolTip(params);
		pTriangulationCamerasTextCtrl_->SetValue(resultsCameras);
		pTriangulationNumberPointsTextCtrl_->SetValue(resultNumberOfTracks);
		pTriangulationResidualErrorsTextCtrl_->SetValue(resultResidualErrors);
		pTriangulationResidualErrorsTextCtrl_->SetToolTip(resultResidualErrors);
		pTriangulationTimeTextCtrl_->SetValue(runningTime);
	}
	else
		pProjectLowerSizer_->Hide(pProjectTriangulationSizer_, true);

	if(type == R3DProject::R3DTreeItem::TypeDensification)
	{
		pProjectLowerSizer_->Show(pProjectDensificationSizer_, true);
		wxString name, type, params, runningTime;
		if(pObject != NULL)
		{
			R3DProject::Densification *pDensification = dynamic_cast<R3DProject::Densification *>(pObject);
			if(pDensification != NULL)
			{
				name = pDensification->name_;
				if(pDensification->densificationType_ == R3DProject::DTCMVSPMVS)
					type = wxString(wxT("CMVS/PMVS"));
				else if(pDensification->densificationType_ == R3DProject::DTMVE)
					type = wxString(wxT("MVE (Multi view environment)"));
				else if(pDensification->densificationType_ == R3DProject::DTCMPMVS)
					type = wxString(wxT("CMPMVS"));
				else
					type = wxString(wxT("Unknown"));

				if(pDensification->densificationType_ == R3DProject::DTCMVSPMVS)
				{
					params.Printf(wxT("UseVis: %s Level: %d Cell size: %d Threshold: %g wsize: %d Min image num: %d Max cluster size: %d"), 
						(pDensification->useCMVS_ ? wxT("yes") : wxT("no")),
						pDensification->pmvsLevel_, pDensification->pmvsCSize_,
						pDensification->pmvsThreshold_, pDensification->pmvsWSize_,
						pDensification->pmvsMinImageNum_, pDensification->pmvsMaxClusterSize_);
				}
				else if(pDensification->densificationType_ == R3DProject::DTMVE)
				{
					params.Printf(wxT("Scale: %d, Filter width: %d"),
						pDensification->mveScale_, pDensification->mveFilterWidth_);
				}
				runningTime = pDensification->runningTime_;
			}
		}
		pDensificationIdTextCtrl_->SetValue(wxString::Format(wxT("%d"), pObject->runningId_));
		pDensificationTypeTextCtrl_->SetValue(type);
		pDensificationParametersTextCtrl_->SetValue(params);
		pDensificationParametersTextCtrl_->SetToolTip(params);
		pDensificationRunningTimeTextCtrl_->SetValue(runningTime);
	}
	else
		pProjectLowerSizer_->Hide(pProjectDensificationSizer_, true);

	if(type == R3DProject::R3DTreeItem::TypeSurface)
	{
		pProjectLowerSizer_->Show(pProjectSurfaceSizer_, true);
		wxString name, type, colorType, params, cparams, runningTime;
		if(pObject != NULL)
		{
			R3DProject::Surface *pSurface = dynamic_cast<R3DProject::Surface *>(pObject);
			if(pSurface != NULL)
			{
				name = pSurface->name_;
				if(pSurface->surfaceType_ == R3DProject::STPoissonRecon)
				{
					type = wxString(wxT("Poisson reconstruction"));
					params.Printf(wxT("Depth: %d Samples per Node: %g Point weight: %g Trim threshold: %g"),
						pSurface->poissonDepth_, pSurface->poissonSamplesPerNode_,
						pSurface->poissonPointWeight_, pSurface->poissonTrimThreshold_);
				}
				else if(pSurface->surfaceType_ == R3DProject::STFSSRecon)
				{
					type = wxString(wxT("Floating scale surface reconstruction"));
					params.Printf(wxT("Levels: %d Scale factor mult: %g Confidence threshold: %g Min component size: %d"),
						pSurface->fssrRefineOctreeLevels_, pSurface->fssrScaleFactorMultiplier_,
						pSurface->fssrConfidenceThreshold_, pSurface->fssrMinComponentSize_);
				}
				else
					type = wxString(wxT("Unknown"));

				if(pSurface->colorizationType_ == R3DProject::CTColoredVertices)
				{
					colorType = wxT("Colored vertices");
					if(pSurface->surfaceType_ != R3DProject::STFSSRecon)
						cparams.Printf(wxT("Number of neighbours: %d"), pSurface->colVertNumNeighbours_);
				}
				else if(pSurface->colorizationType_ == R3DProject::CTTextures)
				{
					colorType = wxT("Textures");
					wxString outlierRemovalTypeStr(wxT("None"));
					if(pSurface->textOutlierRemovalType_ == 1)
						outlierRemovalTypeStr = wxT("Gauss clamping");
					else if(pSurface->textOutlierRemovalType_ == 2)
						outlierRemovalTypeStr = wxT("Gauss damping");
					cparams.Printf(wxT("GeomVisTest: %s Global seam level: %s Local seam level: %s Outlier removal: "),
						(pSurface->textGeometricVisibilityTest_ ? wxT("yes") : wxT("no")),
						(pSurface->textGlobalSeamLeveling_ ? wxT("yes") : wxT("no")),
						(pSurface->textLocalSeamLeveling_ ? wxT("yes") : wxT("no")));
					cparams.Append(outlierRemovalTypeStr);
				}
				else
					colorType = wxT("Unknown");

				runningTime = pSurface->runningTime_;
			}
		}
		pSurfaceIdTextCtrl_->SetValue(wxString::Format(wxT("%d"), pObject->runningId_));
		pSurfaceTypeTextCtrl_->SetValue(type);
		pSurfaceParametersTextCtrl_->SetValue(params);
		pSurfaceParametersTextCtrl_->SetToolTip(params);
		pSurfaceColorizationTypeTextCtrl_->SetValue(colorType);
		pSurfaceColorizationParametersTextCtrl_->SetValue(cparams);
		pSurfaceColorizationParametersTextCtrl_->SetToolTip(cparams);
		pSurfaceRunningTimeTextCtrl_->SetValue(runningTime);
	}
	else
		pProjectLowerSizer_->Hide(pProjectSurfaceSizer_, true);

	pProjectLowerSizer_->Layout();
	pProjectLowerScrolledWindow_->FitInside();
}

void Regard3DMainFrame::editPictureSet(R3DProject::PictureSet *pPictureSet)
{
	// Picture sets with one or more compute matches can't be edited
	bool showOnly = !(pPictureSet->computeMatches_.empty());

	Regard3DPictureSetDialog dlg(this);
	dlg.setPreviewGeneratorThread(&previewGeneratorThread_);
	dlg.setImageInfoThread(&imageInfoThread_);
	dlg.setPictureSet(pPictureSet);
	pImageUpdatesDlg_ = &dlg;
	int retVal = wxID_CANCEL;

	try
	{
		retVal = dlg.ShowModal();
	}
	catch(...)
	{
		pImageUpdatesDlg_ = NULL;
		throw;
	}
	pImageUpdatesDlg_ = NULL;

	if(!showOnly && (retVal == wxID_OK))
	{
		dlg.updateProject(&project_);
		updateProjectDetails();
		project_.save();
	}
}

void Regard3DMainFrame::clonePictureSet(R3DProject::PictureSet *pPictureSet)
{
	int newID = project_.clonePictureSet(pPictureSet);
	project_.populateTreeControl(pProjectTreeCtrl_);
	selectProjectTreeItem(newID);
	updateProjectDetails();
	project_.save();
}

void Regard3DMainFrame::deletePictureSet(R3DProject::PictureSet *pPictureSet)
{
	if(wxMessageBox(wxT("Are you sure to delete the picture set?\n")
		wxT("All derived objects will also be deleted!"),
		wxT("Delete picture set"), wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION, this) == wxYES)
	{
		clear3DModel();
		project_.setShownItemId(-1);

		project_.removePictureSet(pPictureSet);
		project_.populateTreeControl(pProjectTreeCtrl_);
		project_.save();
		updateProjectDetails();
	}
}

void Regard3DMainFrame::addComputeMatches(R3DProject::PictureSet *pPictureSet)
{
	// Determine biggest picture
	long long maxPixelCount = 0;
	const ImageInfoVector &iiv = pPictureSet->imageList_;
	for(size_t i = 0; i < iiv.size(); i++)
	{
		const ImageInfo &iv = iiv[i];
		long long pixelCount = static_cast<long long>(iv.imageWidth_) * static_cast<long long>(iv.imageHeight_);
		maxPixelCount = std::max(maxPixelCount, pixelCount);
	}

	Regard3DComputeMatchesDialog dlg(this);
	dlg.setMaxPixelCount(maxPixelCount);
	if(dlg.ShowModal() == wxID_OK)
	{
		// Start compute matches
		float keypointSensitivity, keypointMatchingRatio;
		int nrOfThreads;

		dlg.getResults(keypointSensitivity, keypointMatchingRatio,
			nrOfThreads);

		Regard3DFeatures::R3DFParams params;
		params.threshold_ = keypointSensitivity;
		params.distRatio_ = keypointMatchingRatio;
		params.numberOfThreads_ = nrOfThreads;
		bool svgOutput = false;	// TODO: Make configurable?

		wxString featureDetector(wxT("AKAZE")), descriptorExtractor(wxT("LIOP"));
		int newID = project_.addComputeMatches(pPictureSet, featureDetector, descriptorExtractor,
			keypointSensitivity, keypointMatchingRatio);
		if(newID >= 0)
		{
			project_.populateTreeControl(pProjectTreeCtrl_);
			project_.save();
			selectProjectTreeItem(newID);
		}

		R3DProject::Object *pObject = project_.getObjectByTypeAndID(R3DProject::R3DTreeItem::TypeComputeMatches, newID);
		R3DProject::ComputeMatches *pComputeMatches = NULL;
		if(pObject != NULL)
		{
			pComputeMatches = dynamic_cast<R3DProject::ComputeMatches *>(pObject);
		}
		if(pComputeMatches == NULL)
			return;
		pComputeMatches->state_ = R3DProject::OSRunning;

		if(pR3DComputeMatchesThread_ != NULL)
			delete pR3DComputeMatchesThread_;
		pR3DComputeMatchesThread_ = new R3DComputeMatchesThread();
		pR3DComputeMatchesThread_->setMainFrame(this);
		pR3DComputeMatchesThread_->setParameters(params, svgOutput);
		pR3DComputeMatchesThread_->setComputeMatches(pComputeMatches, pPictureSet);
		pR3DComputeMatchesThread_->startComputeMatchesThread();

		pProgressDialog_ = new Regard3DProgressDialog(this, wxT("Computing matches"));
		pProgressDialog_->Show();
	}
}

void Regard3DMainFrame::showMatches(R3DProject::ComputeMatches *pComputeMatches)
{
	Regard3DMatchingResultsDialog dlg(this);
	dlg.setPreviewGeneratorThread(&previewGeneratorThread_);
	dlg.setComputeMatches(&project_, pComputeMatches);
	pImageUpdatesDlg_ = &dlg;
	int retVal = wxID_CANCEL;

	try
	{
		retVal = dlg.ShowModal();
	}
	catch(...)
	{
		pImageUpdatesDlg_ = NULL;
		throw;
	}
	pImageUpdatesDlg_ = NULL;
}

void Regard3DMainFrame::triangulate(R3DProject::ComputeMatches *pComputeMatches)
{
	Regard3DTriangulationDialog dlg(this);
	dlg.setPreviewGeneratorThread(&previewGeneratorThread_);
	dlg.setComputeMatches(&project_, pComputeMatches);
	pImageUpdatesDlg_ = &dlg;

	int retVal = wxID_CANCEL;
	try
	{
		retVal = dlg.ShowModal();
	}
	catch(...)
	{
		pImageUpdatesDlg_ = NULL;
		throw;
	}
	pImageUpdatesDlg_ = NULL;

	if(retVal == wxID_OK)
	{
		bool useGlobalAlgorithm = false, globalMSTBasedRot = true;
		size_t initialPairA = 0, initialPairB = 0;
		dlg.getResults(useGlobalAlgorithm, initialPairA, initialPairB, globalMSTBasedRot);

		if(pR3DTriangulationThread_ != NULL)
			delete pR3DTriangulationThread_;

		pProgressDialog_ = new Regard3DProgressDialog(this, wxT("Triangulation..."));
		pProgressDialog_->Show();

		R3DProject::Triangulation *pTriangulation = NULL;
		int newID = project_.addTriangulation(pComputeMatches, initialPairA, initialPairB,
			useGlobalAlgorithm, globalMSTBasedRot);
		if(newID >= 0)
		{
			project_.populateTreeControl(pProjectTreeCtrl_);
			project_.save();
			selectProjectTreeItem(newID);
		}

		R3DProject::Object *pObject = project_.getObjectByTypeAndID(R3DProject::R3DTreeItem::TypeTriangulation, newID);
		if(pObject != NULL)
		{
			pTriangulation = dynamic_cast<R3DProject::Triangulation *>(pObject);
		}
		if(pTriangulation == NULL)
		{
			wxASSERT(false);
		}
		pTriangulation->state_ = R3DProject::OSRunning;
		project_.prepareTriangulation(pTriangulation);
		clear3DModel();
		setProjectTreeItemBold(-1);

		pR3DTriangulationThread_ = new R3DTriangulationThread();
		pR3DTriangulationThread_->setMainFrame(this);
		pR3DTriangulationThread_->setParameters(useGlobalAlgorithm, initialPairA, initialPairB, globalMSTBasedRot);
		pR3DTriangulationThread_->setTriangulation(&project_, pTriangulation);
		pR3DTriangulationThread_->startTriangulationThread();
	}
}

void Regard3DMainFrame::deleteComputeMatches(R3DProject::ComputeMatches *pComputeMatches)
{
	if(wxMessageBox(wxT("Are you sure to delete the compute matches result?\n")
		wxT("All derived objects will also be deleted!"),
		wxT("Delete compute matches"), wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION, this) == wxYES)
	{
		clear3DModel();
		project_.setShownItemId(-1);

		project_.removeComputeMatches(pComputeMatches);
		project_.populateTreeControl(pProjectTreeCtrl_);
		project_.save();
		updateProjectDetails();
	}
}

void Regard3DMainFrame::createDensePointcloud(R3DProject::Triangulation *pTriangulation)
{
	Regard3DDensificationDialog dlg(this);
	if(dlg.ShowModal() == wxID_OK)
	{
		int newID = project_.addDensification(pTriangulation);
		if(newID >= 0)
		{
			project_.populateTreeControl(pProjectTreeCtrl_);
			project_.save();
			selectProjectTreeItem(newID);
		}

		R3DProject::Densification *pDensification = NULL;
		R3DProject::Object *pObject = project_.getObjectByTypeAndID(R3DProject::R3DTreeItem::TypeDensification, newID);
		if(pObject != NULL)
		{
			pDensification = dynamic_cast<R3DProject::Densification *>(pObject);
		}
		if(pDensification == NULL)
		{
			wxASSERT(false);
		}

		dlg.getResults(pDensification);

		pDensification->state_ = R3DProject::OSRunning;
		clear3DModel();
		setProjectTreeItemBold(-1);

		pProgressDialog_ = new Regard3DProgressDialog(this, wxT("Densification..."));
		pProgressDialog_->Show();

		project_.prepareDensification(pDensification);

		if(pDensification->densificationType_ == R3DProject::DTCMVSPMVS)
		{
			pProgressDialog_->Pulse(wxT("Exporting project to PMVS"));

			if(pR3DSmallTasksThread_ != NULL)
				delete pR3DSmallTasksThread_;

			pR3DSmallTasksThread_ = new R3DSmallTasksThread();
			pR3DSmallTasksThread_->setMainFrame(this);
			pR3DSmallTasksThread_->exportToPMVS(pDensification);

			// The actual densification process will be started in OnSmallTaskFinished
		}
		else
		{
			pDensificationProcess_ = new R3DDensificationProcess(this);
			pDensificationProcess_->runDensificationProcess(pDensification);
		}
	}
}

void Regard3DMainFrame::showTriangulatedPoints(R3DProject::Triangulation *pTriangulation)
{
	R3DProjectPaths paths;
	project_.getProjectPathsTri(paths, pTriangulation);

	// Load model if it exists
	wxFileName modelName(wxString(paths.relativeOutPath_.c_str(), *wxConvCurrent), wxT("FinalColorized.ply"));
//	wxFileName modelName(project_.getAbsoluteOutPath(), wxT("FinalColorized.ply"));
	if(modelName.FileExists())
	{
		load3DModel(modelName.GetFullPath());
		setProjectTreeItemBold(pTriangulation->id_);
	}
	else
	{
		setProjectTreeItemBold(-1);
		wxMessageBox(wxT("Error: Result file not found"),
			wxT("Show triangulated points"), wxICON_ERROR | wxOK, this);
	}
}

void Regard3DMainFrame::deleteTriangulation(R3DProject::Triangulation *pTriangulation)
{
	if(wxMessageBox(wxT("Are you sure to delete the triangulation?\n")
		wxT("All derived objects will also be deleted!"),
		wxT("Delete triangulation"), wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION, this) == wxYES)
	{
		clear3DModel();
		project_.setShownItemId(-1);

		project_.removeTriangulation(pTriangulation);
		project_.populateTreeControl(pProjectTreeCtrl_);
		project_.save();
		updateProjectDetails();
	}
}

void Regard3DMainFrame::createSurface(R3DProject::Densification *pDensification)
{
	Regard3DSurfaceDialog dlg(this);
	dlg.setParams(pDensification);
	if(dlg.ShowModal() == wxID_OK)
	{
		int newID = project_.addSurface(pDensification);
		if(newID >= 0)
		{
			project_.populateTreeControl(pProjectTreeCtrl_);
			project_.save();
			selectProjectTreeItem(newID);
		}

		R3DProject::Surface *pSurface = NULL;
		R3DProject::Object *pObject = project_.getObjectByTypeAndID(R3DProject::R3DTreeItem::TypeSurface, newID);
		if(pObject != NULL)
		{
			pSurface = dynamic_cast<R3DProject::Surface *>(pObject);
		}
		if(pSurface == NULL)
		{
			wxASSERT(false);
		}

		dlg.getResults(pSurface);

		pSurface->state_ = R3DProject::OSRunning;
		clear3DModel();
		setProjectTreeItemBold(-1);

		pProgressDialog_ = new Regard3DProgressDialog(this, wxT("Surface generation..."));
		pProgressDialog_->Show();
		wxThread::Yield();

		project_.prepareSurface(pSurface);

		pR3DSurfaceGenProcess_ = new R3DSurfaceGenProcess(this);
		pR3DSurfaceGenProcess_->runSurfaceGenProcess(pSurface);
	}

}

void Regard3DMainFrame::showDensePoints(R3DProject::Densification *pDensification)
{
	R3DProjectPaths paths;
	project_.getProjectPathsDns(paths, pDensification);

	// Load generated model
	wxFileName denseModelFN(wxString(paths.relativeDenseModelName_.c_str(), wxConvLibc));
	if(denseModelFN.FileExists())
	{
		load3DModel(denseModelFN.GetFullPath());
		setProjectTreeItemBold(pDensification->id_);
	}
	else
	{
		setProjectTreeItemBold(-1);
		wxMessageBox(wxT("Error: Result file not found"),
			wxT("Show dense points"), wxICON_ERROR | wxOK, this);
	}
}

void Regard3DMainFrame::deleteDensification(R3DProject::Densification *pDensification)
{
	if(wxMessageBox(wxT("Are you sure to delete the densification?\n")
		wxT("All derived objects will also be deleted!"),
		wxT("Delete densification"), wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION, this) == wxYES)
	{
		clear3DModel();
		project_.setShownItemId(-1);

		project_.removeDensification(pDensification);
		project_.populateTreeControl(pProjectTreeCtrl_);
		project_.save();
		updateProjectDetails();
	}
}

void Regard3DMainFrame::showSurface(R3DProject::Surface *pSurface)
{
	R3DProjectPaths paths;
	project_.getProjectPathsSrf(paths, pSurface);

	wxFileName surfaceModelFN(wxString(paths.relativeSurfacePath_.c_str(), wxConvLibc), pSurface->finalSurfaceFilename_);
	if(surfaceModelFN.FileExists())
	{
		loadSurfaceModel(surfaceModelFN.GetFullPath());

		setProjectTreeItemBold(pSurface->id_);
	}
}

void Regard3DMainFrame::deleteSurface(R3DProject::Surface *pSurface)
{
	if(wxMessageBox(wxT("Are you sure to delete the surface?"),
		wxT("Delete surface"), wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION, this) == wxYES)
	{
		clear3DModel();
		project_.setShownItemId(-1);

		project_.removeSurface(pSurface);
		project_.populateTreeControl(pProjectTreeCtrl_);
		project_.save();
		updateProjectDetails();
	}
}

void Regard3DMainFrame::exportTriangulationToCMPMVS(R3DProject::Triangulation *pTriangulation)
{
	wxDirDialog dlg(this, wxT("Choose directory to export project to:"), wxEmptyString,
		wxDD_DEFAULT_STYLE);
	if(dlg.ShowModal() == wxID_OK)
	{
		pStdProgressDialog_ = new wxProgressDialog(wxT("Exporting to CMPMVS"), wxT("Exporting project to CMPMVS"), -1, this, wxPD_APP_MODAL);
		pStdProgressDialog_->Show();
		pStdProgressDialog_->Pulse();	// Run in indeterminate mode

		if(pR3DSmallTasksThread_ != NULL)
			delete pR3DSmallTasksThread_;

		pR3DSmallTasksThread_ = new R3DSmallTasksThread();
		pR3DSmallTasksThread_->setMainFrame(this);
		pR3DSmallTasksThread_->exportTriangulationToCMPMVS(pTriangulation, dlg.GetPath());
	}
}

void Regard3DMainFrame::exportPointCloud(R3DProject::Densification *pDensification)
{
	wxFileDialog dlg(this, wxT("Select directory and filename for exported point cloud"),
		wxEmptyString, wxT("pointcloud.ply"),
		wxT("Stanford Polygon File Format (*.ply)|*.ply|Point Cloud Data (*.pcd)|*.pcd"),
		wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
	if(dlg.ShowModal() == wxID_OK)
	{
		wxString filename = dlg.GetPath();

		pStdProgressDialog_ = new wxProgressDialog(wxT("Exporting point cloud"), wxT("Exporting point cloud"), -1, this, wxPD_APP_MODAL);
		pStdProgressDialog_->Show();
		pStdProgressDialog_->Pulse();	// Run in indeterminate mode

		if(pR3DSmallTasksThread_ != NULL)
			delete pR3DSmallTasksThread_;

		pR3DSmallTasksThread_ = new R3DSmallTasksThread();
		pR3DSmallTasksThread_->setMainFrame(this);
		pR3DSmallTasksThread_->exportDensificationToPointCloud(pDensification, filename);
	}
}

void Regard3DMainFrame::exportDensificationToMeshLab(R3DProject::Densification *pDensification)
{
	wxDirDialog dlg(this, wxT("Choose directory to export scene to:"), wxEmptyString,
		wxDD_DEFAULT_STYLE);
	if(dlg.ShowModal() == wxID_OK)
	{
		pStdProgressDialog_ = new wxProgressDialog(wxT("Exporting to MeshLab"), wxT("Exporting scene to MeshLab"), -1, this, wxPD_APP_MODAL);
		pStdProgressDialog_->Show();
		pStdProgressDialog_->Pulse();	// Run in indeterminate mode

		if(pR3DSmallTasksThread_ != NULL)
			delete pR3DSmallTasksThread_;

		pR3DSmallTasksThread_ = new R3DSmallTasksThread();
		pR3DSmallTasksThread_->setMainFrame(this);
		pR3DSmallTasksThread_->exportDensificationToMeshLab(pDensification, dlg.GetPath());
	}
}

void Regard3DMainFrame::exportSurface(R3DProject::Surface *pSurface)
{
	wxString fileFormatPLY(wxT("Stanford Polygon File Format (*.ply)|*.ply"));
	wxString fileFormatOBJ(wxT("Alias Wavefront object (*.obj)|*.obj"));

	wxString fileFormats, defaultFilename;
	if(pSurface->colorizationType_ == R3DProject::CTColoredVertices)
	{
		fileFormats = fileFormatPLY + wxString(wxT("|")) + fileFormatOBJ;
		defaultFilename = wxT("surface.ply");
	}
	else
	{
		fileFormats = fileFormatOBJ;
		defaultFilename = wxT("surface.obj");
	}
	wxFileDialog dlg(this, wxT("Select directory and filename for exported surface file"),
		wxEmptyString, defaultFilename, fileFormats, wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
	if(dlg.ShowModal() == wxID_OK)
	{
		wxString filename = dlg.GetPath();

		pStdProgressDialog_ = new wxProgressDialog(wxT("Exporting surface"), wxT("Exporting surface"), -1, this, wxPD_APP_MODAL);
		pStdProgressDialog_->Show();
		pStdProgressDialog_->Pulse();	// Run in indeterminate mode

		if(pR3DSmallTasksThread_ != NULL)
			delete pR3DSmallTasksThread_;

		pR3DSmallTasksThread_ = new R3DSmallTasksThread();
		pR3DSmallTasksThread_->setMainFrame(this);
		pR3DSmallTasksThread_->exportSurface(pSurface, filename);
	}
}


/**
 * Saves the current orientation to the selected object.
 */
void Regard3DMainFrame::saveOrientation()
{
	bool saveProject = false;

	wxTreeItemId rootId = pProjectTreeCtrl_->GetRootItem();
	if(rootId.IsOk())
	{
		int previousShownItemId = project_.getShownItemId();
		wxTreeItemId previousShownItem = findRecursiveProjectTreeItem(rootId, previousShownItemId);
		if(previousShownItem.IsOk())
		{
			R3DProject::R3DTreeItem::R3DTreeItemType type;
			int id;
			if(getProjectTreeItemDetails(previousShownItem, type, id))
			{
				// Store manipulator settings to object
				R3DProject::Object *pObject = project_.getObjectByTypeAndID(type, id);
				if(pObject != NULL)
				{

					osgGA::CameraManipulator *pManip = viewer_->getCameraManipulator();
					osgGA::TrackballManipulator *pTrackballManip = dynamic_cast<osgGA::TrackballManipulator *>(pManip);
					if(pTrackballManip != NULL)
					{
						osg::Vec3d center = pTrackballManip->getCenter();
						double scale = pTrackballManip->getDistance();

						osg::Vec3d eye;
						osg::Quat rot;
						pTrackballManip->getTransformation(eye, rot);

						std::wostringstream ostr;
						ostr << eye.x() << L" " << eye.y() << L" " << eye.z()
							<< L" " << rot.x() << L" " << rot.y() << L" " << rot.z() << L" " << rot.w()
							<< L" " << center.x() << L" " << center.y() << L" " << center.z() << L" " << scale;
						wxString orientation( ostr.str().c_str() );
						pObject->orientation_ = orientation;

						saveProject = true;
					}
				}
			}
		}
	}

	// Save project if successful
	if(saveProject)
		project_.save();
}

/**
 * Restores the orientation stored in the selected object.
 */
void Regard3DMainFrame::restoreOrientation()
{
	wxTreeItemId rootId = pProjectTreeCtrl_->GetRootItem();
	if(rootId.IsOk())
	{
		int previousShownItemId = project_.getShownItemId();
		wxTreeItemId previousShownItem = findRecursiveProjectTreeItem(rootId, previousShownItemId);
		if(previousShownItem.IsOk())
		{
			R3DProject::R3DTreeItem::R3DTreeItemType type;
			int id;
			if(getProjectTreeItemDetails(previousShownItem, type, id))
			{
				// Store manipulator settings to object
				R3DProject::Object *pObject = project_.getObjectByTypeAndID(type, id);

				if(pObject != NULL)
				{
					wxString orientation = pObject->orientation_;
					if(!orientation.IsEmpty())
					{
						std::wistringstream istr(std::wstring(orientation.c_str()));
						osg::Vec3d center;
						double scale;
						osg::Vec3d eye;
						osg::Quat rot;

						istr >> eye.x() >> eye.y() >> eye.z()
							>> rot.x() >> rot.y() >> rot.z() >> rot.w()
							>> center.x() >> center.y() >> center.z() >> scale;

						osgGA::CameraManipulator *pManip = viewer_->getCameraManipulator();
						osgGA::TrackballManipulator *pTrackballManip = dynamic_cast<osgGA::TrackballManipulator *>(pManip);
						if(pTrackballManip != NULL)
						{
							pTrackballManip->setCenter(center);
							pTrackballManip->setDistance(scale);
							pTrackballManip->setTransformation(eye, rot);
						}

					}
				}
			}
		}
	}
}

BEGIN_EVENT_TABLE( Regard3DMainFrame, Regard3DMainFrameBase )
	EVT_TIMER(ID_TIMER, Regard3DMainFrame::OnTimer)
	EVT_TOOL(ID_NEWPROJECTTOOL, Regard3DMainFrame::OnNewProject)
	EVT_TOOL(ID_OPENPROJECTTOOL, Regard3DMainFrame::OnOpenProject)
	EVT_TOOL(ID_TEMP_DEV_TOOL, Regard3DMainFrame::OnTempDevToolClicked)
	EVT_COMMAND(ID_PREVIEW_FINISHED, myCustomEventType, Regard3DMainFrame::OnPreviewFinished)
	EVT_COMMAND(ID_NEW_IMAGE_INFOS, myCustomEventType, Regard3DMainFrame::OnNewImageInfos)
	EVT_COMMAND(ID_UPDATE_PROGRESS_BAR, myCustomEventType, Regard3DMainFrame::OnUpdateProgressBar)
	EVT_COMMAND(ID_COMPUTE_MATCHES_FINISHED, myCustomEventType, Regard3DMainFrame::OnComputeMatchesFinished)
	EVT_COMMAND(ID_TRIANGULATION_FINISHED, myCustomEventType, Regard3DMainFrame::OnTriangulationFinished)
	EVT_COMMAND(ID_DENSIFICATION_FINISHED, myCustomEventType, Regard3DMainFrame::OnDensificationFinished)
	EVT_COMMAND(ID_SURFACE_GEN_FINISHED, myCustomEventType, Regard3DMainFrame::OnSurfaceGenFinished)
	EVT_COMMAND(ID_SMALL_TASK_FINISHED, myCustomEventType, Regard3DMainFrame::OnSmallTaskFinished)
	EVT_MENU( ID_CTX_ADD_PICTURESET, Regard3DMainFrame::OnContextMenuAddPictureSet )
	EVT_MENU( ID_CTX_COMPUTE_MATCHES, Regard3DMainFrame::OnContextMenuComputeMatches )
	EVT_MENU( ID_CTX_EDIT_PICTURESET, Regard3DMainFrame::OnContextMenuEditPictureSet )
	EVT_MENU( ID_CTX_CLONE_PICTURESET, Regard3DMainFrame::OnContextMenuClonePictureSet )
	EVT_MENU( ID_CTX_SHOW_MATCHES, Regard3DMainFrame::OnContextMenuShowMatches )
	EVT_MENU( ID_CTX_TRIANGULATION, Regard3DMainFrame::OnContextMenuTriangulation )
	EVT_MENU( ID_CTX_CREATE_DENSE_POINTCLOUD, Regard3DMainFrame::OnContextMenuCreateDensePointcloud )
	EVT_MENU( ID_CTX_SHOW_TRIANGULATED_POINTS, Regard3DMainFrame::OnContextMenuShowTriangulatedPoints )
	EVT_MENU( ID_CTX_DELETE_PICTURESET, Regard3DMainFrame::OnContextMenuDeletePictureSet )
	EVT_MENU( ID_CTX_DELETE_COMPUTE_MATCHES, Regard3DMainFrame::OnContextMenuDeleteComputeMatches )
	EVT_MENU( ID_CTX_DELETE_TRIANGULATION, Regard3DMainFrame::OnContextMenuDeleteTriangulation )
	EVT_MENU( ID_CTX_CREATE_SURFACE, Regard3DMainFrame::OnContextMenuCreateSurface )
	EVT_MENU( ID_CTX_SHOW_DENSE_POINTS, Regard3DMainFrame::OnContextMenuShowDensePoints )
	EVT_MENU( ID_CTX_DELETE_DENSIFICATION, Regard3DMainFrame::OnContextMenuDeleteDensification )
	EVT_MENU( ID_CTX_SHOW_SURFACE, Regard3DMainFrame::OnContextMenuShowSurface )
	EVT_MENU( ID_CTX_DELETE_SURFACE, Regard3DMainFrame::OnContextMenuDeleteSurface )
	EVT_MENU( ID_CTX_EXPORT_TRIANGULATION_TO_CMPMVS, Regard3DMainFrame::OnContextMenuExportTriangulationToCMPMVS )
	EVT_MENU( ID_CTX_EXPORT_POINT_CLOUD, Regard3DMainFrame::OnContextMenuExportPointCloud )
	EVT_MENU( ID_CTX_EXPORT_DENSIFICATION_TO_MESHLAB, Regard3DMainFrame::OnContextMenuExportDensificationToMeshLab )
	EVT_MENU( ID_CTX_EXPORT_SURFACE, Regard3DMainFrame::OnContextMenuExportSurface )
END_EVENT_TABLE()
   