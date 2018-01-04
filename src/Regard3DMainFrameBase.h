///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Sep  8 2010)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __Regard3DMainFrameBase__
#define __Regard3DMainFrameBase__

#include <wx/string.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/menu.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/toolbar.h>
#include <wx/treectrl.h>
#include <wx/sizer.h>
#include <wx/panel.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/button.h>
#include <wx/scrolwin.h>
#include <wx/splitter.h>
#include "OSGGLCanvas.h"
#include <wx/checkbox.h>
#include <wx/slider.h>
#include <wx/statbox.h>
#include <wx/radiobox.h>
#include <wx/statusbr.h>
#include <wx/frame.h>
#include <wx/radiobut.h>
#include <wx/filepicker.h>
#include <wx/valtext.h>
#include <wx/dialog.h>
#include <wx/gauge.h>
#include "ImagePanel.h"
#include <wx/choice.h>
#include <wx/listctrl.h>
#include "PreviewCanvas.h"
#include <wx/choicebk.h>

///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
/// Class Regard3DMainFrameBase
///////////////////////////////////////////////////////////////////////////////
class Regard3DMainFrameBase : public wxFrame 
{
	DECLARE_EVENT_TABLE()
	private:
		
		// Private event handlers
		void _wxFB_OnMainFrameClose( wxCloseEvent& event ){ OnMainFrameClose( event ); }
		void _wxFB_OnMainFrameIdle( wxIdleEvent& event ){ OnMainFrameIdle( event ); }
		void _wxFB_OnNewProject( wxCommandEvent& event ){ OnNewProject( event ); }
		void _wxFB_OnOpenProject( wxCommandEvent& event ){ OnOpenProject( event ); }
		void _wxFB_OnCloseProject( wxCommandEvent& event ){ OnCloseProject( event ); }
		void _wxFB_OnMainFrameExitMenuItem( wxCommandEvent& event ){ OnMainFrameExitMenuItem( event ); }
		void _wxFB_OnPropertiesMenuItem( wxCommandEvent& event ){ OnPropertiesMenuItem( event ); }
		void _wxFB_OnEditUserCameraDBMenuSelection( wxCommandEvent& event ){ OnEditUserCameraDBMenuSelection( event ); }
		void _wxFB_OnViewConsoleOutputFrameMenuItem( wxCommandEvent& event ){ OnViewConsoleOutputFrameMenuItem( event ); }
		void _wxFB_OnViewConsoleOutputFrameUpdate( wxUpdateUIEvent& event ){ OnViewConsoleOutputFrameUpdate( event ); }
		void _wxFB_OnAboutMenuItem( wxCommandEvent& event ){ OnAboutMenuItem( event ); }
		void _wxFB_OnProjectTreeItemActivated( wxTreeEvent& event ){ OnProjectTreeItemActivated( event ); }
		void _wxFB_OnProjectTreeItemMenu( wxTreeEvent& event ){ OnProjectTreeItemMenu( event ); }
		void _wxFB_OnProjectTreeSelChanged( wxTreeEvent& event ){ OnProjectTreeSelChanged( event ); }
		void _wxFB_OnProjectTreeSelChanging( wxTreeEvent& event ){ OnProjectTreeSelChanging( event ); }
		void _wxFB_OnProjectAddPictureSetButton( wxCommandEvent& event ){ OnProjectAddPictureSetButton( event ); }
		void _wxFB_OnPictureSetComputeMatchesButton( wxCommandEvent& event ){ OnPictureSetComputeMatchesButton( event ); }
		void _wxFB_OnEditPictureSetButton( wxCommandEvent& event ){ OnEditPictureSetButton( event ); }
		void _wxFB_OnClonePictureSetButton( wxCommandEvent& event ){ OnClonePictureSetButton( event ); }
		void _wxFB_OnProjectDeletePictureSetButton( wxCommandEvent& event ){ OnProjectDeletePictureSetButton( event ); }
		void _wxFB_OnShowMatchingResultsButton( wxCommandEvent& event ){ OnShowMatchingResultsButton( event ); }
		void _wxFB_OnTriangulationButton( wxCommandEvent& event ){ OnTriangulationButton( event ); }
		void _wxFB_OnProjectDeleteComputeMatchesButton( wxCommandEvent& event ){ OnProjectDeleteComputeMatchesButton( event ); }
		void _wxFB_OnCreateDensePointcloudButton( wxCommandEvent& event ){ OnCreateDensePointcloudButton( event ); }
		void _wxFB_OnShowTriangulatedPoints( wxCommandEvent& event ){ OnShowTriangulatedPoints( event ); }
		void _wxFB_OnExportTriangulationToExternalMVSButton( wxCommandEvent& event ){ OnExportTriangulationToExternalMVSButton( event ); }
		void _wxFB_OnProjectDeleteTriangulationButton( wxCommandEvent& event ){ OnProjectDeleteTriangulationButton( event ); }
		void _wxFB_OnCreateSurfaceButton( wxCommandEvent& event ){ OnCreateSurfaceButton( event ); }
		void _wxFB_OnShowDensePointCloudButton( wxCommandEvent& event ){ OnShowDensePointCloudButton( event ); }
		void _wxFB_OnExportDensificationButton( wxCommandEvent& event ){ OnExportDensificationButton( event ); }
		void _wxFB_OnExportDensificationToMeshLabButton( wxCommandEvent& event ){ OnExportDensificationToMeshLabButton( event ); }
		void _wxFB_OnProjectDeleteDensificationButton( wxCommandEvent& event ){ OnProjectDeleteDensificationButton( event ); }
		void _wxFB_OnShowSurfaceButton( wxCommandEvent& event ){ OnShowSurfaceButton( event ); }
		void _wxFB_OnExportSurfaceButton( wxCommandEvent& event ){ OnExportSurfaceButton( event ); }
		void _wxFB_OnDeleteSurfaceButton( wxCommandEvent& event ){ OnDeleteSurfaceButton( event ); }
		void _wxFB_OnShowTrackballCheckBox( wxCommandEvent& event ){ OnShowTrackballCheckBox( event ); }
		void _wxFB_OnPointSizeScroll( wxScrollEvent& event ){ OnPointSizeScroll( event ); }
		void _wxFB_OnShowTextureCheckBox( wxCommandEvent& event ){ OnShowTextureCheckBox( event ); }
		void _wxFB_OnEnableLightingCheckBox( wxCommandEvent& event ){ OnEnableLightingCheckBox( event ); }
		void _wxFB_OnPolygonModeRadioBox( wxCommandEvent& event ){ OnPolygonModeRadioBox( event ); }
		void _wxFB_OnShadingModelRadioBox( wxCommandEvent& event ){ OnShadingModelRadioBox( event ); }
		void _wxFB_OnResetViewButton( wxCommandEvent& event ){ OnResetViewButton( event ); }
		
	
	protected:
		enum
		{
			ID_REGARD3DMAINFRAME = 1000,
			ID_MAINFRAMEMENUBAR,
			ID_NEWPROJECTMENUITEM,
			ID_OPENPROJECTMENUITEM,
			ID_CLOSEPROJECTMENUITEM,
			ID_PROPERTIESMENUITEM,
			ID_EDITUSERCAMERADBMENUITEM,
			ID_VIEWCONSOLEOUTPUTFRAMEMENUITEM,
			ID_MAINFRAMETOOLBAR,
			ID_MAINSPLITTER,
			ID_MAINSPLITTERLEFTPANEL,
			ID_PROJECTSPLITTER,
			ID_PROJECTTREECTRL,
			ID_PROJECTLOWERSCROLLEDWINDOW,
			ID_PROJECTNAMETEXTCTRL,
			ID_PROJECTPATHTEXTCTRL,
			ID_PROJECTADDPICTURESETBUTTON,
			ID_PICTURESETNAMETEXTCTRL,
			ID_PICTURESETIDTEXTCTRL,
			ID_PICTURESETSIZETEXTCTRL,
			ID_PICTURESETCOMPUTEMATCHESBUTTON,
			ID_EDITPICTURESETBUTTON,
			ID_CLONEPICTURESETBUTTON,
			ID_PROJECTDELETEPICTURESETBUTTON,
			ID_COMPUTEMATCHESIDTEXTCTRL,
			ID_COMPUTEMATCHESPARAMETERSTEXTCTRL,
			ID_COMPUTEMATCHESRESULTSTEXTCTRL,
			ID_COMPUTEMATCHESTIMETEXTCTRL,
			ID_SHOWMATCHINGRESULTSBUTTON,
			ID_TRIANGULATIONBUTTON,
			ID_PROJECTDELETECOMPUTEMATCHESBUTTON,
			ID_TRIANGULATIONIDTEXTCTRL,
			ID_TRIANGULATIONPARAMETERSTEXTCTRL,
			ID_TRIANGULATIONCAMERASTEXTCTRL,
			ID_TRIANGULATIONNUMBERPOINTSTEXTCTRL,
			ID_TRIANGULATIONRESIDUALERRORSTEXTCTRL,
			ID_TRIANGULATIONTIMETEXTCTRL,
			ID_CREATEDENSEPOINTCLOUDBUTTON,
			ID_SHOWTRIANGULATEDPOINTS,
			ID_EXPORTTRIANGULATIONTOEXTERNALMVSBUTTON,
			ID_PROJECTDELETETRIANGULATIONBUTTON,
			ID_DENSIFICATIONIDTEXTCTRL,
			ID_DENSIFICATIONTYPETEXTCTRL,
			ID_DENSIFICATIONPARAMETERSTEXTCTRL,
			ID_DENSIFICATIONRUNNINGTIMETEXTCTRL,
			ID_CREATESURFACEBUTTON,
			ID_SHOWDENSEPOINTCLOUDBUTTON,
			ID_EXPORTDENSIFICATIONBUTTON,
			ID_EXPORTDENSIFICATIONTOMESHLABBUTTON,
			ID_PPROJECTDELETEDENSIFICATIONBUTTON_,
			ID_SURFACEIDTEXTCTRL,
			ID_SURFACETYPETEXTCTRL,
			ID_SURFACEPARAMETERSTEXTCTRL,
			ID_SURFACECOLORIZATIONTYPETEXTCTRL,
			ID_SURFACECOLORIZATIONPARAMETERSTEXTCTRL,
			ID_SURFACERUNNINGTIMETEXTCTRL,
			ID_SHOWSURFACEBUTTON,
			ID_EXPORTSURFACEBUTTON,
			ID_DELETESURFACEBUTTON,
			ID_MAINSPLITTERRIGHTPANEL,
			ID_OSGGLCANVAS,
			ID_SHOWTRACKBALLCHECKBOX,
			ID_POINTSIZESLIDER,
			ID_POINTSIZETEXTCTRL,
			ID_SHOWTEXTURECHECKBOX,
			ID_ENABLELIGHTINGCHECKBOX,
			ID_POLYGONMODERADIOBOX,
			ID_SHADINGMODELRADIOBOX,
			ID_RESETVIEWBUTTON,
			ID_MAINSTATUSBAR,
		};
		
		wxMenuBar* pMainFrameMenuBar_;
		wxMenu* pFileMenu_;
		wxMenu* pOptionsMenu_;
		wxMenu* pViewMenu_;
		wxMenuItem* pViewConsoleOutputFrameMenuItem_;
		wxMenu* pHelpMenu_;
		wxToolBar* pMainFrameToolBar_;
		wxSplitterWindow* pMainSplitter_;
		wxPanel* pMainSplitterLeftPanel_;
		wxSplitterWindow* pProjectSplitter_;
		wxPanel* m_panel15;
		wxTreeCtrl* pProjectTreeCtrl_;
		wxScrolledWindow* pProjectLowerScrolledWindow_;
		wxBoxSizer* pProjectLowerSizer_;
		wxBoxSizer* pProjectProjectSizer_;
		wxStaticText* m_staticText112;
		wxTextCtrl* pProjectNameTextCtrl_;
		wxStaticText* m_staticText122;
		wxTextCtrl* pProjectPathTextCtrl_;
		wxButton* pProjectAddPictureSetButton_;
		wxBoxSizer* pProjectPictureSetSizer_;
		wxStaticText* m_staticText113;
		wxTextCtrl* pPictureSetNameTextCtrl_;
		wxStaticText* m_staticText123;
		wxTextCtrl* pPictureSetIdTextCtrl_;
		wxStaticText* m_staticText132;
		wxTextCtrl* pPictureSetSizeTextCtrl_;
		wxButton* pPictureSetComputeMatchesButton_;
		wxButton* pEditPictureSetButton_;
		wxButton* pClonePictureSetButton_;
		wxButton* pProjectDeletePictureSetButton_;
		wxBoxSizer* pProjectComputeMatchesSizer_;
		wxStaticText* m_staticText114;
		wxTextCtrl* pComputeMatchesIdTextCtrl_;
		wxStaticText* m_staticText124;
		wxTextCtrl* pComputeMatchesParametersTextCtrl_;
		wxStaticText* m_staticText133;
		wxTextCtrl* pComputeMatchesResultsTextCtrl_;
		wxStaticText* m_staticText45;
		wxTextCtrl* pComputeMatchesTimeTextCtrl_;
		wxButton* pShowMatchingResultsButton_;
		wxButton* pTriangulationButton_;
		wxButton* pProjectDeleteComputeMatchesButton_;
		wxBoxSizer* pProjectTriangulationSizer_;
		wxStaticText* m_staticText111;
		wxTextCtrl* pTriangulationIdTextCtrl_;
		wxStaticText* m_staticText121;
		wxTextCtrl* pTriangulationParametersTextCtrl_;
		wxStaticText* m_staticText131;
		wxTextCtrl* pTriangulationCamerasTextCtrl_;
		wxStaticText* m_staticText46;
		wxTextCtrl* pTriangulationNumberPointsTextCtrl_;
		wxStaticText* m_staticText47;
		wxTextCtrl* pTriangulationResidualErrorsTextCtrl_;
		wxStaticText* m_staticText48;
		wxTextCtrl* pTriangulationTimeTextCtrl_;
		wxButton* pCreateDensePointcloudButton_;
		wxButton* pShowTriangulatedPoints_;
		wxButton* pExportTriangulationToExternalMVSButton_;
		wxButton* pProjectDeleteTriangulationButton_;
		wxBoxSizer* pProjectDensificationSizer_;
		wxStaticText* m_staticText35;
		wxTextCtrl* pDensificationIdTextCtrl_;
		wxStaticText* m_staticText36;
		wxTextCtrl* pDensificationTypeTextCtrl_;
		wxStaticText* m_staticText37;
		wxTextCtrl* pDensificationParametersTextCtrl_;
		wxStaticText* m_staticText38;
		wxTextCtrl* pDensificationRunningTimeTextCtrl_;
		wxButton* pCreateSurfaceButton_;
		wxButton* pShowDensePointCloudButton_;
		wxButton* pExportDensificationButton_;
		wxButton* pExportDensificationToMeshLabButton_;
		wxButton* pProjectDeleteDensificationButton_;
		wxBoxSizer* pProjectSurfaceSizer_;
		wxStaticText* m_staticText461;
		wxTextCtrl* pSurfaceIdTextCtrl_;
		wxStaticText* m_staticText471;
		wxTextCtrl* pSurfaceTypeTextCtrl_;
		wxStaticText* m_staticText481;
		wxTextCtrl* pSurfaceParametersTextCtrl_;
		wxStaticText* m_staticText49;
		wxTextCtrl* pSurfaceColorizationTypeTextCtrl_;
		wxStaticText* m_staticText50;
		wxTextCtrl* pSurfaceColorizationParametersTextCtrl_;
		wxStaticText* m_staticText51;
		wxTextCtrl* pSurfaceRunningTimeTextCtrl_;
		wxButton* pShowSurfaceButton_;
		wxButton* pExportSurfaceButton_;
		wxButton* pDeleteSurfaceButton_;
		wxPanel* pMainSplitterRightPanel_;
		OSGGLCanvas *pOSGGLCanvas_;
		wxCheckBox* pShowTrackballCheckBox_;
		wxSlider* pPointSizeSlider_;
		wxTextCtrl* pPointSizeTextCtrl_;
		wxCheckBox* pShowTextureCheckBox_;
		wxCheckBox* pEnableLightingCheckBox_;
		wxRadioBox* pPolygonModeRadioBox_;
		wxRadioBox* pShadingModelRadioBox_;
		wxButton* pResetViewButton_;
		wxStatusBar* pMainStatusBar_;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnMainFrameClose( wxCloseEvent& event ) = 0;
		virtual void OnMainFrameIdle( wxIdleEvent& event ) = 0;
		virtual void OnNewProject( wxCommandEvent& event ) = 0;
		virtual void OnOpenProject( wxCommandEvent& event ) = 0;
		virtual void OnCloseProject( wxCommandEvent& event ) = 0;
		virtual void OnMainFrameExitMenuItem( wxCommandEvent& event ) = 0;
		virtual void OnPropertiesMenuItem( wxCommandEvent& event ) = 0;
		virtual void OnEditUserCameraDBMenuSelection( wxCommandEvent& event ) = 0;
		virtual void OnViewConsoleOutputFrameMenuItem( wxCommandEvent& event ) = 0;
		virtual void OnViewConsoleOutputFrameUpdate( wxUpdateUIEvent& event ) = 0;
		virtual void OnAboutMenuItem( wxCommandEvent& event ) = 0;
		virtual void OnProjectTreeItemActivated( wxTreeEvent& event ) = 0;
		virtual void OnProjectTreeItemMenu( wxTreeEvent& event ) = 0;
		virtual void OnProjectTreeSelChanged( wxTreeEvent& event ) = 0;
		virtual void OnProjectTreeSelChanging( wxTreeEvent& event ) = 0;
		virtual void OnProjectAddPictureSetButton( wxCommandEvent& event ) = 0;
		virtual void OnPictureSetComputeMatchesButton( wxCommandEvent& event ) = 0;
		virtual void OnEditPictureSetButton( wxCommandEvent& event ) = 0;
		virtual void OnClonePictureSetButton( wxCommandEvent& event ) = 0;
		virtual void OnProjectDeletePictureSetButton( wxCommandEvent& event ) = 0;
		virtual void OnShowMatchingResultsButton( wxCommandEvent& event ) = 0;
		virtual void OnTriangulationButton( wxCommandEvent& event ) = 0;
		virtual void OnProjectDeleteComputeMatchesButton( wxCommandEvent& event ) = 0;
		virtual void OnCreateDensePointcloudButton( wxCommandEvent& event ) = 0;
		virtual void OnShowTriangulatedPoints( wxCommandEvent& event ) = 0;
		virtual void OnExportTriangulationToExternalMVSButton( wxCommandEvent& event ) = 0;
		virtual void OnProjectDeleteTriangulationButton( wxCommandEvent& event ) = 0;
		virtual void OnCreateSurfaceButton( wxCommandEvent& event ) = 0;
		virtual void OnShowDensePointCloudButton( wxCommandEvent& event ) = 0;
		virtual void OnExportDensificationButton( wxCommandEvent& event ) = 0;
		virtual void OnExportDensificationToMeshLabButton( wxCommandEvent& event ) = 0;
		virtual void OnProjectDeleteDensificationButton( wxCommandEvent& event ) = 0;
		virtual void OnShowSurfaceButton( wxCommandEvent& event ) = 0;
		virtual void OnExportSurfaceButton( wxCommandEvent& event ) = 0;
		virtual void OnDeleteSurfaceButton( wxCommandEvent& event ) = 0;
		virtual void OnShowTrackballCheckBox( wxCommandEvent& event ) = 0;
		virtual void OnPointSizeScroll( wxScrollEvent& event ) = 0;
		virtual void OnShowTextureCheckBox( wxCommandEvent& event ) = 0;
		virtual void OnEnableLightingCheckBox( wxCommandEvent& event ) = 0;
		virtual void OnPolygonModeRadioBox( wxCommandEvent& event ) = 0;
		virtual void OnShadingModelRadioBox( wxCommandEvent& event ) = 0;
		virtual void OnResetViewButton( wxCommandEvent& event ) = 0;
		
	
	public:
		
		Regard3DMainFrameBase( wxWindow* parent, wxWindowID id = ID_REGARD3DMAINFRAME, const wxString& title = wxT("Regard 3D"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 1089,709 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
		~Regard3DMainFrameBase();
		
		void pMainSplitter_OnIdle( wxIdleEvent& )
		{
			pMainSplitter_->SetSashPosition( 250 );
			pMainSplitter_->Disconnect( wxEVT_IDLE, wxIdleEventHandler( Regard3DMainFrameBase::pMainSplitter_OnIdle ), NULL, this );
		}
		
		void pProjectSplitter_OnIdle( wxIdleEvent& )
		{
			pProjectSplitter_->SetSashPosition( 0 );
			pProjectSplitter_->Disconnect( wxEVT_IDLE, wxIdleEventHandler( Regard3DMainFrameBase::pProjectSplitter_OnIdle ), NULL, this );
		}
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class NewProjectDialogBase
///////////////////////////////////////////////////////////////////////////////
class NewProjectDialogBase : public wxDialog 
{
	DECLARE_EVENT_TABLE()
	private:
		
		// Private event handlers
		void _wxFB_OnUseDefaultProjectPathRadioButton( wxCommandEvent& event ){ OnUseDefaultProjectPathRadioButton( event ); }
		void _wxFB_OnSetProjectPathRadioButton( wxCommandEvent& event ){ OnSetProjectPathRadioButton( event ); }
		void _wxFB_OnProjectPathChanged( wxFileDirPickerEvent& event ){ OnProjectPathChanged( event ); }
		void _wxFB_OnProjectNameText( wxCommandEvent& event ){ OnProjectNameText( event ); }
		void _wxFB_OnOK( wxCommandEvent& event ){ OnOK( event ); }
		
	
	protected:
		enum
		{
			ID_NEWPROJECTDIALOG = 1000,
			ID_USEDEFAULTPROJECTPATHRADIOBUTTON,
			ID_SETPROJECTPATHRADIOBUTTON,
			ID_PROJECTPATHDIRPICKER,
			ID_PROJECTNAMETEXTCTRL,
			ID_PROJECTFILENAMETEXTCTRL,
		};
		
		wxPanel* m_panel7;
		wxRadioButton* pUseDefaultProjectPathRadioButton_;
		wxRadioButton* pSetProjectPathRadioButton_;
		wxDirPickerCtrl* pProjectPathDirPicker_;
		wxStaticText* m_staticText6;
		wxTextCtrl* pProjectNameTextCtrl_;
		wxStaticText* m_staticText7;
		wxTextCtrl* pProjectFilenameTextCtrl_;
		
		wxStdDialogButtonSizer* m_sdbSizer1;
		wxButton* m_sdbSizer1OK;
		wxButton* m_sdbSizer1Cancel;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnUseDefaultProjectPathRadioButton( wxCommandEvent& event ) = 0;
		virtual void OnSetProjectPathRadioButton( wxCommandEvent& event ) = 0;
		virtual void OnProjectPathChanged( wxFileDirPickerEvent& event ) = 0;
		virtual void OnProjectNameText( wxCommandEvent& event ) = 0;
		virtual void OnOK( wxCommandEvent& event ) = 0;
		
	
	public:
		wxString projectName_; 
		
		NewProjectDialogBase( wxWindow* parent, wxWindowID id = ID_NEWPROJECTDIALOG, const wxString& title = wxT("Create new Regard3D project"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( -1,-1 ), long style = wxDEFAULT_DIALOG_STYLE );
		~NewProjectDialogBase();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class Regard3DConsoleOutputFrameBase
///////////////////////////////////////////////////////////////////////////////
class Regard3DConsoleOutputFrameBase : public wxFrame 
{
	DECLARE_EVENT_TABLE()
	private:
		
		// Private event handlers
		void _wxFB_OnConsoleOutputClose( wxCloseEvent& event ){ OnConsoleOutputClose( event ); }
		void _wxFB_OnClearConsoleOutputButton( wxCommandEvent& event ){ OnClearConsoleOutputButton( event ); }
		
	
	protected:
		enum
		{
			ID_REGARD3DCONSOLEOUTPUTFRAMEBASE = 1000,
			ID_CONSOLEOUTPUTTEXTCTRL,
			ID_CLEARCONSOLEOUTPUTBUTTON,
		};
		
		wxPanel* m_panel7;
		wxTextCtrl* pConsoleOutputTextCtrl_;
		
		wxButton* pClearConsoleOutputButton_;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnConsoleOutputClose( wxCloseEvent& event ) = 0;
		virtual void OnClearConsoleOutputButton( wxCommandEvent& event ) = 0;
		
	
	public:
		
		Regard3DConsoleOutputFrameBase( wxWindow* parent, wxWindowID id = ID_REGARD3DCONSOLEOUTPUTFRAMEBASE, const wxString& title = wxT("Console output"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 500,300 ), long style = wxCAPTION|wxCLOSE_BOX|wxFRAME_TOOL_WINDOW|wxMAXIMIZE_BOX|wxRESIZE_BORDER|wxSYSTEM_MENU|wxTAB_TRAVERSAL );
		~Regard3DConsoleOutputFrameBase();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class Regard3DProgressDialogBase
///////////////////////////////////////////////////////////////////////////////
class Regard3DProgressDialogBase : public wxDialog 
{
	DECLARE_EVENT_TABLE()
	private:
		
		// Private event handlers
		void _wxFB_OnShowOutputWindowButton( wxCommandEvent& event ){ OnShowOutputWindowButton( event ); }
		
	
	protected:
		enum
		{
			ID_REGARD3DPROGRESSDIALOG = 1000,
			ID_PROGRESSTEXT,
			ID_PROGRESSSTATUSTEXTCTRL,
			ID_ELAPSEDTIMETEXTCTRL,
			ID_PROGRESSGAUGE,
			ID_SHOWOUTPUTWINDOWBUTTON,
		};
		
		wxPanel* m_panel8;
		wxStaticText* pProgressText_;
		wxTextCtrl* pProgressStatusTextCtrl_;
		wxStaticText* m_staticText9;
		wxTextCtrl* pElapsedTimeTextCtrl_;
		wxGauge* pProgressGauge_;
		wxButton* pShowOutputWindowButton_;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnShowOutputWindowButton( wxCommandEvent& event ) = 0;
		
	
	public:
		
		Regard3DProgressDialogBase( wxWindow* parent, wxWindowID id = ID_REGARD3DPROGRESSDIALOG, const wxString& title = wxT("Progress dialog"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 400,190 ), long style = wxCAPTION|wxSYSTEM_MENU );
		~Regard3DProgressDialogBase();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class Regard3DImagePreviewDialogBase
///////////////////////////////////////////////////////////////////////////////
class Regard3DImagePreviewDialogBase : public wxDialog 
{
	DECLARE_EVENT_TABLE()
	private:
		
		// Private event handlers
		void _wxFB_OnClose( wxCloseEvent& event ){ OnClose( event ); }
		void _wxFB_OnInitDialog( wxInitDialogEvent& event ){ OnInitDialog( event ); }
		void _wxFB_OnKeypointTypeChoice( wxCommandEvent& event ){ OnKeypointTypeChoice( event ); }
		void _wxFB_OnShowSingleKeypointsCheckBox( wxCommandEvent& event ){ OnShowSingleKeypointsCheckBox( event ); }
		void _wxFB_OnMatchesChoice( wxCommandEvent& event ){ OnMatchesChoice( event ); }
		void _wxFB_OnEnableTrackFilter( wxCommandEvent& event ){ OnEnableTrackFilter( event ); }
		void _wxFB_OnZoomInButton( wxCommandEvent& event ){ OnZoomInButton( event ); }
		void _wxFB_OnZoomOutButton( wxCommandEvent& event ){ OnZoomOutButton( event ); }
		void _wxFB_OnCloseButton( wxCommandEvent& event ){ OnCloseButton( event ); }
		
	
	protected:
		enum
		{
			ID_REGARD3DIMAGEPREVIEWDIALOG = 1000,
			ID_IMAGEPREVIEWDIALOGPANEL,
			ID_IMAGEPANEL,
			ID_KEYPOINTTYPECHOICE,
			ID_SHOWSINGLEKEYPOINTSCHECKBOX,
			ID_MATCHESCHOICE,
			ID_ENABLETRACKFILTERCHECKBOX,
			ID_ZOOMFACTORTEXTCTRL,
			ID_ZOOMINBUTTON,
			ID_ZOOMOUTBUTTON,
			ID_CLOSEBUTTON,
		};
		
		wxPanel* pImagePreviewDialogPanel_;
		ImagePanel *pImagePanel_;
		wxChoice* pKeypointTypeChoice_;
		wxCheckBox* pShowSingleKeypointsCheckBox_;
		wxChoice* pMatchesChoice_;
		wxCheckBox* pEnableTrackFilterCheckBox_;
		
		wxStaticText* m_staticText10;
		wxTextCtrl* pZoomFactorTextCtrl_;
		wxButton* pZoomInButton_;
		wxButton* pZoomOutButton_;
		
		wxButton* pCloseButton_;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnClose( wxCloseEvent& event ) = 0;
		virtual void OnInitDialog( wxInitDialogEvent& event ) = 0;
		virtual void OnKeypointTypeChoice( wxCommandEvent& event ) = 0;
		virtual void OnShowSingleKeypointsCheckBox( wxCommandEvent& event ) = 0;
		virtual void OnMatchesChoice( wxCommandEvent& event ) = 0;
		virtual void OnEnableTrackFilter( wxCommandEvent& event ) = 0;
		virtual void OnZoomInButton( wxCommandEvent& event ) = 0;
		virtual void OnZoomOutButton( wxCommandEvent& event ) = 0;
		virtual void OnCloseButton( wxCommandEvent& event ) = 0;
		
	
	public:
		
		Regard3DImagePreviewDialogBase( wxWindow* parent, wxWindowID id = ID_REGARD3DIMAGEPREVIEWDIALOG, const wxString& title = wxT("Image Preview"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 800,500 ), long style = wxDEFAULT_DIALOG_STYLE|wxMAXIMIZE_BOX|wxRESIZE_BORDER );
		~Regard3DImagePreviewDialogBase();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class Regard3DPictureSetDialogBase
///////////////////////////////////////////////////////////////////////////////
class Regard3DPictureSetDialogBase : public wxDialog 
{
	DECLARE_EVENT_TABLE()
	private:
		
		// Private event handlers
		void _wxFB_OnInitDialog( wxInitDialogEvent& event ){ OnInitDialog( event ); }
		void _wxFB_OnAddFilesButton( wxCommandEvent& event ){ OnAddFilesButton( event ); }
		void _wxFB_OnRemoveSelectedFilesButton( wxCommandEvent& event ){ OnRemoveSelectedFilesButton( event ); }
		void _wxFB_OnClearFileListButton( wxCommandEvent& event ){ OnClearFileListButton( event ); }
		void _wxFB_OnImageListColClick( wxListEvent& event ){ OnImageListColClick( event ); }
		void _wxFB_OnImageListItemDeselected( wxListEvent& event ){ OnImageListItemDeselected( event ); }
		void _wxFB_OnListItemRightClick( wxListEvent& event ){ OnListItemRightClick( event ); }
		void _wxFB_OnImageListItemSelected( wxListEvent& event ){ OnImageListItemSelected( event ); }
		void _wxFB_OnImageListKeyDown( wxListEvent& event ){ OnImageListKeyDown( event ); }
		void _wxFB_OnCancel( wxCommandEvent& event ){ OnCancel( event ); }
		void _wxFB_OnOK( wxCommandEvent& event ){ OnOK( event ); }
		
	
	protected:
		enum
		{
			ID_REGARD3DPICTURESETDIALOG = 1000,
			ID_PCENTRALSPLITTER_,
			ID_ADDFILESBUTTON,
			ID_REMOVESELECTEDFILESBUTTON,
			ID_CLEARFILELISTBUTTON,
			ID_IMAGELISTCTRL,
			ID_PICTURESETNAMETEXTCTRL,
			ID_IMAGEPREVIEWPANEL,
			ID_IPPREVIEWCANVAS,
		};
		
		wxPanel* m_panel13;
		wxPanel* m_panel10;
		wxButton* pAddFilesButton_;
		wxButton* pRemoveSelectedFilesButton_;
		wxButton* pClearFileListButton_;
		wxListCtrl* pImageListCtrl_;
		wxStaticText* m_staticText14;
		wxTextCtrl* pPictureSetNameTextCtrl_;
		wxPanel* pImagePreviewPanel_;
		PreviewCanvas *pPreviewCanvas_;
		wxStdDialogButtonSizer* m_sdbSizer2;
		wxButton* m_sdbSizer2OK;
		wxButton* m_sdbSizer2Cancel;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnInitDialog( wxInitDialogEvent& event ) = 0;
		virtual void OnAddFilesButton( wxCommandEvent& event ) = 0;
		virtual void OnRemoveSelectedFilesButton( wxCommandEvent& event ) = 0;
		virtual void OnClearFileListButton( wxCommandEvent& event ) = 0;
		virtual void OnImageListColClick( wxListEvent& event ) = 0;
		virtual void OnImageListItemDeselected( wxListEvent& event ) = 0;
		virtual void OnListItemRightClick( wxListEvent& event ) = 0;
		virtual void OnImageListItemSelected( wxListEvent& event ) = 0;
		virtual void OnImageListKeyDown( wxListEvent& event ) = 0;
		virtual void OnCancel( wxCommandEvent& event ) = 0;
		virtual void OnOK( wxCommandEvent& event ) = 0;
		
	
	public:
		wxSplitterWindow* pCentralSplitter_;
		
		Regard3DPictureSetDialogBase( wxWindow* parent, wxWindowID id = ID_REGARD3DPICTURESETDIALOG, const wxString& title = wxT("Picture Set"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( -1,-1 ), long style = wxDEFAULT_DIALOG_STYLE|wxMAXIMIZE_BOX|wxRESIZE_BORDER );
		~Regard3DPictureSetDialogBase();
		
		void pCentralSplitter_OnIdle( wxIdleEvent& )
		{
			pCentralSplitter_->SetSashPosition( 0 );
			pCentralSplitter_->Disconnect( wxEVT_IDLE, wxIdleEventHandler( Regard3DPictureSetDialogBase::pCentralSplitter_OnIdle ), NULL, this );
		}
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class Regard3DComputeMatchesDialogBase
///////////////////////////////////////////////////////////////////////////////
class Regard3DComputeMatchesDialogBase : public wxDialog 
{
	DECLARE_EVENT_TABLE()
	private:
		
		// Private event handlers
		void _wxFB_OnClose( wxCloseEvent& event ){ OnClose( event ); }
		void _wxFB_OnInitDialog( wxInitDialogEvent& event ){ OnInitDialog( event ); }
		void _wxFB_OnKeypointSensitivitySlider( wxScrollEvent& event ){ OnKeypointSensitivitySlider( event ); }
		void _wxFB_OnKeypointMatchingRatioSlider( wxScrollEvent& event ){ OnKeypointMatchingRatioSlider( event ); }
		void _wxFB_OnOKButtonClick( wxCommandEvent& event ){ OnOKButtonClick( event ); }
		
	
	protected:
		enum
		{
			ID_REGARD3DCOMPUTEMATCHESDIALOG = 1000,
			ID_COMPUTEMATCHESDIALOGPANEL,
			ID_KEYPOINTSENSITIVITYTEXTCTRL,
			ID_KEYPOINTSENSITIVITYSLIDER,
			ID_KEYPOINTSENSITIVITYVALTEXTCTRL,
			ID_KEYPOINTMATCHINGRATIOTEXTCTRL,
			ID_KEYPOINTMATCHINGRATIOSLIDER,
			ID_KEYPOINTMATCHINGRATIOVALTEXTCTRL,
			ID_KEYPOINTDETECTORRADIOBOX,
			ID_ADDTBMRDETECTORCHECKBOX,
			ID_MATCHINGALGORITHMCHOICE,
			ID_CAMERAMODELCHOICE,
		};
		
		wxPanel* pComputeMatchesDialogPanel_;
		wxStaticText* m_staticText1;
		wxTextCtrl* pKeypointSensitivityTextCtrl_;
		wxSlider* pKeypointSensitivitySlider_;
		wxTextCtrl* pKeypointSensitivityValTextCtrl_;
		wxStaticText* m_staticText2;
		wxTextCtrl* pKeypointMatchingRatioTextCtrl_;
		wxSlider* pKeypointMatchingRatioSlider_;
		wxTextCtrl* pKeypointMatchingRatioValTextCtrl_;
		wxRadioBox* pKeypointDetectorRadioBox_;
		wxCheckBox* pAddTBMRDetectorCheckBox_;
		wxChoice* pMatchingAlgorithmChoice_;
		wxChoice* pCameraModelChoice_;
		wxStdDialogButtonSizer* m_sdbSizer3;
		wxButton* m_sdbSizer3OK;
		wxButton* m_sdbSizer3Cancel;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnClose( wxCloseEvent& event ) = 0;
		virtual void OnInitDialog( wxInitDialogEvent& event ) = 0;
		virtual void OnKeypointSensitivitySlider( wxScrollEvent& event ) = 0;
		virtual void OnKeypointMatchingRatioSlider( wxScrollEvent& event ) = 0;
		virtual void OnOKButtonClick( wxCommandEvent& event ) = 0;
		
	
	public:
		
		Regard3DComputeMatchesDialogBase( wxWindow* parent, wxWindowID id = ID_REGARD3DCOMPUTEMATCHESDIALOG, const wxString& title = wxT("Compute Matches"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( -1,-1 ), long style = wxDEFAULT_DIALOG_STYLE );
		~Regard3DComputeMatchesDialogBase();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class Regard3DTriangulationDialogBase
///////////////////////////////////////////////////////////////////////////////
class Regard3DTriangulationDialogBase : public wxDialog 
{
	DECLARE_EVENT_TABLE()
	private:
		
		// Private event handlers
		void _wxFB_OnInitDialog( wxInitDialogEvent& event ){ OnInitDialog( event ); }
		void _wxFB_OnTriangulationMethodRadioBox( wxCommandEvent& event ){ OnTriangulationMethodRadioBox( event ); }
		void _wxFB_OnTInitialImagePairColClick( wxListEvent& event ){ OnTInitialImagePairColClick( event ); }
		void _wxFB_OnTInitialImagePairItemDeselected( wxListEvent& event ){ OnTInitialImagePairItemDeselected( event ); }
		void _wxFB_OnTInitialImagePairItemSelected( wxListEvent& event ){ OnTInitialImagePairItemSelected( event ); }
		void _wxFB_OnTPreviewWithMatchesCheckBox( wxCommandEvent& event ){ OnTPreviewWithMatchesCheckBox( event ); }
		
	
	protected:
		enum
		{
			ID_REGARD3DTRIANGULATIONDIALOG = 1000,
			ID_TRIANGULATIONPANEL,
			ID_TRIANGULATIONMETHODRADIOBOX,
			ID_TREFINECAMERAINTRINSICSCHECKBOX,
			ID_TINCREMENTALMETHODBOXSIZER,
			ID_INCREMENTALMETHODBOXSPLITTER,
			ID_INCREMENTALMETHODBOXLEFTPANEL,
			ID_TINITIALIMAGEPAIRLISTCTRL,
			ID_INCREMENTALMETHODBOXRIGHTPANEL,
			ID_PREVIEWCANVAS,
			ID_TPREVIEWWITHMATCHESCHECKBOX,
			ID_TGLOBALMETHODBOXSIZER,
			ID_TGLOBALROTAVGMETHODRATIOBOX,
			ID_TGLOBALTRANSLAVGMETHODRADIOBOX,
		};
		
		wxPanel* pTriangulationPanel_;
		wxRadioBox* pTriangulationMethodRadioBox_;
		
		wxCheckBox* pTRefineCameraIntrinsicsCheckBox_;
		
		wxStaticBoxSizer* pTIncrementalMethodBoxSizer_;
		wxPanel* pIncrementalMethodBoxLeftPanel_;
		wxListCtrl* pTInitialImagePairListCtrl_;
		wxPanel* pIncrementalMethodBoxRightPanel_;
		PreviewCanvas *pPreviewCanvas_;
		wxCheckBox* pTPreviewWithMatchesCheckBox_;
		
		wxStaticBoxSizer* pTGlobalMethodBoxSizer_;
		wxRadioBox* pTGlobalRotAvgMethodRatioBox_;
		wxRadioBox* pTGlobalTranslAvgMethodRadioBox_;
		wxStdDialogButtonSizer* m_sdbSizer4;
		wxButton* m_sdbSizer4OK;
		wxButton* m_sdbSizer4Cancel;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnInitDialog( wxInitDialogEvent& event ) = 0;
		virtual void OnTriangulationMethodRadioBox( wxCommandEvent& event ) = 0;
		virtual void OnTInitialImagePairColClick( wxListEvent& event ) = 0;
		virtual void OnTInitialImagePairItemDeselected( wxListEvent& event ) = 0;
		virtual void OnTInitialImagePairItemSelected( wxListEvent& event ) = 0;
		virtual void OnTPreviewWithMatchesCheckBox( wxCommandEvent& event ) = 0;
		
	
	public:
		wxSplitterWindow* pIncrementalMethodBoxSplitter_;
		
		Regard3DTriangulationDialogBase( wxWindow* parent, wxWindowID id = ID_REGARD3DTRIANGULATIONDIALOG, const wxString& title = wxT("Triangulation"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( -1,-1 ), long style = wxDEFAULT_DIALOG_STYLE|wxMAXIMIZE_BOX|wxRESIZE_BORDER );
		~Regard3DTriangulationDialogBase();
		
		void pIncrementalMethodBoxSplitter_OnIdle( wxIdleEvent& )
		{
			pIncrementalMethodBoxSplitter_->SetSashPosition( 0 );
			pIncrementalMethodBoxSplitter_->Disconnect( wxEVT_IDLE, wxIdleEventHandler( Regard3DTriangulationDialogBase::pIncrementalMethodBoxSplitter_OnIdle ), NULL, this );
		}
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class Regard3DMatchingResultsDialogBase
///////////////////////////////////////////////////////////////////////////////
class Regard3DMatchingResultsDialogBase : public wxDialog 
{
	DECLARE_EVENT_TABLE()
	private:
		
		// Private event handlers
		void _wxFB_OnInitDialog( wxInitDialogEvent& event ){ OnInitDialog( event ); }
		void _wxFB_OnIPImageListColClick( wxListEvent& event ){ OnIPImageListColClick( event ); }
		void _wxFB_OnIPImageListItemDeselected( wxListEvent& event ){ OnIPImageListItemDeselected( event ); }
		void _wxFB_OnIPImageListItemSelected( wxListEvent& event ){ OnIPImageListItemSelected( event ); }
		void _wxFB_OnIPImageListKeyDown( wxListEvent& event ){ OnIPImageListKeyDown( event ); }
		void _wxFB_OnIPPreviewWithKeypointsCheckBox( wxCommandEvent& event ){ OnIPPreviewWithKeypointsCheckBox( event ); }
		void _wxFB_OnIPOpenPreviewWindow( wxCommandEvent& event ){ OnIPOpenPreviewWindow( event ); }
		void _wxFB_OnMatchesChoice( wxCommandEvent& event ){ OnMatchesChoice( event ); }
		void _wxFB_OnTInitialImagePairColClick( wxListEvent& event ){ OnTInitialImagePairColClick( event ); }
		void _wxFB_OnTInitialImagePairItemDeselected( wxListEvent& event ){ OnTInitialImagePairItemDeselected( event ); }
		void _wxFB_OnTInitialImagePairItemSelected( wxListEvent& event ){ OnTInitialImagePairItemSelected( event ); }
		void _wxFB_OnTPreviewWithMatchesCheckBox( wxCommandEvent& event ){ OnTPreviewWithMatchesCheckBox( event ); }
		void _wxFB_OnTOpenPreviewWindow( wxCommandEvent& event ){ OnTOpenPreviewWindow( event ); }
		
	
	protected:
		enum
		{
			ID_REGARD3DMATCHINGRESULTSDIALOG = 1000,
			ID_MATCHINGRESULTSPANEL,
			ID_PHORIZONTALSPLITTER_,
			ID_KEYPOINTSPANEL,
			ID_UPPERVERTICALSPLITTER,
			ID_IMAGELISTPANEL,
			ID_IMAGELISTCTRL,
			ID_KEYPOINTPREVIEWPANEL,
			ID_IPPREVIEWCANVAS,
			ID_IPPREVIEWWITHKEYPOINTSCHECKBOX,
			ID_IPOPENPREVIEWWINDOW,
			ID_MATCHESPANEL,
			ID_LOWERVERTICALSPLITTER,
			ID_MATCHINGLISTPANEL,
			ID_MATCHESCHOICE,
			ID_TINITIALIMAGEPAIRLISTCTRL,
			ID_MATCHINGPREVIEWPANEL,
			ID_TPREVIEWCANVAS,
			ID_TPREVIEWWITHMATCHESCHECKBOX,
			ID_TOPENPREVIEWWINDOW,
		};
		
		wxPanel* pMatchingResultsPanel_;
		wxPanel* pKeypointsPanel_;
		wxPanel* pImageListPanel_;
		wxListCtrl* pImageListCtrl_;
		wxPanel* pKeypointPreviewPanel_;
		PreviewCanvas *pIPPreviewCanvas_;
		wxCheckBox* pIPPreviewWithKeypointsCheckBox_;
		
		wxButton* pIPOpenPreviewWindow_;
		wxPanel* pMatchesPanel_;
		wxPanel* pMatchingListPanel_;
		
		wxStaticText* m_staticText20;
		wxChoice* pMatchesChoice_;
		
		wxListCtrl* pTInitialImagePairListCtrl_;
		wxPanel* pMatchingPreviewPanel_;
		PreviewCanvas *pTPreviewCanvas_;
		wxCheckBox* pTPreviewWithMatchesCheckBox_;
		
		wxButton* pTOpenPreviewWindow_;
		wxStdDialogButtonSizer* m_sdbSizer5;
		wxButton* m_sdbSizer5OK;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnInitDialog( wxInitDialogEvent& event ) = 0;
		virtual void OnIPImageListColClick( wxListEvent& event ) = 0;
		virtual void OnIPImageListItemDeselected( wxListEvent& event ) = 0;
		virtual void OnIPImageListItemSelected( wxListEvent& event ) = 0;
		virtual void OnIPImageListKeyDown( wxListEvent& event ) = 0;
		virtual void OnIPPreviewWithKeypointsCheckBox( wxCommandEvent& event ) = 0;
		virtual void OnIPOpenPreviewWindow( wxCommandEvent& event ) = 0;
		virtual void OnMatchesChoice( wxCommandEvent& event ) = 0;
		virtual void OnTInitialImagePairColClick( wxListEvent& event ) = 0;
		virtual void OnTInitialImagePairItemDeselected( wxListEvent& event ) = 0;
		virtual void OnTInitialImagePairItemSelected( wxListEvent& event ) = 0;
		virtual void OnTPreviewWithMatchesCheckBox( wxCommandEvent& event ) = 0;
		virtual void OnTOpenPreviewWindow( wxCommandEvent& event ) = 0;
		
	
	public:
		wxSplitterWindow* pHorizontalSplitter_;
		wxSplitterWindow* pUpperVerticalSplitter_;
		wxSplitterWindow* pLowerVerticalSplitter_;
		
		Regard3DMatchingResultsDialogBase( wxWindow* parent, wxWindowID id = ID_REGARD3DMATCHINGRESULTSDIALOG, const wxString& title = wxT("Matching results"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( -1,-1 ), long style = wxDEFAULT_DIALOG_STYLE|wxMAXIMIZE_BOX|wxRESIZE_BORDER );
		~Regard3DMatchingResultsDialogBase();
		
		void pHorizontalSplitter_OnIdle( wxIdleEvent& )
		{
			pHorizontalSplitter_->SetSashPosition( 0 );
			pHorizontalSplitter_->Disconnect( wxEVT_IDLE, wxIdleEventHandler( Regard3DMatchingResultsDialogBase::pHorizontalSplitter_OnIdle ), NULL, this );
		}
		
		void pUpperVerticalSplitter_OnIdle( wxIdleEvent& )
		{
			pUpperVerticalSplitter_->SetSashPosition( 0 );
			pUpperVerticalSplitter_->Disconnect( wxEVT_IDLE, wxIdleEventHandler( Regard3DMatchingResultsDialogBase::pUpperVerticalSplitter_OnIdle ), NULL, this );
		}
		
		void pLowerVerticalSplitter_OnIdle( wxIdleEvent& )
		{
			pLowerVerticalSplitter_->SetSashPosition( 0 );
			pLowerVerticalSplitter_->Disconnect( wxEVT_IDLE, wxIdleEventHandler( Regard3DMatchingResultsDialogBase::pLowerVerticalSplitter_OnIdle ), NULL, this );
		}
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class Regard3DDensificationDialogBase
///////////////////////////////////////////////////////////////////////////////
class Regard3DDensificationDialogBase : public wxDialog 
{
	DECLARE_EVENT_TABLE()
	private:
		
		// Private event handlers
		void _wxFB_OnInitDialog( wxInitDialogEvent& event ){ OnInitDialog( event ); }
		void _wxFB_OnUseCMVSCheckBox( wxCommandEvent& event ){ OnUseCMVSCheckBox( event ); }
		void _wxFB_OnPMVSLevelSliderScroll( wxScrollEvent& event ){ OnPMVSLevelSliderScroll( event ); }
		void _wxFB_OnPMVSCellSizeSliderScroll( wxScrollEvent& event ){ OnPMVSCellSizeSliderScroll( event ); }
		void _wxFB_OnPMVSThresholdSliderScroll( wxScrollEvent& event ){ OnPMVSThresholdSliderScroll( event ); }
		void _wxFB_OnPMVSWSizeSliderScroll( wxScrollEvent& event ){ OnPMVSWSizeSliderScroll( event ); }
		void _wxFB_OnPMVSMinImageNumSliderScroll( wxScrollEvent& event ){ OnPMVSMinImageNumSliderScroll( event ); }
		void _wxFB_OnMVEScaleSliderScroll( wxScrollEvent& event ){ OnMVEScaleSliderScroll( event ); }
		void _wxFB_OnMVEFilterWidthSliderScroll( wxScrollEvent& event ){ OnMVEFilterWidthSliderScroll( event ); }
		void _wxFB_OnSMVSInputScaleSliderScroll( wxScrollEvent& event ){ OnSMVSInputScaleSliderScroll( event ); }
		void _wxFB_OnSMVSOutputScaleSliderScroll( wxScrollEvent& event ){ OnSMVSOutputScaleSliderScroll( event ); }
		void _wxFB_OnSMVSSurfaceSmoothingFactorSliderScroll( wxScrollEvent& event ){ OnSMVSSurfaceSmoothingFactorSliderScroll( event ); }
		
	
	protected:
		enum
		{
			ID_REGARD3DDENSIFICATIONDIALOG = 1000,
			ID_DENSIFICATIONPANEL,
			ID_DENSIFICATIONMETHODSIZER,
			ID_DENSIFICATIONMETHODCHOICEBOOK,
			ID_PMVSPARAMSPANEL,
			ID_PMVSPARAMSBOXSIZER,
			ID_NUMBEROFTHREADSCHOICE,
			ID_USECMVSCHECKBOX,
			ID_MAXIMAGETEXTCTRL,
			ID_PMVSLEVELTEXTCTRL,
			ID_PMVSLEVELSLIDER,
			ID_PMVSCELLSIZETEXTCTRL,
			ID_PMVSCELLSIZESLIDER,
			ID_PMVSTHRESHOLDTEXTCTRL,
			ID_PMVSTHRESHOLDSLIDER,
			ID_PMVSWSIZETEXTCTRL,
			ID_PMVSWSIZESLIDER,
			ID_PMVSMINIMAGENUMTEXTCTRL,
			ID_PMVSMINIMAGENUMSLIDER,
			ID_DMRECONPARAMSPANEL,
			ID_MVEPARAMSBOXSIZER,
			ID_MVESCALETEXTCTRL,
			ID_MVESCALESLIDER,
			ID_MVEFILTERWIDTHTEXTCTRL,
			ID_MVEFILTERWIDTHSLIDER,
			ID_SMVSRECONPARAMSPANEL,
			ID_SMVSINPUTSCALETEXTCTRL,
			ID_SMVSINPUTSCALESLIDER,
			ID_SMVSOUTPUTSCALETEXTCTRL,
			ID_SMVSOUTPUTSCALESLIDER,
			ID_SMVSSHADINGOPTCHECKBOX,
			ID_SMVSSEMIGLOBALMATCIHINGCHECKBOX,
			ID_SMVSSURFACESMOOTHINGFACTORTEXTCTRL,
			ID_SMVSSURFACESMOOTHINGFACTORSLIDER,
		};
		
		wxPanel* pDensificationPanel_;
		wxChoicebook* pDensificationMethodChoicebook_;
		wxPanel* pPMVSParamsPanel_;
		wxStaticBoxSizer* pPMVSParamsBoxSizer_;
		wxStaticText* m_staticText29;
		wxChoice* pNumberOfThreadsChoice_;
		wxCheckBox* pUseCMVSCheckBox_;
		wxStaticText* m_staticText28;
		wxTextCtrl* pMaxImageTextCtrl_;
		wxStaticText* m_staticText30;
		wxTextCtrl* pPMVSLevelTextCtrl_;
		wxSlider* pPMVSLevelSlider_;
		wxStaticText* m_staticText31;
		wxTextCtrl* pPMVSCellSizeTextCtrl_;
		wxSlider* pPMVSCellSizeSlider_;
		wxStaticText* m_staticText32;
		wxTextCtrl* pPMVSThresholdTextCtrl_;
		wxSlider* pPMVSThresholdSlider_;
		wxStaticText* m_staticText33;
		wxTextCtrl* pPMVSWSizeTextCtrl_;
		wxSlider* pPMVSWSizeSlider_;
		wxStaticText* m_staticText34;
		wxTextCtrl* pPMVSMinImageNumTextCtrl_;
		wxSlider* pPMVSMinImageNumSlider_;
		wxPanel* pDMReconParamsPanel_;
		wxStaticBoxSizer* pMVEParamsBoxSizer_;
		wxStaticText* m_staticText53;
		wxTextCtrl* pMVEScaleTextCtrl_;
		wxSlider* pMVEScaleSlider_;
		wxStaticText* m_staticText54;
		wxTextCtrl* pMVEFilterWidthTextCtrl_;
		wxSlider* pMVEFilterWidthSlider_;
		wxPanel* pSMVSReconParamsPanel_;
		wxStaticText* m_staticText57;
		wxTextCtrl* pSMVSInputScaleTextCtrl_;
		wxSlider* pSMVSInputScaleSlider_;
		wxStaticText* m_staticText58;
		wxTextCtrl* pSMVSOutputScaleTextCtrl_;
		wxSlider* pSMVSOutputScaleSlider_;
		wxStaticText* m_staticText59;
		wxCheckBox* pSMVSShadingOptCheckBox_;
		
		wxStaticText* m_staticText60;
		wxCheckBox* pSMVSSemiGlobalMatcihingCheckBox_;
		
		wxStaticText* m_staticText61;
		wxTextCtrl* pSMVSSurfaceSmoothingFactorTextCtrl_;
		wxSlider* pSMVSSurfaceSmoothingFactorSlider_;
		
		wxStdDialogButtonSizer* pStdDialogButtonSizer_;
		wxButton* pStdDialogButtonSizer_OK;
		wxButton* pStdDialogButtonSizer_Cancel;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnInitDialog( wxInitDialogEvent& event ) = 0;
		virtual void OnUseCMVSCheckBox( wxCommandEvent& event ) = 0;
		virtual void OnPMVSLevelSliderScroll( wxScrollEvent& event ) = 0;
		virtual void OnPMVSCellSizeSliderScroll( wxScrollEvent& event ) = 0;
		virtual void OnPMVSThresholdSliderScroll( wxScrollEvent& event ) = 0;
		virtual void OnPMVSWSizeSliderScroll( wxScrollEvent& event ) = 0;
		virtual void OnPMVSMinImageNumSliderScroll( wxScrollEvent& event ) = 0;
		virtual void OnMVEScaleSliderScroll( wxScrollEvent& event ) = 0;
		virtual void OnMVEFilterWidthSliderScroll( wxScrollEvent& event ) = 0;
		virtual void OnSMVSInputScaleSliderScroll( wxScrollEvent& event ) = 0;
		virtual void OnSMVSOutputScaleSliderScroll( wxScrollEvent& event ) = 0;
		virtual void OnSMVSSurfaceSmoothingFactorSliderScroll( wxScrollEvent& event ) = 0;
		
	
	public:
		wxString maxImage_; 
		
		Regard3DDensificationDialogBase( wxWindow* parent, wxWindowID id = ID_REGARD3DDENSIFICATIONDIALOG, const wxString& title = wxT("Densification"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( -1,-1 ), long style = wxDEFAULT_DIALOG_STYLE );
		~Regard3DDensificationDialogBase();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class Regard3DSurfaceDialogBase
///////////////////////////////////////////////////////////////////////////////
class Regard3DSurfaceDialogBase : public wxDialog 
{
	DECLARE_EVENT_TABLE()
	private:
		
		// Private event handlers
		void _wxFB_OnInitDialog( wxInitDialogEvent& event ){ OnInitDialog( event ); }
		void _wxFB_OnSurfaceGenerationMethodRadioBox( wxCommandEvent& event ){ OnSurfaceGenerationMethodRadioBox( event ); }
		void _wxFB_OnPoissonDepthSliderScroll( wxScrollEvent& event ){ OnPoissonDepthSliderScroll( event ); }
		void _wxFB_OnPoissonSamplesPerNodeSliderScroll( wxScrollEvent& event ){ OnPoissonSamplesPerNodeSliderScroll( event ); }
		void _wxFB_OnPoissonPointWeightSliderScroll( wxScrollEvent& event ){ OnPoissonPointWeightSliderScroll( event ); }
		void _wxFB_OnPoissonTrimThresholdSliderScroll( wxScrollEvent& event ){ OnPoissonTrimThresholdSliderScroll( event ); }
		void _wxFB_OnFSSRRefineOctreeLevelsSliderScroll( wxScrollEvent& event ){ OnFSSRRefineOctreeLevelsSliderScroll( event ); }
		void _wxFB_OnFSSRScaleFactorMultiplierSliderScroll( wxScrollEvent& event ){ OnFSSRScaleFactorMultiplierSliderScroll( event ); }
		void _wxFB_OnFSSRConfidenceThresholdSliderScroll( wxScrollEvent& event ){ OnFSSRConfidenceThresholdSliderScroll( event ); }
		void _wxFB_OnFSSRMinComponentSizeSliderScroll( wxScrollEvent& event ){ OnFSSRMinComponentSizeSliderScroll( event ); }
		void _wxFB_OnColorizationMethodRadioBox( wxCommandEvent& event ){ OnColorizationMethodRadioBox( event ); }
		void _wxFB_OnColVertNumberOfNeighboursSliderScroll( wxScrollEvent& event ){ OnColVertNumberOfNeighboursSliderScroll( event ); }
		
	
	protected:
		enum
		{
			ID_REGARD3DSURFACEDIALOG = 1000,
			ID_SURFACEPANEL,
			ID_SURFACEGENERATIONMETHODRADIOBOX,
			ID_POISSONPARAMSBOXSIZER,
			ID_POISSONDEPTHTEXTCTRL,
			ID_POISSONDEPTHSLIDER,
			ID_POISSONSAMPLESPERNODETEXTCTRL,
			ID_POISSONSAMPLESPERNODESLIDER,
			ID_POISSONPOINTWEIGHTTEXTCTRL,
			ID_POISSONPOINTWEIGHTSLIDER,
			ID_POISSONTRIMTHRESHOLDTEXTCTRL,
			ID_POISSONTRIMTHRESHOLDSLIDER,
			ID_FSSRPARAMSBOXSIZER,
			ID_FSSRREFINEOCTREELEVELSTEXTCTRL,
			ID_FSSRREFINEOCTREELEVELSSLIDER,
			ID_FSSRSCALEFACTORMULTIPLIERTEXTCTRL,
			ID_FSSRSCALEFACTORMULTIPLIERSLIDER,
			ID_FSSRCONFIDENCETHRESHOLDTEXTCTRL,
			ID_FSSRCONFIDENCETHRESHOLDSLIDER,
			ID_FSSRMINCOMPONENTSIZETEXTCTRL,
			ID_FSSRMINCOMPONENTSIZESLIDER,
			ID_PCOLORIZATIONMETHODRADIOBOX_,
			ID_COLVERTPARAMSBOXSIZER,
			ID_COLVERTNUMBEROFNEIGHBOURSTEXTCTRL,
			ID_COLVERTNUMBEROFNEIGHBOURSSLIDER,
			ID_TEXTURIZATIONPARAMSBOXSIZER,
			ID_TEXTOUTLIERREMOVALCHOICE,
			ID_TEXTGEOMVISTESTCHECKBOX,
			ID_TEXTGLOBALSEAMLEVCHECKBOX,
			ID_TEXTLOCALSEAMLEVCHECKBOX,
		};
		
		wxPanel* pSurfacePanel_;
		wxRadioBox* pSurfaceGenerationMethodRadioBox_;
		wxStaticText* m_staticText39;
		wxTextCtrl* pPoissonDepthTextCtrl_;
		wxSlider* pPoissonDepthSlider_;
		wxStaticText* m_staticText52;
		wxTextCtrl* pPoissonSamplesPerNodeTextCtrl_;
		wxSlider* pPoissonSamplesPerNodeSlider_;
		wxStaticText* m_staticText40;
		wxTextCtrl* pPoissonPointWeightTextCtrl_;
		wxSlider* pPoissonPointWeightSlider_;
		wxStaticText* m_staticText41;
		wxTextCtrl* pPoissonTrimThresholdTextCtrl_;
		wxSlider* pPoissonTrimThresholdSlider_;
		wxStaticBoxSizer* pFSSRParamsBoxSizer_;
		wxStaticText* m_staticText55;
		wxTextCtrl* pFSSRRefineOctreeLevelsTextCtrl_;
		wxSlider* pFSSRRefineOctreeLevelsSlider_;
		wxStaticText* m_staticText56;
		wxTextCtrl* pFSSRScaleFactorMultiplierTextCtrl_;
		wxSlider* pFSSRScaleFactorMultiplierSlider_;
		wxStaticText* m_staticText57;
		wxTextCtrl* pFSSRConfidenceThresholdTextCtrl_;
		wxSlider* pFSSRConfidenceThresholdSlider_;
		wxStaticText* m_staticText58;
		wxTextCtrl* pFSSRMinComponentSizeTextCtrl_;
		wxSlider* pFSSRMinComponentSizeSlider_;
		wxRadioBox* pColorizationMethodRadioBox_;
		wxStaticBoxSizer* pColVertParamsBoxSizer_;
		wxStaticText* m_staticText51;
		wxTextCtrl* pColVertNumberOfNeighboursTextCtrl_;
		wxSlider* pColVertNumberOfNeighboursSlider_;
		wxStaticBoxSizer* pTexturizationParamsBoxSizer_;
		wxStaticText* m_staticText42;
		wxChoice* pTextOutlierRemovalChoice_;
		wxStaticText* m_staticText43;
		wxCheckBox* pTextGeomVisTestCheckBox_;
		wxStaticText* m_staticText44;
		wxCheckBox* pTextGlobalSeamLevCheckBox_;
		wxStaticText* m_staticText45;
		wxCheckBox* pTextLocalSeamLevCheckBox_;
		
		wxStdDialogButtonSizer* m_sdbSizer7;
		wxButton* m_sdbSizer7OK;
		wxButton* m_sdbSizer7Cancel;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnInitDialog( wxInitDialogEvent& event ) = 0;
		virtual void OnSurfaceGenerationMethodRadioBox( wxCommandEvent& event ) = 0;
		virtual void OnPoissonDepthSliderScroll( wxScrollEvent& event ) = 0;
		virtual void OnPoissonSamplesPerNodeSliderScroll( wxScrollEvent& event ) = 0;
		virtual void OnPoissonPointWeightSliderScroll( wxScrollEvent& event ) = 0;
		virtual void OnPoissonTrimThresholdSliderScroll( wxScrollEvent& event ) = 0;
		virtual void OnFSSRRefineOctreeLevelsSliderScroll( wxScrollEvent& event ) = 0;
		virtual void OnFSSRScaleFactorMultiplierSliderScroll( wxScrollEvent& event ) = 0;
		virtual void OnFSSRConfidenceThresholdSliderScroll( wxScrollEvent& event ) = 0;
		virtual void OnFSSRMinComponentSizeSliderScroll( wxScrollEvent& event ) = 0;
		virtual void OnColorizationMethodRadioBox( wxCommandEvent& event ) = 0;
		virtual void OnColVertNumberOfNeighboursSliderScroll( wxScrollEvent& event ) = 0;
		
	
	public:
		
		Regard3DSurfaceDialogBase( wxWindow* parent, wxWindowID id = ID_REGARD3DSURFACEDIALOG, const wxString& title = wxT("Surface generation"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( -1,-1 ), long style = wxDEFAULT_DIALOG_STYLE );
		~Regard3DSurfaceDialogBase();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class Regard3DPropertiesDialogBase
///////////////////////////////////////////////////////////////////////////////
class Regard3DPropertiesDialogBase : public wxDialog 
{
	DECLARE_EVENT_TABLE()
	private:
		
		// Private event handlers
		void _wxFB_OnInitDialog( wxInitDialogEvent& event ){ OnInitDialog( event ); }
		
	
	protected:
		enum
		{
			ID_REGARD3DPROPERTIESDIALOG = 1000,
			ID_POPERTIESPANEL,
			ID_DEFAULTPROJECTPATHDIRPICKER,
			ID_MOUSEBUTTONRADIOBOX,
			ID_MOUSEWHEELRADIOBOX,
		};
		
		wxPanel* pPopertiesPanel_;
		
		wxStdDialogButtonSizer* m_sdbSizer8;
		wxButton* m_sdbSizer8OK;
		wxButton* m_sdbSizer8Cancel;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnInitDialog( wxInitDialogEvent& event ) = 0;
		
	
	public:
		wxDirPickerCtrl* pDefaultProjectPathDirPicker_;
		wxRadioBox* pMouseButtonRadioBox_;
		wxRadioBox* pMouseWheelRadioBox_;
		
		Regard3DPropertiesDialogBase( wxWindow* parent, wxWindowID id = ID_REGARD3DPROPERTIESDIALOG, const wxString& title = wxT("Regard 3D Properties"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( -1,-1 ), long style = wxDEFAULT_DIALOG_STYLE );
		~Regard3DPropertiesDialogBase();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class Regard3DUserCameraDBDialogBase
///////////////////////////////////////////////////////////////////////////////
class Regard3DUserCameraDBDialogBase : public wxDialog 
{
	DECLARE_EVENT_TABLE()
	private:
		
		// Private event handlers
		void _wxFB_OnInitDialog( wxInitDialogEvent& event ){ OnInitDialog( event ); }
		void _wxFB_OnDeleteSelectedEntryButtonClicked( wxCommandEvent& event ){ OnDeleteSelectedEntryButtonClicked( event ); }
		void _wxFB_OnDeleteAllEntriesButtonClicked( wxCommandEvent& event ){ OnDeleteAllEntriesButtonClicked( event ); }
		
	
	protected:
		enum
		{
			ID_REGARD3DUSERCAMERADBDIALOGBASE = 1000,
			ID_USERCAMERADBPANEL,
			ID_USERCAMERADBLISTCTRL,
			ID_DELETESELECTEDITEMBUTTON,
			ID_DELETEDBBUTTON,
		};
		
		wxPanel* pUserCameraDBPanel_;
		
		wxButton* pDeleteSelectedItemButton_;
		
		wxButton* pDeleteDBButton_;
		
		wxStdDialogButtonSizer* m_sdbSizer9;
		wxButton* m_sdbSizer9OK;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnInitDialog( wxInitDialogEvent& event ) = 0;
		virtual void OnDeleteSelectedEntryButtonClicked( wxCommandEvent& event ) = 0;
		virtual void OnDeleteAllEntriesButtonClicked( wxCommandEvent& event ) = 0;
		
	
	public:
		wxListCtrl* pUserCameraDBListCtrl_;
		
		Regard3DUserCameraDBDialogBase( wxWindow* parent, wxWindowID id = ID_REGARD3DUSERCAMERADBDIALOGBASE, const wxString& title = wxT("User-defined camera DB"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize, long style = wxDEFAULT_DIALOG_STYLE );
		~Regard3DUserCameraDBDialogBase();
	
};

#endif //__Regard3DMainFrameBase__
