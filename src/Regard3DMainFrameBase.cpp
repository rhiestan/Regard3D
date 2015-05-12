///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Sep  8 2010)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "CommonIncludes.h"

#include "Regard3DMainFrameBase.h"

///////////////////////////////////////////////////////////////////////////

BEGIN_EVENT_TABLE( Regard3DMainFrameBase, wxFrame )
	EVT_CLOSE( Regard3DMainFrameBase::_wxFB_OnMainFrameClose )
	EVT_IDLE( Regard3DMainFrameBase::_wxFB_OnMainFrameIdle )
	EVT_MENU( ID_NEWPROJECTMENUITEM, Regard3DMainFrameBase::_wxFB_OnNewProject )
	EVT_MENU( ID_OPENPROJECTMENUITEM, Regard3DMainFrameBase::_wxFB_OnOpenProject )
	EVT_MENU( ID_CLOSEPROJECTMENUITEM, Regard3DMainFrameBase::_wxFB_OnCloseProject )
	EVT_MENU( wxID_EXIT, Regard3DMainFrameBase::_wxFB_OnMainFrameExitMenuItem )
	EVT_MENU( ID_PROPERTIESMENUITEM, Regard3DMainFrameBase::_wxFB_OnPropertiesMenuItem )
	EVT_MENU( ID_VIEWCONSOLEOUTPUTFRAMEMENUITEM, Regard3DMainFrameBase::_wxFB_OnViewConsoleOutputFrameMenuItem )
	EVT_UPDATE_UI( ID_VIEWCONSOLEOUTPUTFRAMEMENUITEM, Regard3DMainFrameBase::_wxFB_OnViewConsoleOutputFrameUpdate )
	EVT_MENU( wxID_ABOUT, Regard3DMainFrameBase::_wxFB_OnAboutMenuItem )
	EVT_TREE_ITEM_ACTIVATED( ID_PROJECTTREECTRL, Regard3DMainFrameBase::_wxFB_OnProjectTreeItemActivated )
	EVT_TREE_ITEM_MENU( ID_PROJECTTREECTRL, Regard3DMainFrameBase::_wxFB_OnProjectTreeItemMenu )
	EVT_TREE_SEL_CHANGED( ID_PROJECTTREECTRL, Regard3DMainFrameBase::_wxFB_OnProjectTreeSelChanged )
	EVT_TREE_SEL_CHANGING( ID_PROJECTTREECTRL, Regard3DMainFrameBase::_wxFB_OnProjectTreeSelChanging )
	EVT_BUTTON( ID_PROJECTADDPICTURESETBUTTON, Regard3DMainFrameBase::_wxFB_OnProjectAddPictureSetButton )
	EVT_BUTTON( ID_PICTURESETCOMPUTEMATCHESBUTTON, Regard3DMainFrameBase::_wxFB_OnPictureSetComputeMatchesButton )
	EVT_BUTTON( ID_EDITPICTURESETBUTTON, Regard3DMainFrameBase::_wxFB_OnEditPictureSetButton )
	EVT_BUTTON( ID_CLONEPICTURESETBUTTON, Regard3DMainFrameBase::_wxFB_OnClonePictureSetButton )
	EVT_BUTTON( ID_PROJECTDELETEPICTURESETBUTTON, Regard3DMainFrameBase::_wxFB_OnProjectDeletePictureSetButton )
	EVT_BUTTON( ID_SHOWMATCHINGRESULTSBUTTON, Regard3DMainFrameBase::_wxFB_OnShowMatchingResultsButton )
	EVT_BUTTON( ID_TRIANGULATIONBUTTON, Regard3DMainFrameBase::_wxFB_OnTriangulationButton )
	EVT_BUTTON( ID_PROJECTDELETECOMPUTEMATCHESBUTTON, Regard3DMainFrameBase::_wxFB_OnProjectDeleteComputeMatchesButton )
	EVT_BUTTON( ID_CREATEDENSEPOINTCLOUDBUTTON, Regard3DMainFrameBase::_wxFB_OnCreateDensePointcloudButton )
	EVT_BUTTON( ID_SHOWTRIANGULATEDPOINTS, Regard3DMainFrameBase::_wxFB_OnShowTriangulatedPoints )
	EVT_BUTTON( ID_EXPORTTRIANGULATIONTOEXTERNALMVSBUTTON, Regard3DMainFrameBase::_wxFB_OnExportTriangulationToExternalMVSButton )
	EVT_BUTTON( ID_PROJECTDELETETRIANGULATIONBUTTON, Regard3DMainFrameBase::_wxFB_OnProjectDeleteTriangulationButton )
	EVT_BUTTON( ID_CREATESURFACEBUTTON, Regard3DMainFrameBase::_wxFB_OnCreateSurfaceButton )
	EVT_BUTTON( ID_SHOWDENSEPOINTCLOUDBUTTON, Regard3DMainFrameBase::_wxFB_OnShowDensePointCloudButton )
	EVT_BUTTON( ID_EXPORTDENSIFICATIONBUTTON, Regard3DMainFrameBase::_wxFB_OnExportDensificationButton )
	EVT_BUTTON( ID_EXPORTDENSIFICATIONTOMESHLABBUTTON, Regard3DMainFrameBase::_wxFB_OnExportDensificationToMeshLabButton )
	EVT_BUTTON( ID_PPROJECTDELETEDENSIFICATIONBUTTON_, Regard3DMainFrameBase::_wxFB_OnProjectDeleteDensificationButton )
	EVT_BUTTON( ID_SHOWSURFACEBUTTON, Regard3DMainFrameBase::_wxFB_OnShowSurfaceButton )
	EVT_BUTTON( ID_EXPORTSURFACEBUTTON, Regard3DMainFrameBase::_wxFB_OnExportSurfaceButton )
	EVT_BUTTON( ID_DELETESURFACEBUTTON, Regard3DMainFrameBase::_wxFB_OnDeleteSurfaceButton )
	EVT_CHECKBOX( ID_SHOWTRACKBALLCHECKBOX, Regard3DMainFrameBase::_wxFB_OnShowTrackballCheckBox )
	EVT_COMMAND_SCROLL( ID_POINTSIZESLIDER, Regard3DMainFrameBase::_wxFB_OnPointSizeScroll )
	EVT_CHECKBOX( ID_SHOWTEXTURECHECKBOX, Regard3DMainFrameBase::_wxFB_OnShowTextureCheckBox )
	EVT_CHECKBOX( ID_ENABLELIGHTINGCHECKBOX, Regard3DMainFrameBase::_wxFB_OnEnableLightingCheckBox )
	EVT_RADIOBOX( ID_POLYGONMODERADIOBOX, Regard3DMainFrameBase::_wxFB_OnPolygonModeRadioBox )
	EVT_RADIOBOX( ID_SHADINGMODELRADIOBOX, Regard3DMainFrameBase::_wxFB_OnShadingModelRadioBox )
	EVT_BUTTON( ID_RESETVIEWBUTTON, Regard3DMainFrameBase::_wxFB_OnResetViewButton )
END_EVENT_TABLE()

Regard3DMainFrameBase::Regard3DMainFrameBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxSize( 200,200 ), wxDefaultSize );
	
	pMainFrameMenuBar_ = new wxMenuBar( 0 );
	pFileMenu_ = new wxMenu();
	wxMenuItem* pNewProjectMenuItem_;
	pNewProjectMenuItem_ = new wxMenuItem( pFileMenu_, ID_NEWPROJECTMENUITEM, wxString( wxT("New Project...") ) , wxEmptyString, wxITEM_NORMAL );
	pFileMenu_->Append( pNewProjectMenuItem_ );
	
	wxMenuItem* pOpenProjectMenuItem_;
	pOpenProjectMenuItem_ = new wxMenuItem( pFileMenu_, ID_OPENPROJECTMENUITEM, wxString( wxT("Open Project...") ) , wxEmptyString, wxITEM_NORMAL );
	pFileMenu_->Append( pOpenProjectMenuItem_ );
	
	wxMenuItem* pCloseProjectMenuItem_;
	pCloseProjectMenuItem_ = new wxMenuItem( pFileMenu_, ID_CLOSEPROJECTMENUITEM, wxString( wxT("Close Project") ) , wxEmptyString, wxITEM_NORMAL );
	pFileMenu_->Append( pCloseProjectMenuItem_ );
	
	wxMenuItem* pExitMenuItem_;
	pExitMenuItem_ = new wxMenuItem( pFileMenu_, wxID_EXIT, wxString( wxT("Exit") ) , wxEmptyString, wxITEM_NORMAL );
	pFileMenu_->Append( pExitMenuItem_ );
	
	pMainFrameMenuBar_->Append( pFileMenu_, wxT("File") ); 
	
	pOptionsMenu_ = new wxMenu();
	wxMenuItem* pPropertiesMenuItem_;
	pPropertiesMenuItem_ = new wxMenuItem( pOptionsMenu_, ID_PROPERTIESMENUITEM, wxString( wxT("Properties...") ) , wxEmptyString, wxITEM_NORMAL );
	pOptionsMenu_->Append( pPropertiesMenuItem_ );
	
	pMainFrameMenuBar_->Append( pOptionsMenu_, wxT("Options") ); 
	
	pViewMenu_ = new wxMenu();
	pViewConsoleOutputFrameMenuItem_ = new wxMenuItem( pViewMenu_, ID_VIEWCONSOLEOUTPUTFRAMEMENUITEM, wxString( wxT("Console output") ) , wxEmptyString, wxITEM_CHECK );
	pViewMenu_->Append( pViewConsoleOutputFrameMenuItem_ );
	
	pMainFrameMenuBar_->Append( pViewMenu_, wxT("View") ); 
	
	pHelpMenu_ = new wxMenu();
	wxMenuItem* pAboutMenuItem_;
	pAboutMenuItem_ = new wxMenuItem( pHelpMenu_, wxID_ABOUT, wxString( wxT("About") ) , wxEmptyString, wxITEM_NORMAL );
	pHelpMenu_->Append( pAboutMenuItem_ );
	
	pMainFrameMenuBar_->Append( pHelpMenu_, wxT("Help") ); 
	
	this->SetMenuBar( pMainFrameMenuBar_ );
	
	pMainFrameToolBar_ = this->CreateToolBar( wxTB_HORIZONTAL, ID_MAINFRAMETOOLBAR ); 
	pMainFrameToolBar_->Realize();
	
	wxBoxSizer* pMainFrameBoxSizer_;
	pMainFrameBoxSizer_ = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer28;
	bSizer28 = new wxBoxSizer( wxHORIZONTAL );
	
	pMainSplitter_ = new wxSplitterWindow( this, ID_MAINSPLITTER, wxDefaultPosition, wxDefaultSize, wxSP_3D );
	pMainSplitter_->Connect( wxEVT_IDLE, wxIdleEventHandler( Regard3DMainFrameBase::pMainSplitter_OnIdle ), NULL, this );
	pMainSplitter_->SetMinimumPaneSize( 100 );
	
	pMainSplitterLeftPanel_ = new wxPanel( pMainSplitter_, ID_MAINSPLITTERLEFTPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	pMainSplitterLeftPanel_->SetMinSize( wxSize( 150,-1 ) );
	
	wxBoxSizer* bSizer29;
	bSizer29 = new wxBoxSizer( wxVERTICAL );
	
	pProjectSplitter_ = new wxSplitterWindow( pMainSplitterLeftPanel_, ID_PROJECTSPLITTER, wxDefaultPosition, wxDefaultSize, wxSP_3D );
	pProjectSplitter_->Connect( wxEVT_IDLE, wxIdleEventHandler( Regard3DMainFrameBase::pProjectSplitter_OnIdle ), NULL, this );
	pProjectSplitter_->SetMinimumPaneSize( 100 );
	
	m_panel15 = new wxPanel( pProjectSplitter_, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer31;
	bSizer31 = new wxBoxSizer( wxVERTICAL );
	
	pProjectTreeCtrl_ = new wxTreeCtrl( m_panel15, ID_PROJECTTREECTRL, wxDefaultPosition, wxDefaultSize, wxTR_DEFAULT_STYLE|wxTR_HAS_BUTTONS|wxTR_TWIST_BUTTONS );
	bSizer31->Add( pProjectTreeCtrl_, 1, wxALL|wxEXPAND, 3 );
	
	m_panel15->SetSizer( bSizer31 );
	m_panel15->Layout();
	bSizer31->Fit( m_panel15 );
	pProjectLowerScrolledWindow_ = new wxScrolledWindow( pProjectSplitter_, ID_PROJECTLOWERSCROLLEDWINDOW, wxDefaultPosition, wxDefaultSize, wxHSCROLL|wxVSCROLL );
	pProjectLowerScrolledWindow_->SetScrollRate( 5, 5 );
	pProjectLowerSizer_ = new wxBoxSizer( wxVERTICAL );
	
	pProjectProjectSizer_ = new wxBoxSizer( wxVERTICAL );
	
	wxFlexGridSizer* pProjectDetailsSizer_;
	pProjectDetailsSizer_ = new wxFlexGridSizer( 2, 2, 0, 0 );
	pProjectDetailsSizer_->AddGrowableCol( 1 );
	pProjectDetailsSizer_->SetFlexibleDirection( wxBOTH );
	pProjectDetailsSizer_->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText112 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Project name:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText112->Wrap( -1 );
	pProjectDetailsSizer_->Add( m_staticText112, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pProjectNameTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_PROJECTNAMETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pProjectDetailsSizer_->Add( pProjectNameTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText122 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Project path:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText122->Wrap( -1 );
	pProjectDetailsSizer_->Add( m_staticText122, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pProjectPathTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_PROJECTPATHTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pProjectDetailsSizer_->Add( pProjectPathTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	pProjectProjectSizer_->Add( pProjectDetailsSizer_, 1, wxEXPAND, 5 );
	
	pProjectAddPictureSetButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_PROJECTADDPICTURESETBUTTON, wxT("Add Picture Set..."), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectProjectSizer_->Add( pProjectAddPictureSetButton_, 0, wxALL, 3 );
	
	pProjectLowerSizer_->Add( pProjectProjectSizer_, 0, wxEXPAND, 3 );
	
	pProjectPictureSetSizer_ = new wxBoxSizer( wxVERTICAL );
	
	wxFlexGridSizer* pPictureSetDetailsSizer_;
	pPictureSetDetailsSizer_ = new wxFlexGridSizer( 3, 2, 0, 0 );
	pPictureSetDetailsSizer_->AddGrowableCol( 1 );
	pPictureSetDetailsSizer_->SetFlexibleDirection( wxBOTH );
	pPictureSetDetailsSizer_->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText113 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Picture set name:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText113->Wrap( -1 );
	pPictureSetDetailsSizer_->Add( m_staticText113, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pPictureSetNameTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_PICTURESETNAMETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pPictureSetDetailsSizer_->Add( pPictureSetNameTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText123 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Picture set id:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText123->Wrap( -1 );
	pPictureSetDetailsSizer_->Add( m_staticText123, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pPictureSetIdTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_PICTURESETIDTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pPictureSetDetailsSizer_->Add( pPictureSetIdTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText132 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Number of pictures:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText132->Wrap( -1 );
	pPictureSetDetailsSizer_->Add( m_staticText132, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pPictureSetSizeTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_PICTURESETSIZETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pPictureSetDetailsSizer_->Add( pPictureSetSizeTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	pProjectPictureSetSizer_->Add( pPictureSetDetailsSizer_, 1, wxEXPAND, 5 );
	
	pPictureSetComputeMatchesButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_PICTURESETCOMPUTEMATCHESBUTTON, wxT("Compute matches..."), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectPictureSetSizer_->Add( pPictureSetComputeMatchesButton_, 0, wxALL, 3 );
	
	pEditPictureSetButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_EDITPICTURESETBUTTON, wxT("Edit picture set..."), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectPictureSetSizer_->Add( pEditPictureSetButton_, 0, wxALL, 3 );
	
	pClonePictureSetButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_CLONEPICTURESETBUTTON, wxT("Clone picture set"), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectPictureSetSizer_->Add( pClonePictureSetButton_, 0, wxALL, 3 );
	
	pProjectDeletePictureSetButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_PROJECTDELETEPICTURESETBUTTON, wxT("Delete"), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectPictureSetSizer_->Add( pProjectDeletePictureSetButton_, 0, wxALL, 3 );
	
	pProjectLowerSizer_->Add( pProjectPictureSetSizer_, 0, wxEXPAND, 3 );
	
	pProjectComputeMatchesSizer_ = new wxBoxSizer( wxVERTICAL );
	
	wxFlexGridSizer* pComputeMatchesDetailsSizer_;
	pComputeMatchesDetailsSizer_ = new wxFlexGridSizer( 4, 2, 0, 0 );
	pComputeMatchesDetailsSizer_->AddGrowableCol( 1 );
	pComputeMatchesDetailsSizer_->SetFlexibleDirection( wxBOTH );
	pComputeMatchesDetailsSizer_->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText114 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Matches Id:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText114->Wrap( -1 );
	pComputeMatchesDetailsSizer_->Add( m_staticText114, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pComputeMatchesIdTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_COMPUTEMATCHESIDTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pComputeMatchesDetailsSizer_->Add( pComputeMatchesIdTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText124 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Parameters:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText124->Wrap( -1 );
	pComputeMatchesDetailsSizer_->Add( m_staticText124, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pComputeMatchesParametersTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_COMPUTEMATCHESPARAMETERSTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pComputeMatchesDetailsSizer_->Add( pComputeMatchesParametersTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText133 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Keypoints:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText133->Wrap( -1 );
	pComputeMatchesDetailsSizer_->Add( m_staticText133, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pComputeMatchesResultsTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_COMPUTEMATCHESRESULTSTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pComputeMatchesResultsTextCtrl_->SetToolTip( wxT("Keypoints per image (min/max/avg/median)") );
	
	pComputeMatchesDetailsSizer_->Add( pComputeMatchesResultsTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText45 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Running time:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText45->Wrap( -1 );
	pComputeMatchesDetailsSizer_->Add( m_staticText45, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pComputeMatchesTimeTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_COMPUTEMATCHESTIMETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pComputeMatchesDetailsSizer_->Add( pComputeMatchesTimeTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	pProjectComputeMatchesSizer_->Add( pComputeMatchesDetailsSizer_, 1, wxEXPAND, 5 );
	
	pShowMatchingResultsButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_SHOWMATCHINGRESULTSBUTTON, wxT("Show matching results..."), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectComputeMatchesSizer_->Add( pShowMatchingResultsButton_, 0, wxALL, 3 );
	
	pTriangulationButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_TRIANGULATIONBUTTON, wxT("Triangulation..."), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectComputeMatchesSizer_->Add( pTriangulationButton_, 0, wxALL, 3 );
	
	pProjectDeleteComputeMatchesButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_PROJECTDELETECOMPUTEMATCHESBUTTON, wxT("Delete"), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectComputeMatchesSizer_->Add( pProjectDeleteComputeMatchesButton_, 0, wxALL, 3 );
	
	pProjectLowerSizer_->Add( pProjectComputeMatchesSizer_, 0, wxEXPAND, 3 );
	
	pProjectTriangulationSizer_ = new wxBoxSizer( wxVERTICAL );
	
	wxFlexGridSizer* pTriangulationDetailsSizer_;
	pTriangulationDetailsSizer_ = new wxFlexGridSizer( 6, 2, 0, 0 );
	pTriangulationDetailsSizer_->AddGrowableCol( 1 );
	pTriangulationDetailsSizer_->SetFlexibleDirection( wxBOTH );
	pTriangulationDetailsSizer_->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText111 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Triangulation Id:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText111->Wrap( -1 );
	pTriangulationDetailsSizer_->Add( m_staticText111, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pTriangulationIdTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_TRIANGULATIONIDTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pTriangulationDetailsSizer_->Add( pTriangulationIdTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText121 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Parameters:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText121->Wrap( -1 );
	pTriangulationDetailsSizer_->Add( m_staticText121, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pTriangulationParametersTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_TRIANGULATIONPARAMETERSTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pTriangulationDetailsSizer_->Add( pTriangulationParametersTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText131 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Cameras:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText131->Wrap( -1 );
	pTriangulationDetailsSizer_->Add( m_staticText131, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pTriangulationCamerasTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_TRIANGULATIONCAMERASTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pTriangulationCamerasTextCtrl_->SetToolTip( wxT("Cameras calibrated/total") );
	
	pTriangulationDetailsSizer_->Add( pTriangulationCamerasTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText46 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Number of points:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText46->Wrap( -1 );
	pTriangulationDetailsSizer_->Add( m_staticText46, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pTriangulationNumberPointsTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_TRIANGULATIONNUMBERPOINTSTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pTriangulationDetailsSizer_->Add( pTriangulationNumberPointsTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText47 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Residual errors:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText47->Wrap( -1 );
	pTriangulationDetailsSizer_->Add( m_staticText47, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pTriangulationResidualErrorsTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_TRIANGULATIONRESIDUALERRORSTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pTriangulationDetailsSizer_->Add( pTriangulationResidualErrorsTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText48 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Running time:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText48->Wrap( -1 );
	pTriangulationDetailsSizer_->Add( m_staticText48, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pTriangulationTimeTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_TRIANGULATIONTIMETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pTriangulationDetailsSizer_->Add( pTriangulationTimeTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	pProjectTriangulationSizer_->Add( pTriangulationDetailsSizer_, 1, wxEXPAND, 5 );
	
	pCreateDensePointcloudButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_CREATEDENSEPOINTCLOUDBUTTON, wxT("Create dense pointcloud..."), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectTriangulationSizer_->Add( pCreateDensePointcloudButton_, 0, wxALL, 3 );
	
	pShowTriangulatedPoints_ = new wxButton( pProjectLowerScrolledWindow_, ID_SHOWTRIANGULATEDPOINTS, wxT("Show triangulated points"), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectTriangulationSizer_->Add( pShowTriangulatedPoints_, 0, wxALL, 3 );
	
	pExportTriangulationToExternalMVSButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_EXPORTTRIANGULATIONTOEXTERNALMVSBUTTON, wxT("Export to external MVS"), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectTriangulationSizer_->Add( pExportTriangulationToExternalMVSButton_, 0, wxALL, 3 );
	
	pProjectDeleteTriangulationButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_PROJECTDELETETRIANGULATIONBUTTON, wxT("Delete"), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectTriangulationSizer_->Add( pProjectDeleteTriangulationButton_, 0, wxALL, 3 );
	
	pProjectLowerSizer_->Add( pProjectTriangulationSizer_, 0, wxEXPAND, 3 );
	
	pProjectDensificationSizer_ = new wxBoxSizer( wxVERTICAL );
	
	wxFlexGridSizer* pDensificationDetailsSizer_;
	pDensificationDetailsSizer_ = new wxFlexGridSizer( 4, 2, 0, 0 );
	pDensificationDetailsSizer_->AddGrowableCol( 1 );
	pDensificationDetailsSizer_->SetFlexibleDirection( wxBOTH );
	pDensificationDetailsSizer_->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText35 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Densification Id:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText35->Wrap( -1 );
	pDensificationDetailsSizer_->Add( m_staticText35, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pDensificationIdTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_DENSIFICATIONIDTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pDensificationDetailsSizer_->Add( pDensificationIdTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText36 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Densification type:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText36->Wrap( -1 );
	pDensificationDetailsSizer_->Add( m_staticText36, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pDensificationTypeTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_DENSIFICATIONTYPETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pDensificationDetailsSizer_->Add( pDensificationTypeTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText37 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Parameters:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText37->Wrap( -1 );
	pDensificationDetailsSizer_->Add( m_staticText37, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pDensificationParametersTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_DENSIFICATIONPARAMETERSTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pDensificationDetailsSizer_->Add( pDensificationParametersTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText38 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Running time:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText38->Wrap( -1 );
	pDensificationDetailsSizer_->Add( m_staticText38, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pDensificationRunningTimeTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_DENSIFICATIONRUNNINGTIMETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pDensificationDetailsSizer_->Add( pDensificationRunningTimeTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	pProjectDensificationSizer_->Add( pDensificationDetailsSizer_, 1, wxEXPAND, 5 );
	
	pCreateSurfaceButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_CREATESURFACEBUTTON, wxT("Create Surface..."), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectDensificationSizer_->Add( pCreateSurfaceButton_, 0, wxALL, 3 );
	
	pShowDensePointCloudButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_SHOWDENSEPOINTCLOUDBUTTON, wxT("Show point cloud"), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectDensificationSizer_->Add( pShowDensePointCloudButton_, 0, wxALL, 3 );
	
	pExportDensificationButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_EXPORTDENSIFICATIONBUTTON, wxT("Export point cloud"), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectDensificationSizer_->Add( pExportDensificationButton_, 0, wxALL, 3 );
	
	pExportDensificationToMeshLabButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_EXPORTDENSIFICATIONTOMESHLABBUTTON, wxT("Export scene to MeshLab"), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectDensificationSizer_->Add( pExportDensificationToMeshLabButton_, 0, wxALL, 3 );
	
	pProjectDeleteDensificationButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_PPROJECTDELETEDENSIFICATIONBUTTON_, wxT("Delete"), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectDensificationSizer_->Add( pProjectDeleteDensificationButton_, 0, wxALL, 3 );
	
	pProjectLowerSizer_->Add( pProjectDensificationSizer_, 0, wxEXPAND, 5 );
	
	pProjectSurfaceSizer_ = new wxBoxSizer( wxVERTICAL );
	
	wxFlexGridSizer* pSurfaceDetailsSizer_;
	pSurfaceDetailsSizer_ = new wxFlexGridSizer( 6, 2, 0, 0 );
	pSurfaceDetailsSizer_->AddGrowableCol( 1 );
	pSurfaceDetailsSizer_->SetFlexibleDirection( wxBOTH );
	pSurfaceDetailsSizer_->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText461 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Surface Id:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText461->Wrap( -1 );
	pSurfaceDetailsSizer_->Add( m_staticText461, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pSurfaceIdTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_SURFACEIDTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pSurfaceDetailsSizer_->Add( pSurfaceIdTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText471 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Surface type:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText471->Wrap( -1 );
	pSurfaceDetailsSizer_->Add( m_staticText471, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pSurfaceTypeTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_SURFACETYPETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pSurfaceDetailsSizer_->Add( pSurfaceTypeTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText481 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Surface parameters:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText481->Wrap( -1 );
	pSurfaceDetailsSizer_->Add( m_staticText481, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pSurfaceParametersTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_SURFACEPARAMETERSTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pSurfaceDetailsSizer_->Add( pSurfaceParametersTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText49 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Colorization type:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText49->Wrap( -1 );
	pSurfaceDetailsSizer_->Add( m_staticText49, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pSurfaceColorizationTypeTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_SURFACECOLORIZATIONTYPETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pSurfaceDetailsSizer_->Add( pSurfaceColorizationTypeTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText50 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Colorization params:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText50->Wrap( -1 );
	pSurfaceDetailsSizer_->Add( m_staticText50, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pSurfaceColorizationParametersTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_SURFACECOLORIZATIONPARAMETERSTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pSurfaceDetailsSizer_->Add( pSurfaceColorizationParametersTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText51 = new wxStaticText( pProjectLowerScrolledWindow_, wxID_ANY, wxT("Running time:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText51->Wrap( -1 );
	pSurfaceDetailsSizer_->Add( m_staticText51, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pSurfaceRunningTimeTextCtrl_ = new wxTextCtrl( pProjectLowerScrolledWindow_, ID_SURFACERUNNINGTIMETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pSurfaceDetailsSizer_->Add( pSurfaceRunningTimeTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	pProjectSurfaceSizer_->Add( pSurfaceDetailsSizer_, 0, wxEXPAND, 3 );
	
	pShowSurfaceButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_SHOWSURFACEBUTTON, wxT("Show surface"), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectSurfaceSizer_->Add( pShowSurfaceButton_, 0, wxALL, 3 );
	
	pExportSurfaceButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_EXPORTSURFACEBUTTON, wxT("Export surface"), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectSurfaceSizer_->Add( pExportSurfaceButton_, 0, wxALL, 3 );
	
	pDeleteSurfaceButton_ = new wxButton( pProjectLowerScrolledWindow_, ID_DELETESURFACEBUTTON, wxT("Delete"), wxDefaultPosition, wxDefaultSize, 0 );
	pProjectSurfaceSizer_->Add( pDeleteSurfaceButton_, 0, wxALL, 3 );
	
	pProjectLowerSizer_->Add( pProjectSurfaceSizer_, 0, wxEXPAND, 3 );
	
	pProjectLowerScrolledWindow_->SetSizer( pProjectLowerSizer_ );
	pProjectLowerScrolledWindow_->Layout();
	pProjectLowerSizer_->Fit( pProjectLowerScrolledWindow_ );
	pProjectSplitter_->SplitHorizontally( m_panel15, pProjectLowerScrolledWindow_, 0 );
	bSizer29->Add( pProjectSplitter_, 1, wxEXPAND, 5 );
	
	pMainSplitterLeftPanel_->SetSizer( bSizer29 );
	pMainSplitterLeftPanel_->Layout();
	bSizer29->Fit( pMainSplitterLeftPanel_ );
	pMainSplitterRightPanel_ = new wxPanel( pMainSplitter_, ID_MAINSPLITTERRIGHTPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer30;
	bSizer30 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer19;
	bSizer19 = new wxBoxSizer( wxHORIZONTAL );
	
	pOSGGLCanvas_ = new OSGGLCanvas(pMainSplitterRightPanel_, ID_OSGGLCANVAS);
	bSizer19->Add( pOSGGLCanvas_, 1, wxALL|wxEXPAND, 3 );
	
	wxStaticBoxSizer* sbSizer10;
	sbSizer10 = new wxStaticBoxSizer( new wxStaticBox( pMainSplitterRightPanel_, wxID_ANY, wxT("3D View controls") ), wxVERTICAL );
	
	pShowTrackballCheckBox_ = new wxCheckBox( pMainSplitterRightPanel_, ID_SHOWTRACKBALLCHECKBOX, wxT("Show trackball"), wxDefaultPosition, wxDefaultSize, 0 );
	pShowTrackballCheckBox_->SetValue(true); 
	sbSizer10->Add( pShowTrackballCheckBox_, 0, wxALL, 3 );
	
	wxStaticBoxSizer* sbSizer11;
	sbSizer11 = new wxStaticBoxSizer( new wxStaticBox( pMainSplitterRightPanel_, wxID_ANY, wxT("Point size") ), wxVERTICAL );
	
	pPointSizeSlider_ = new wxSlider( pMainSplitterRightPanel_, ID_POINTSIZESLIDER, 1, 0, 10, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	sbSizer11->Add( pPointSizeSlider_, 0, wxALL|wxEXPAND, 3 );
	
	pPointSizeTextCtrl_ = new wxTextCtrl( pMainSplitterRightPanel_, ID_POINTSIZETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	sbSizer11->Add( pPointSizeTextCtrl_, 0, wxALL|wxEXPAND, 3 );
	
	sbSizer10->Add( sbSizer11, 0, wxEXPAND, 5 );
	
	pShowTextureCheckBox_ = new wxCheckBox( pMainSplitterRightPanel_, ID_SHOWTEXTURECHECKBOX, wxT("Show texture"), wxDefaultPosition, wxDefaultSize, 0 );
	pShowTextureCheckBox_->SetValue(true); 
	sbSizer10->Add( pShowTextureCheckBox_, 0, wxALL, 3 );
	
	pEnableLightingCheckBox_ = new wxCheckBox( pMainSplitterRightPanel_, ID_ENABLELIGHTINGCHECKBOX, wxT("Enable lighting"), wxDefaultPosition, wxDefaultSize, 0 );
	pEnableLightingCheckBox_->SetValue(true); 
	sbSizer10->Add( pEnableLightingCheckBox_, 0, wxALL, 3 );
	
	wxString pPolygonModeRadioBox_Choices[] = { wxT("Fill"), wxT("Line"), wxT("Point") };
	int pPolygonModeRadioBox_NChoices = sizeof( pPolygonModeRadioBox_Choices ) / sizeof( wxString );
	pPolygonModeRadioBox_ = new wxRadioBox( pMainSplitterRightPanel_, ID_POLYGONMODERADIOBOX, wxT("Polygon mode"), wxDefaultPosition, wxDefaultSize, pPolygonModeRadioBox_NChoices, pPolygonModeRadioBox_Choices, 1, wxRA_SPECIFY_COLS );
	pPolygonModeRadioBox_->SetSelection( 0 );
	sbSizer10->Add( pPolygonModeRadioBox_, 0, wxALL|wxEXPAND, 3 );
	
	wxString pShadingModelRadioBox_Choices[] = { wxT("Smooth"), wxT("Flat") };
	int pShadingModelRadioBox_NChoices = sizeof( pShadingModelRadioBox_Choices ) / sizeof( wxString );
	pShadingModelRadioBox_ = new wxRadioBox( pMainSplitterRightPanel_, ID_SHADINGMODELRADIOBOX, wxT("Shading model"), wxDefaultPosition, wxDefaultSize, pShadingModelRadioBox_NChoices, pShadingModelRadioBox_Choices, 1, wxRA_SPECIFY_COLS );
	pShadingModelRadioBox_->SetSelection( 0 );
	sbSizer10->Add( pShadingModelRadioBox_, 0, wxALL|wxEXPAND, 3 );
	
	pResetViewButton_ = new wxButton( pMainSplitterRightPanel_, ID_RESETVIEWBUTTON, wxT("Reset orientation"), wxDefaultPosition, wxDefaultSize, 0 );
	sbSizer10->Add( pResetViewButton_, 0, wxALL|wxEXPAND, 3 );
	
	bSizer19->Add( sbSizer10, 0, wxALL|wxEXPAND, 3 );
	
	bSizer30->Add( bSizer19, 1, wxEXPAND, 5 );
	
	pMainSplitterRightPanel_->SetSizer( bSizer30 );
	pMainSplitterRightPanel_->Layout();
	bSizer30->Fit( pMainSplitterRightPanel_ );
	pMainSplitter_->SplitVertically( pMainSplitterLeftPanel_, pMainSplitterRightPanel_, 250 );
	bSizer28->Add( pMainSplitter_, 1, wxEXPAND, 3 );
	
	pMainFrameBoxSizer_->Add( bSizer28, 1, wxEXPAND, 5 );
	
	this->SetSizer( pMainFrameBoxSizer_ );
	this->Layout();
	pMainStatusBar_ = this->CreateStatusBar( 1, wxST_SIZEGRIP, ID_MAINSTATUSBAR );
	
	this->Centre( wxBOTH );
}

Regard3DMainFrameBase::~Regard3DMainFrameBase()
{
}

BEGIN_EVENT_TABLE( NewProjectDialogBase, wxDialog )
	EVT_RADIOBUTTON( ID_USEDEFAULTPROJECTPATHRADIOBUTTON, NewProjectDialogBase::_wxFB_OnUseDefaultProjectPathRadioButton )
	EVT_RADIOBUTTON( ID_SETPROJECTPATHRADIOBUTTON, NewProjectDialogBase::_wxFB_OnSetProjectPathRadioButton )
	EVT_DIRPICKER_CHANGED( ID_PROJECTPATHDIRPICKER, NewProjectDialogBase::_wxFB_OnProjectPathChanged )
	EVT_TEXT( ID_PROJECTNAMETEXTCTRL, NewProjectDialogBase::_wxFB_OnProjectNameText )
	EVT_BUTTON( wxID_OK, NewProjectDialogBase::_wxFB_OnOK )
END_EVENT_TABLE()

NewProjectDialogBase::NewProjectDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer13;
	bSizer13 = new wxBoxSizer( wxVERTICAL );
	
	m_panel7 = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer14;
	bSizer14 = new wxBoxSizer( wxVERTICAL );
	
	wxStaticBoxSizer* sbSizer7;
	sbSizer7 = new wxStaticBoxSizer( new wxStaticBox( m_panel7, wxID_ANY, wxT("Project path") ), wxVERTICAL );
	
	pUseDefaultProjectPathRadioButton_ = new wxRadioButton( m_panel7, ID_USEDEFAULTPROJECTPATHRADIOBUTTON, wxT("Use default project path"), wxDefaultPosition, wxDefaultSize, 0 );
	pUseDefaultProjectPathRadioButton_->SetValue( true ); 
	sbSizer7->Add( pUseDefaultProjectPathRadioButton_, 0, wxALL, 3 );
	
	pSetProjectPathRadioButton_ = new wxRadioButton( m_panel7, ID_SETPROJECTPATHRADIOBUTTON, wxT("Set project path:"), wxDefaultPosition, wxDefaultSize, 0 );
	sbSizer7->Add( pSetProjectPathRadioButton_, 0, wxALL, 3 );
	
	pProjectPathDirPicker_ = new wxDirPickerCtrl( m_panel7, ID_PROJECTPATHDIRPICKER, wxEmptyString, wxT("Select a folder"), wxDefaultPosition, wxDefaultSize, wxDIRP_DEFAULT_STYLE );
	sbSizer7->Add( pProjectPathDirPicker_, 1, wxALL|wxEXPAND, 5 );
	
	bSizer14->Add( sbSizer7, 0, wxALL|wxEXPAND, 3 );
	
	wxStaticBoxSizer* sbSizer8;
	sbSizer8 = new wxStaticBoxSizer( new wxStaticBox( m_panel7, wxID_ANY, wxT("Project name") ), wxVERTICAL );
	
	wxFlexGridSizer* fgSizer2;
	fgSizer2 = new wxFlexGridSizer( 2, 2, 0, 0 );
	fgSizer2->AddGrowableCol( 1 );
	fgSizer2->SetFlexibleDirection( wxBOTH );
	fgSizer2->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText6 = new wxStaticText( m_panel7, wxID_ANY, wxT("Project name:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText6->Wrap( -1 );
	fgSizer2->Add( m_staticText6, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pProjectNameTextCtrl_ = new wxTextCtrl( m_panel7, ID_PROJECTNAMETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	pProjectNameTextCtrl_->SetValidator( wxTextValidator( wxFILTER_NONE, &projectName_ ) );
	
	fgSizer2->Add( pProjectNameTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText7 = new wxStaticText( m_panel7, wxID_ANY, wxT("Project file:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText7->Wrap( -1 );
	fgSizer2->Add( m_staticText7, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pProjectFilenameTextCtrl_ = new wxTextCtrl( m_panel7, ID_PROJECTFILENAMETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer2->Add( pProjectFilenameTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	sbSizer8->Add( fgSizer2, 1, wxEXPAND, 5 );
	
	bSizer14->Add( sbSizer8, 0, wxALL|wxEXPAND, 3 );
	
	
	bSizer14->Add( 0, 0, 1, wxEXPAND, 5 );
	
	m_sdbSizer1 = new wxStdDialogButtonSizer();
	m_sdbSizer1OK = new wxButton( m_panel7, wxID_OK );
	m_sdbSizer1->AddButton( m_sdbSizer1OK );
	m_sdbSizer1Cancel = new wxButton( m_panel7, wxID_CANCEL );
	m_sdbSizer1->AddButton( m_sdbSizer1Cancel );
	m_sdbSizer1->Realize();
	bSizer14->Add( m_sdbSizer1, 0, wxALL|wxEXPAND, 3 );
	
	m_panel7->SetSizer( bSizer14 );
	m_panel7->Layout();
	bSizer14->Fit( m_panel7 );
	bSizer13->Add( m_panel7, 1, wxEXPAND, 0 );
	
	this->SetSizer( bSizer13 );
	this->Layout();
	
	this->Centre( wxBOTH );
}

NewProjectDialogBase::~NewProjectDialogBase()
{
}

BEGIN_EVENT_TABLE( Regard3DConsoleOutputFrameBase, wxFrame )
	EVT_CLOSE( Regard3DConsoleOutputFrameBase::_wxFB_OnConsoleOutputClose )
	EVT_BUTTON( ID_CLEARCONSOLEOUTPUTBUTTON, Regard3DConsoleOutputFrameBase::_wxFB_OnClearConsoleOutputButton )
END_EVENT_TABLE()

Regard3DConsoleOutputFrameBase::Regard3DConsoleOutputFrameBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer14;
	bSizer14 = new wxBoxSizer( wxVERTICAL );
	
	m_panel7 = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer54;
	bSizer54 = new wxBoxSizer( wxVERTICAL );
	
	pConsoleOutputTextCtrl_ = new wxTextCtrl( m_panel7, ID_CONSOLEOUTPUTTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxHSCROLL|wxTE_MULTILINE|wxTE_READONLY );
	pConsoleOutputTextCtrl_->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 76, 90, 90, false, wxEmptyString ) );
	
	bSizer54->Add( pConsoleOutputTextCtrl_, 1, wxALL|wxEXPAND, 3 );
	
	wxBoxSizer* bSizer15;
	bSizer15 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer15->Add( 0, 0, 1, wxEXPAND, 5 );
	
	pClearConsoleOutputButton_ = new wxButton( m_panel7, ID_CLEARCONSOLEOUTPUTBUTTON, wxT("Clear"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer15->Add( pClearConsoleOutputButton_, 0, wxALL, 3 );
	
	bSizer54->Add( bSizer15, 0, wxEXPAND, 5 );
	
	m_panel7->SetSizer( bSizer54 );
	m_panel7->Layout();
	bSizer54->Fit( m_panel7 );
	bSizer14->Add( m_panel7, 1, wxEXPAND, 5 );
	
	this->SetSizer( bSizer14 );
	this->Layout();
	
	this->Centre( wxBOTH );
}

Regard3DConsoleOutputFrameBase::~Regard3DConsoleOutputFrameBase()
{
}

BEGIN_EVENT_TABLE( Regard3DProgressDialogBase, wxDialog )
	EVT_BUTTON( ID_SHOWOUTPUTWINDOWBUTTON, Regard3DProgressDialogBase::_wxFB_OnShowOutputWindowButton )
END_EVENT_TABLE()

Regard3DProgressDialogBase::Regard3DProgressDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxSize( 400,-1 ), wxDefaultSize );
	
	wxBoxSizer* bSizer16;
	bSizer16 = new wxBoxSizer( wxVERTICAL );
	
	m_panel8 = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer17;
	bSizer17 = new wxBoxSizer( wxVERTICAL );
	
	wxFlexGridSizer* fgSizer3;
	fgSizer3 = new wxFlexGridSizer( 2, 2, 0, 0 );
	fgSizer3->AddGrowableCol( 1 );
	fgSizer3->SetFlexibleDirection( wxBOTH );
	fgSizer3->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	pProgressText_ = new wxStaticText( m_panel8, ID_PROGRESSTEXT, wxT("Status:"), wxDefaultPosition, wxDefaultSize, 0 );
	pProgressText_->Wrap( -1 );
	fgSizer3->Add( pProgressText_, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pProgressStatusTextCtrl_ = new wxTextCtrl( m_panel8, ID_PROGRESSSTATUSTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer3->Add( pProgressStatusTextCtrl_, 0, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText9 = new wxStaticText( m_panel8, wxID_ANY, wxT("Elapsed time:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText9->Wrap( -1 );
	fgSizer3->Add( m_staticText9, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pElapsedTimeTextCtrl_ = new wxTextCtrl( m_panel8, ID_ELAPSEDTIMETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer3->Add( pElapsedTimeTextCtrl_, 0, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	bSizer17->Add( fgSizer3, 0, wxALL|wxEXPAND, 12 );
	
	pProgressGauge_ = new wxGauge( m_panel8, ID_PROGRESSGAUGE, 100, wxDefaultPosition, wxSize( -1,15 ), wxGA_HORIZONTAL|wxGA_SMOOTH );
	bSizer17->Add( pProgressGauge_, 0, wxALL|wxEXPAND, 12 );
	
	wxBoxSizer* bSizer18;
	bSizer18 = new wxBoxSizer( wxHORIZONTAL );
	
	pShowOutputWindowButton_ = new wxButton( m_panel8, ID_SHOWOUTPUTWINDOWBUTTON, wxT("Show output window"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer18->Add( pShowOutputWindowButton_, 0, wxALL, 12 );
	
	bSizer17->Add( bSizer18, 0, wxEXPAND, 5 );
	
	m_panel8->SetSizer( bSizer17 );
	m_panel8->Layout();
	bSizer17->Fit( m_panel8 );
	bSizer16->Add( m_panel8, 1, wxEXPAND, 0 );
	
	this->SetSizer( bSizer16 );
	this->Layout();
	
	this->Centre( wxBOTH );
}

Regard3DProgressDialogBase::~Regard3DProgressDialogBase()
{
}

BEGIN_EVENT_TABLE( Regard3DImagePreviewDialogBase, wxDialog )
	EVT_CLOSE( Regard3DImagePreviewDialogBase::_wxFB_OnClose )
	EVT_INIT_DIALOG( Regard3DImagePreviewDialogBase::_wxFB_OnInitDialog )
	EVT_CHOICE( ID_KEYPOINTTYPECHOICE, Regard3DImagePreviewDialogBase::_wxFB_OnKeypointTypeChoice )
	EVT_CHECKBOX( ID_SHOWSINGLEKEYPOINTSCHECKBOX, Regard3DImagePreviewDialogBase::_wxFB_OnShowSingleKeypointsCheckBox )
	EVT_CHOICE( ID_MATCHESCHOICE, Regard3DImagePreviewDialogBase::_wxFB_OnMatchesChoice )
	EVT_CHECKBOX( ID_ENABLETRACKFILTERCHECKBOX, Regard3DImagePreviewDialogBase::_wxFB_OnEnableTrackFilter )
	EVT_BUTTON( ID_ZOOMINBUTTON, Regard3DImagePreviewDialogBase::_wxFB_OnZoomInButton )
	EVT_BUTTON( ID_ZOOMOUTBUTTON, Regard3DImagePreviewDialogBase::_wxFB_OnZoomOutButton )
	EVT_BUTTON( ID_CLOSEBUTTON, Regard3DImagePreviewDialogBase::_wxFB_OnCloseButton )
END_EVENT_TABLE()

Regard3DImagePreviewDialogBase::Regard3DImagePreviewDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxSize( 400,200 ), wxDefaultSize );
	
	wxBoxSizer* bSizer21;
	bSizer21 = new wxBoxSizer( wxVERTICAL );
	
	pImagePreviewDialogPanel_ = new wxPanel( this, ID_IMAGEPREVIEWDIALOGPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer22;
	bSizer22 = new wxBoxSizer( wxVERTICAL );
	
	pImagePanel_ = new ImagePanel(pImagePreviewDialogPanel_);
	bSizer22->Add( pImagePanel_, 1, wxALL|wxEXPAND, 3 );
	
	wxBoxSizer* bSizer23;
	bSizer23 = new wxBoxSizer( wxHORIZONTAL );
	
	wxArrayString pKeypointTypeChoice_Choices;
	pKeypointTypeChoice_ = new wxChoice( pImagePreviewDialogPanel_, ID_KEYPOINTTYPECHOICE, wxDefaultPosition, wxDefaultSize, pKeypointTypeChoice_Choices, 0 );
	pKeypointTypeChoice_->SetSelection( 0 );
	bSizer23->Add( pKeypointTypeChoice_, 0, wxALL, 3 );
	
	pShowSingleKeypointsCheckBox_ = new wxCheckBox( pImagePreviewDialogPanel_, ID_SHOWSINGLEKEYPOINTSCHECKBOX, wxT("Show Single Keypoints"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer23->Add( pShowSingleKeypointsCheckBox_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	wxArrayString pMatchesChoice_Choices;
	pMatchesChoice_ = new wxChoice( pImagePreviewDialogPanel_, ID_MATCHESCHOICE, wxDefaultPosition, wxDefaultSize, pMatchesChoice_Choices, 0 );
	pMatchesChoice_->SetSelection( 0 );
	bSizer23->Add( pMatchesChoice_, 0, wxALL, 3 );
	
	pEnableTrackFilterCheckBox_ = new wxCheckBox( pImagePreviewDialogPanel_, ID_ENABLETRACKFILTERCHECKBOX, wxT("Enable Track Filter"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer23->Add( pEnableTrackFilterCheckBox_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	
	bSizer23->Add( 0, 0, 1, wxEXPAND, 5 );
	
	m_staticText10 = new wxStaticText( pImagePreviewDialogPanel_, wxID_ANY, wxT("Zoom:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText10->Wrap( -1 );
	bSizer23->Add( m_staticText10, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pZoomFactorTextCtrl_ = new wxTextCtrl( pImagePreviewDialogPanel_, ID_ZOOMFACTORTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	bSizer23->Add( pZoomFactorTextCtrl_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pZoomInButton_ = new wxButton( pImagePreviewDialogPanel_, ID_ZOOMINBUTTON, wxT("Zoom in"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer23->Add( pZoomInButton_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pZoomOutButton_ = new wxButton( pImagePreviewDialogPanel_, ID_ZOOMOUTBUTTON, wxT("Zoom out"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer23->Add( pZoomOutButton_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	
	bSizer23->Add( 0, 0, 1, wxEXPAND, 5 );
	
	pCloseButton_ = new wxButton( pImagePreviewDialogPanel_, ID_CLOSEBUTTON, wxT("Close"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer23->Add( pCloseButton_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	bSizer22->Add( bSizer23, 0, wxEXPAND, 5 );
	
	pImagePreviewDialogPanel_->SetSizer( bSizer22 );
	pImagePreviewDialogPanel_->Layout();
	bSizer22->Fit( pImagePreviewDialogPanel_ );
	bSizer21->Add( pImagePreviewDialogPanel_, 1, wxEXPAND, 0 );
	
	this->SetSizer( bSizer21 );
	this->Layout();
	
	this->Centre( wxBOTH );
}

Regard3DImagePreviewDialogBase::~Regard3DImagePreviewDialogBase()
{
}

BEGIN_EVENT_TABLE( Regard3DPictureSetDialogBase, wxDialog )
	EVT_INIT_DIALOG( Regard3DPictureSetDialogBase::_wxFB_OnInitDialog )
	EVT_BUTTON( ID_ADDFILESBUTTON, Regard3DPictureSetDialogBase::_wxFB_OnAddFilesButton )
	EVT_BUTTON( ID_REMOVESELECTEDFILESBUTTON, Regard3DPictureSetDialogBase::_wxFB_OnRemoveSelectedFilesButton )
	EVT_BUTTON( ID_CLEARFILELISTBUTTON, Regard3DPictureSetDialogBase::_wxFB_OnClearFileListButton )
	EVT_LIST_COL_CLICK( ID_IMAGELISTCTRL, Regard3DPictureSetDialogBase::_wxFB_OnImageListColClick )
	EVT_LIST_ITEM_DESELECTED( ID_IMAGELISTCTRL, Regard3DPictureSetDialogBase::_wxFB_OnImageListItemDeselected )
	EVT_LIST_ITEM_SELECTED( ID_IMAGELISTCTRL, Regard3DPictureSetDialogBase::_wxFB_OnImageListItemSelected )
	EVT_LIST_KEY_DOWN( ID_IMAGELISTCTRL, Regard3DPictureSetDialogBase::_wxFB_OnImageListKeyDown )
	EVT_BUTTON( wxID_CANCEL, Regard3DPictureSetDialogBase::_wxFB_OnCancel )
	EVT_BUTTON( wxID_OK, Regard3DPictureSetDialogBase::_wxFB_OnOK )
END_EVENT_TABLE()

Regard3DPictureSetDialogBase::Regard3DPictureSetDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxSize( 500,300 ), wxDefaultSize );
	
	wxBoxSizer* bSizer25;
	bSizer25 = new wxBoxSizer( wxVERTICAL );
	
	m_panel13 = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer26;
	bSizer26 = new wxBoxSizer( wxVERTICAL );
	
	pCentralSplitter_ = new wxSplitterWindow( m_panel13, ID_PCENTRALSPLITTER_, wxDefaultPosition, wxDefaultSize, wxSP_3D );
	pCentralSplitter_->Connect( wxEVT_IDLE, wxIdleEventHandler( Regard3DPictureSetDialogBase::pCentralSplitter_OnIdle ), NULL, this );
	
	m_panel10 = new wxPanel( pCentralSplitter_, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxStaticBoxSizer* sbSizer12;
	sbSizer12 = new wxStaticBoxSizer( new wxStaticBox( m_panel10, wxID_ANY, wxT("Image list") ), wxVERTICAL );
	
	wxBoxSizer* bSizer7;
	bSizer7 = new wxBoxSizer( wxHORIZONTAL );
	
	pAddFilesButton_ = new wxButton( m_panel10, ID_ADDFILESBUTTON, wxT("Add files..."), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer7->Add( pAddFilesButton_, 0, wxALL, 3 );
	
	pRemoveSelectedFilesButton_ = new wxButton( m_panel10, ID_REMOVESELECTEDFILESBUTTON, wxT("Remove selected files"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer7->Add( pRemoveSelectedFilesButton_, 0, wxALL, 3 );
	
	pClearFileListButton_ = new wxButton( m_panel10, ID_CLEARFILELISTBUTTON, wxT("Clear file list"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer7->Add( pClearFileListButton_, 0, wxALL, 3 );
	
	sbSizer12->Add( bSizer7, 0, wxEXPAND, 5 );
	
	pImageListCtrl_ = new wxListCtrl( m_panel10, ID_IMAGELISTCTRL, wxDefaultPosition, wxDefaultSize, wxLC_NO_SORT_HEADER|wxLC_REPORT );
	sbSizer12->Add( pImageListCtrl_, 1, wxALL|wxEXPAND, 3 );
	
	wxStaticBoxSizer* sbSizer14;
	sbSizer14 = new wxStaticBoxSizer( new wxStaticBox( m_panel10, wxID_ANY, wxT("Picture set") ), wxVERTICAL );
	
	wxFlexGridSizer* fgSizer5;
	fgSizer5 = new wxFlexGridSizer( 1, 2, 0, 0 );
	fgSizer5->AddGrowableCol( 1 );
	fgSizer5->SetFlexibleDirection( wxBOTH );
	fgSizer5->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText14 = new wxStaticText( m_panel10, wxID_ANY, wxT("Picture set name:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText14->Wrap( -1 );
	fgSizer5->Add( m_staticText14, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pPictureSetNameTextCtrl_ = new wxTextCtrl( m_panel10, ID_PICTURESETNAMETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	fgSizer5->Add( pPictureSetNameTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	sbSizer14->Add( fgSizer5, 1, wxEXPAND, 3 );
	
	sbSizer12->Add( sbSizer14, 0, wxALL|wxEXPAND, 3 );
	
	m_panel10->SetSizer( sbSizer12 );
	m_panel10->Layout();
	sbSizer12->Fit( m_panel10 );
	pImagePreviewPanel_ = new wxPanel( pCentralSplitter_, ID_IMAGEPREVIEWPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxStaticBoxSizer* sbSizer13;
	sbSizer13 = new wxStaticBoxSizer( new wxStaticBox( pImagePreviewPanel_, wxID_ANY, wxT("Preview") ), wxVERTICAL );
	
	pPreviewCanvas_ = new PreviewCanvas(pImagePreviewPanel_);
	sbSizer13->Add( pPreviewCanvas_, 1, wxALL|wxEXPAND, 3 );
	
	pImagePreviewPanel_->SetSizer( sbSizer13 );
	pImagePreviewPanel_->Layout();
	sbSizer13->Fit( pImagePreviewPanel_ );
	pCentralSplitter_->SplitVertically( m_panel10, pImagePreviewPanel_, 0 );
	bSizer26->Add( pCentralSplitter_, 1, wxALL|wxEXPAND, 3 );
	
	m_sdbSizer2 = new wxStdDialogButtonSizer();
	m_sdbSizer2OK = new wxButton( m_panel13, wxID_OK );
	m_sdbSizer2->AddButton( m_sdbSizer2OK );
	m_sdbSizer2Cancel = new wxButton( m_panel13, wxID_CANCEL );
	m_sdbSizer2->AddButton( m_sdbSizer2Cancel );
	m_sdbSizer2->Realize();
	bSizer26->Add( m_sdbSizer2, 0, wxALL|wxEXPAND, 3 );
	
	m_panel13->SetSizer( bSizer26 );
	m_panel13->Layout();
	bSizer26->Fit( m_panel13 );
	bSizer25->Add( m_panel13, 1, wxEXPAND, 5 );
	
	this->SetSizer( bSizer25 );
	this->Layout();
	
	this->Centre( wxBOTH );
}

Regard3DPictureSetDialogBase::~Regard3DPictureSetDialogBase()
{
}

BEGIN_EVENT_TABLE( Regard3DComputeMatchesDialogBase, wxDialog )
	EVT_INIT_DIALOG( Regard3DComputeMatchesDialogBase::_wxFB_OnInitDialog )
	EVT_COMMAND_SCROLL( ID_KEYPOINTSENSITIVITYSLIDER, Regard3DComputeMatchesDialogBase::_wxFB_OnKeypointSensitivitySlider )
	EVT_COMMAND_SCROLL( ID_KEYPOINTMATCHINGRATIOSLIDER, Regard3DComputeMatchesDialogBase::_wxFB_OnKeypointMatchingRatioSlider )
	EVT_CHOICE( ID_NUMBEROFTHREADSCHOICE, Regard3DComputeMatchesDialogBase::_wxFB_OnNumberOfThreadsChoice )
END_EVENT_TABLE()

Regard3DComputeMatchesDialogBase::Regard3DComputeMatchesDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer35;
	bSizer35 = new wxBoxSizer( wxVERTICAL );
	
	pComputeMatchesDialogPanel_ = new wxPanel( this, ID_COMPUTEMATCHESDIALOGPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer36;
	bSizer36 = new wxBoxSizer( wxVERTICAL );
	
	wxStaticBoxSizer* sbSizer3;
	sbSizer3 = new wxStaticBoxSizer( new wxStaticBox( pComputeMatchesDialogPanel_, wxID_ANY, wxT("Image correlations parameters") ), wxVERTICAL );
	
	wxFlexGridSizer* fgSizer1;
	fgSizer1 = new wxFlexGridSizer( 3, 4, 0, 0 );
	fgSizer1->AddGrowableCol( 2 );
	fgSizer1->SetFlexibleDirection( wxHORIZONTAL );
	fgSizer1->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText1 = new wxStaticText( pComputeMatchesDialogPanel_, wxID_ANY, wxT("Keypoint sensitivity:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText1->Wrap( -1 );
	fgSizer1->Add( m_staticText1, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pKeypointSensitivityTextCtrl_ = new wxTextCtrl( pComputeMatchesDialogPanel_, ID_KEYPOINTSENSITIVITYTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pKeypointSensitivityTextCtrl_->SetMinSize( wxSize( 120,-1 ) );
	
	fgSizer1->Add( pKeypointSensitivityTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pKeypointSensitivitySlider_ = new wxSlider( pComputeMatchesDialogPanel_, ID_KEYPOINTSENSITIVITYSLIDER, 1, 0, 3, wxDefaultPosition, wxDefaultSize, wxSL_AUTOTICKS|wxSL_BOTH|wxSL_HORIZONTAL );
	pKeypointSensitivitySlider_->SetToolTip( wxT("This parameter defines how many keypoints are generated. Lower value means higher sensitivity and thus more keypoints.") );
	
	fgSizer1->Add( pKeypointSensitivitySlider_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	pKeypointSensitivityValTextCtrl_ = new wxTextCtrl( pComputeMatchesDialogPanel_, ID_KEYPOINTSENSITIVITYVALTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer1->Add( pKeypointSensitivityValTextCtrl_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	m_staticText2 = new wxStaticText( pComputeMatchesDialogPanel_, wxID_ANY, wxT("Keypoint matching ratio:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText2->Wrap( -1 );
	fgSizer1->Add( m_staticText2, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );
	
	pKeypointMatchingRatioTextCtrl_ = new wxTextCtrl( pComputeMatchesDialogPanel_, ID_KEYPOINTMATCHINGRATIOTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	pKeypointMatchingRatioTextCtrl_->SetMinSize( wxSize( 120,-1 ) );
	
	fgSizer1->Add( pKeypointMatchingRatioTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pKeypointMatchingRatioSlider_ = new wxSlider( pComputeMatchesDialogPanel_, ID_KEYPOINTMATCHINGRATIOSLIDER, 1, 0, 3, wxDefaultPosition, wxDefaultSize, wxSL_AUTOTICKS|wxSL_BOTH|wxSL_HORIZONTAL );
	pKeypointMatchingRatioSlider_->SetToolTip( wxT("Nearest Neighbor distance ratio. This parameter defines how similar two keypoints must be in relation to the next best keypoint in order to be accepted.") );
	
	fgSizer1->Add( pKeypointMatchingRatioSlider_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	pKeypointMatchingRatioValTextCtrl_ = new wxTextCtrl( pComputeMatchesDialogPanel_, ID_KEYPOINTMATCHINGRATIOVALTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer1->Add( pKeypointMatchingRatioValTextCtrl_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	m_staticText4 = new wxStaticText( pComputeMatchesDialogPanel_, wxID_ANY, wxT("Keypoint detection threads:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText4->Wrap( -1 );
	fgSizer1->Add( m_staticText4, 1, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	wxArrayString pNumberOfThreadsChoice_Choices;
	pNumberOfThreadsChoice_ = new wxChoice( pComputeMatchesDialogPanel_, ID_NUMBEROFTHREADSCHOICE, wxDefaultPosition, wxDefaultSize, pNumberOfThreadsChoice_Choices, 0 );
	pNumberOfThreadsChoice_->SetSelection( 0 );
	fgSizer1->Add( pNumberOfThreadsChoice_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pKeypointSuggestedNumThreadsStaticText_ = new wxStaticText( pComputeMatchesDialogPanel_, ID_KEYPOINTSUGGESTEDNUMTHREADSSTATICTEXT, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	pKeypointSuggestedNumThreadsStaticText_->Wrap( -1 );
	fgSizer1->Add( pKeypointSuggestedNumThreadsStaticText_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	
	fgSizer1->Add( 0, 0, 1, wxEXPAND, 5 );
	
	sbSizer3->Add( fgSizer1, 0, wxEXPAND, 3 );
	
	bSizer36->Add( sbSizer3, 1, wxALL|wxEXPAND, 3 );
	
	m_sdbSizer3 = new wxStdDialogButtonSizer();
	m_sdbSizer3OK = new wxButton( pComputeMatchesDialogPanel_, wxID_OK );
	m_sdbSizer3->AddButton( m_sdbSizer3OK );
	m_sdbSizer3Cancel = new wxButton( pComputeMatchesDialogPanel_, wxID_CANCEL );
	m_sdbSizer3->AddButton( m_sdbSizer3Cancel );
	m_sdbSizer3->Realize();
	bSizer36->Add( m_sdbSizer3, 0, wxALL|wxEXPAND, 3 );
	
	pComputeMatchesDialogPanel_->SetSizer( bSizer36 );
	pComputeMatchesDialogPanel_->Layout();
	bSizer36->Fit( pComputeMatchesDialogPanel_ );
	bSizer35->Add( pComputeMatchesDialogPanel_, 1, wxEXPAND, 5 );
	
	this->SetSizer( bSizer35 );
	this->Layout();
	
	this->Centre( wxBOTH );
}

Regard3DComputeMatchesDialogBase::~Regard3DComputeMatchesDialogBase()
{
}

BEGIN_EVENT_TABLE( Regard3DTriangulationDialogBase, wxDialog )
	EVT_INIT_DIALOG( Regard3DTriangulationDialogBase::_wxFB_OnInitDialog )
	EVT_RADIOBOX( ID_TRIANGULATIONMETHODRADIOBOX, Regard3DTriangulationDialogBase::_wxFB_OnTriangulationMethodRadioBox )
	EVT_LIST_COL_CLICK( ID_TINITIALIMAGEPAIRLISTCTRL, Regard3DTriangulationDialogBase::_wxFB_OnTInitialImagePairColClick )
	EVT_LIST_ITEM_DESELECTED( ID_TINITIALIMAGEPAIRLISTCTRL, Regard3DTriangulationDialogBase::_wxFB_OnTInitialImagePairItemDeselected )
	EVT_LIST_ITEM_SELECTED( ID_TINITIALIMAGEPAIRLISTCTRL, Regard3DTriangulationDialogBase::_wxFB_OnTInitialImagePairItemSelected )
	EVT_CHECKBOX( ID_TPREVIEWWITHMATCHESCHECKBOX, Regard3DTriangulationDialogBase::_wxFB_OnTPreviewWithMatchesCheckBox )
END_EVENT_TABLE()

Regard3DTriangulationDialogBase::Regard3DTriangulationDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxSize( 200,200 ), wxDefaultSize );
	
	wxBoxSizer* bSizer38;
	bSizer38 = new wxBoxSizer( wxVERTICAL );
	
	pTriangulationPanel_ = new wxPanel( this, ID_TRIANGULATIONPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer39;
	bSizer39 = new wxBoxSizer( wxVERTICAL );
	
	wxStaticBoxSizer* sbSizer4;
	sbSizer4 = new wxStaticBoxSizer( new wxStaticBox( pTriangulationPanel_, wxID_ANY, wxT("Triangulation parameters") ), wxVERTICAL );
	
	wxString pTriangulationMethodRadioBox_Choices[] = { wxT("Incremental Structure from Motion"), wxT("Global Structure from Motion") };
	int pTriangulationMethodRadioBox_NChoices = sizeof( pTriangulationMethodRadioBox_Choices ) / sizeof( wxString );
	pTriangulationMethodRadioBox_ = new wxRadioBox( pTriangulationPanel_, ID_TRIANGULATIONMETHODRADIOBOX, wxT("Method"), wxDefaultPosition, wxDefaultSize, pTriangulationMethodRadioBox_NChoices, pTriangulationMethodRadioBox_Choices, 1, wxRA_SPECIFY_COLS );
	pTriangulationMethodRadioBox_->SetSelection( 0 );
	sbSizer4->Add( pTriangulationMethodRadioBox_, 0, wxALL, 3 );
	
	pTIncrementalMethodBoxSizer_ = new wxStaticBoxSizer( new wxStaticBox( pTriangulationPanel_, ID_TINCREMENTALMETHODBOXSIZER, wxT("Initial image pair for incremental method") ), wxVERTICAL );
	
	pIncrementalMethodBoxSplitter_ = new wxSplitterWindow( pTriangulationPanel_, ID_INCREMENTALMETHODBOXSPLITTER, wxDefaultPosition, wxDefaultSize, wxSP_3D );
	pIncrementalMethodBoxSplitter_->Connect( wxEVT_IDLE, wxIdleEventHandler( Regard3DTriangulationDialogBase::pIncrementalMethodBoxSplitter_OnIdle ), NULL, this );
	
	pIncrementalMethodBoxLeftPanel_ = new wxPanel( pIncrementalMethodBoxSplitter_, ID_INCREMENTALMETHODBOXLEFTPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer45;
	bSizer45 = new wxBoxSizer( wxVERTICAL );
	
	pTInitialImagePairListCtrl_ = new wxListCtrl( pIncrementalMethodBoxLeftPanel_, ID_TINITIALIMAGEPAIRLISTCTRL, wxDefaultPosition, wxDefaultSize, wxLC_REPORT|wxLC_SINGLE_SEL );
	pTInitialImagePairListCtrl_->SetMinSize( wxSize( 300,200 ) );
	
	bSizer45->Add( pTInitialImagePairListCtrl_, 1, wxALL|wxEXPAND, 3 );
	
	pIncrementalMethodBoxLeftPanel_->SetSizer( bSizer45 );
	pIncrementalMethodBoxLeftPanel_->Layout();
	bSizer45->Fit( pIncrementalMethodBoxLeftPanel_ );
	pIncrementalMethodBoxRightPanel_ = new wxPanel( pIncrementalMethodBoxSplitter_, ID_INCREMENTALMETHODBOXRIGHTPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer9;
	bSizer9 = new wxBoxSizer( wxVERTICAL );
	
	pPreviewCanvas_ = new PreviewCanvas(pIncrementalMethodBoxRightPanel_);
	bSizer9->Add( pPreviewCanvas_, 1, wxALL|wxEXPAND, 3 );
	
	wxBoxSizer* bSizer24;
	bSizer24 = new wxBoxSizer( wxHORIZONTAL );
	
	pTPreviewWithMatchesCheckBox_ = new wxCheckBox( pIncrementalMethodBoxRightPanel_, ID_TPREVIEWWITHMATCHESCHECKBOX, wxT("Show Matches"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer24->Add( pTPreviewWithMatchesCheckBox_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	
	bSizer24->Add( 0, 0, 1, wxEXPAND, 5 );
	
	bSizer9->Add( bSizer24, 0, wxEXPAND, 5 );
	
	pIncrementalMethodBoxRightPanel_->SetSizer( bSizer9 );
	pIncrementalMethodBoxRightPanel_->Layout();
	bSizer9->Fit( pIncrementalMethodBoxRightPanel_ );
	pIncrementalMethodBoxSplitter_->SplitVertically( pIncrementalMethodBoxLeftPanel_, pIncrementalMethodBoxRightPanel_, 0 );
	pTIncrementalMethodBoxSizer_->Add( pIncrementalMethodBoxSplitter_, 1, wxEXPAND, 5 );
	
	sbSizer4->Add( pTIncrementalMethodBoxSizer_, 1, wxALL|wxEXPAND, 3 );
	
	pTGlobalMethodBoxSizer_ = new wxStaticBoxSizer( new wxStaticBox( pTriangulationPanel_, ID_TGLOBALMETHODBOXSIZER, wxT("Global method parameters") ), wxVERTICAL );
	
	wxString pTGlobalRotAvgMethodRatioBox_Choices[] = { wxT("MST based rotation + L1 rotation averaging"), wxT("Dense L2 global rotation computation") };
	int pTGlobalRotAvgMethodRatioBox_NChoices = sizeof( pTGlobalRotAvgMethodRatioBox_Choices ) / sizeof( wxString );
	pTGlobalRotAvgMethodRatioBox_ = new wxRadioBox( pTriangulationPanel_, ID_TGLOBALROTAVGMETHODRATIOBOX, wxT("Rotation averaging method"), wxDefaultPosition, wxDefaultSize, pTGlobalRotAvgMethodRatioBox_NChoices, pTGlobalRotAvgMethodRatioBox_Choices, 1, wxRA_SPECIFY_COLS );
	pTGlobalRotAvgMethodRatioBox_->SetSelection( 0 );
	pTGlobalMethodBoxSizer_->Add( pTGlobalRotAvgMethodRatioBox_, 0, wxALL, 3 );
	
	sbSizer4->Add( pTGlobalMethodBoxSizer_, 0, wxALL|wxEXPAND, 3 );
	
	bSizer39->Add( sbSizer4, 1, wxALL|wxEXPAND, 3 );
	
	m_sdbSizer4 = new wxStdDialogButtonSizer();
	m_sdbSizer4OK = new wxButton( pTriangulationPanel_, wxID_OK );
	m_sdbSizer4->AddButton( m_sdbSizer4OK );
	m_sdbSizer4Cancel = new wxButton( pTriangulationPanel_, wxID_CANCEL );
	m_sdbSizer4->AddButton( m_sdbSizer4Cancel );
	m_sdbSizer4->Realize();
	bSizer39->Add( m_sdbSizer4, 0, wxALL|wxEXPAND, 3 );
	
	pTriangulationPanel_->SetSizer( bSizer39 );
	pTriangulationPanel_->Layout();
	bSizer39->Fit( pTriangulationPanel_ );
	bSizer38->Add( pTriangulationPanel_, 1, wxEXPAND, 3 );
	
	this->SetSizer( bSizer38 );
	this->Layout();
	
	this->Centre( wxBOTH );
}

Regard3DTriangulationDialogBase::~Regard3DTriangulationDialogBase()
{
}

BEGIN_EVENT_TABLE( Regard3DMatchingResultsDialogBase, wxDialog )
	EVT_INIT_DIALOG( Regard3DMatchingResultsDialogBase::_wxFB_OnInitDialog )
	EVT_LIST_COL_CLICK( ID_IMAGELISTCTRL, Regard3DMatchingResultsDialogBase::_wxFB_OnIPImageListColClick )
	EVT_LIST_ITEM_DESELECTED( ID_IMAGELISTCTRL, Regard3DMatchingResultsDialogBase::_wxFB_OnIPImageListItemDeselected )
	EVT_LIST_ITEM_SELECTED( ID_IMAGELISTCTRL, Regard3DMatchingResultsDialogBase::_wxFB_OnIPImageListItemSelected )
	EVT_LIST_KEY_DOWN( ID_IMAGELISTCTRL, Regard3DMatchingResultsDialogBase::_wxFB_OnIPImageListKeyDown )
	EVT_CHECKBOX( ID_IPPREVIEWWITHKEYPOINTSCHECKBOX, Regard3DMatchingResultsDialogBase::_wxFB_OnIPPreviewWithKeypointsCheckBox )
	EVT_BUTTON( ID_IPOPENPREVIEWWINDOW, Regard3DMatchingResultsDialogBase::_wxFB_OnIPOpenPreviewWindow )
	EVT_CHOICE( ID_MATCHESCHOICE, Regard3DMatchingResultsDialogBase::_wxFB_OnMatchesChoice )
	EVT_LIST_COL_CLICK( ID_TINITIALIMAGEPAIRLISTCTRL, Regard3DMatchingResultsDialogBase::_wxFB_OnTInitialImagePairColClick )
	EVT_LIST_ITEM_DESELECTED( ID_TINITIALIMAGEPAIRLISTCTRL, Regard3DMatchingResultsDialogBase::_wxFB_OnTInitialImagePairItemDeselected )
	EVT_LIST_ITEM_SELECTED( ID_TINITIALIMAGEPAIRLISTCTRL, Regard3DMatchingResultsDialogBase::_wxFB_OnTInitialImagePairItemSelected )
	EVT_CHECKBOX( ID_TPREVIEWWITHMATCHESCHECKBOX, Regard3DMatchingResultsDialogBase::_wxFB_OnTPreviewWithMatchesCheckBox )
	EVT_BUTTON( ID_TOPENPREVIEWWINDOW, Regard3DMatchingResultsDialogBase::_wxFB_OnTOpenPreviewWindow )
END_EVENT_TABLE()

Regard3DMatchingResultsDialogBase::Regard3DMatchingResultsDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxSize( 200,200 ), wxDefaultSize );
	
	wxBoxSizer* bSizer48;
	bSizer48 = new wxBoxSizer( wxVERTICAL );
	
	pMatchingResultsPanel_ = new wxPanel( this, ID_MATCHINGRESULTSPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer49;
	bSizer49 = new wxBoxSizer( wxVERTICAL );
	
	pHorizontalSplitter_ = new wxSplitterWindow( pMatchingResultsPanel_, ID_PHORIZONTALSPLITTER_, wxDefaultPosition, wxDefaultSize, wxSP_3D );
	pHorizontalSplitter_->Connect( wxEVT_IDLE, wxIdleEventHandler( Regard3DMatchingResultsDialogBase::pHorizontalSplitter_OnIdle ), NULL, this );
	
	pKeypointsPanel_ = new wxPanel( pHorizontalSplitter_, ID_KEYPOINTSPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer50;
	bSizer50 = new wxBoxSizer( wxVERTICAL );
	
	wxStaticBoxSizer* sbSizer19;
	sbSizer19 = new wxStaticBoxSizer( new wxStaticBox( pKeypointsPanel_, wxID_ANY, wxT("Keypoints") ), wxVERTICAL );
	
	pUpperVerticalSplitter_ = new wxSplitterWindow( pKeypointsPanel_, ID_UPPERVERTICALSPLITTER, wxDefaultPosition, wxDefaultSize, wxSP_3D );
	pUpperVerticalSplitter_->Connect( wxEVT_IDLE, wxIdleEventHandler( Regard3DMatchingResultsDialogBase::pUpperVerticalSplitter_OnIdle ), NULL, this );
	
	pImageListPanel_ = new wxPanel( pUpperVerticalSplitter_, ID_IMAGELISTPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer52;
	bSizer52 = new wxBoxSizer( wxVERTICAL );
	
	pImageListCtrl_ = new wxListCtrl( pImageListPanel_, ID_IMAGELISTCTRL, wxDefaultPosition, wxDefaultSize, wxLC_NO_SORT_HEADER|wxLC_REPORT );
	bSizer52->Add( pImageListCtrl_, 1, wxALL|wxEXPAND, 3 );
	
	pImageListPanel_->SetSizer( bSizer52 );
	pImageListPanel_->Layout();
	bSizer52->Fit( pImageListPanel_ );
	pKeypointPreviewPanel_ = new wxPanel( pUpperVerticalSplitter_, ID_KEYPOINTPREVIEWPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer53;
	bSizer53 = new wxBoxSizer( wxVERTICAL );
	
	pIPPreviewCanvas_ = new PreviewCanvas(pKeypointPreviewPanel_);
	bSizer53->Add( pIPPreviewCanvas_, 1, wxALL|wxEXPAND, 3 );
	
	wxBoxSizer* bSizer20;
	bSizer20 = new wxBoxSizer( wxHORIZONTAL );
	
	pIPPreviewWithKeypointsCheckBox_ = new wxCheckBox( pKeypointPreviewPanel_, ID_IPPREVIEWWITHKEYPOINTSCHECKBOX, wxT("Show Keypoints"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer20->Add( pIPPreviewWithKeypointsCheckBox_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	
	bSizer20->Add( 0, 0, 1, wxEXPAND, 3 );
	
	pIPOpenPreviewWindow_ = new wxButton( pKeypointPreviewPanel_, ID_IPOPENPREVIEWWINDOW, wxT("Open Preview Window"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer20->Add( pIPOpenPreviewWindow_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	bSizer53->Add( bSizer20, 0, wxEXPAND, 3 );
	
	pKeypointPreviewPanel_->SetSizer( bSizer53 );
	pKeypointPreviewPanel_->Layout();
	bSizer53->Fit( pKeypointPreviewPanel_ );
	pUpperVerticalSplitter_->SplitVertically( pImageListPanel_, pKeypointPreviewPanel_, 0 );
	sbSizer19->Add( pUpperVerticalSplitter_, 1, wxEXPAND, 5 );
	
	bSizer50->Add( sbSizer19, 1, wxALL|wxEXPAND, 3 );
	
	pKeypointsPanel_->SetSizer( bSizer50 );
	pKeypointsPanel_->Layout();
	bSizer50->Fit( pKeypointsPanel_ );
	pMatchesPanel_ = new wxPanel( pHorizontalSplitter_, ID_MATCHESPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer521;
	bSizer521 = new wxBoxSizer( wxVERTICAL );
	
	wxStaticBoxSizer* sbSizer20;
	sbSizer20 = new wxStaticBoxSizer( new wxStaticBox( pMatchesPanel_, wxID_ANY, wxT("Matches") ), wxVERTICAL );
	
	pLowerVerticalSplitter_ = new wxSplitterWindow( pMatchesPanel_, ID_LOWERVERTICALSPLITTER, wxDefaultPosition, wxDefaultSize, wxSP_3D );
	pLowerVerticalSplitter_->Connect( wxEVT_IDLE, wxIdleEventHandler( Regard3DMatchingResultsDialogBase::pLowerVerticalSplitter_OnIdle ), NULL, this );
	
	pMatchingListPanel_ = new wxPanel( pLowerVerticalSplitter_, ID_MATCHINGLISTPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer57;
	bSizer57 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer62;
	bSizer62 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer62->Add( 0, 0, 1, wxEXPAND, 5 );
	
	m_staticText20 = new wxStaticText( pMatchingListPanel_, wxID_ANY, wxT("Show matches filtered by:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText20->Wrap( -1 );
	bSizer62->Add( m_staticText20, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	wxArrayString pMatchesChoice_Choices;
	pMatchesChoice_ = new wxChoice( pMatchingListPanel_, ID_MATCHESCHOICE, wxDefaultPosition, wxDefaultSize, pMatchesChoice_Choices, 0 );
	pMatchesChoice_->SetSelection( 0 );
	bSizer62->Add( pMatchesChoice_, 0, wxALL, 3 );
	
	
	bSizer62->Add( 0, 0, 1, wxEXPAND, 5 );
	
	bSizer57->Add( bSizer62, 0, wxEXPAND, 3 );
	
	pTInitialImagePairListCtrl_ = new wxListCtrl( pMatchingListPanel_, ID_TINITIALIMAGEPAIRLISTCTRL, wxDefaultPosition, wxDefaultSize, wxLC_REPORT|wxLC_SINGLE_SEL );
	pTInitialImagePairListCtrl_->SetMinSize( wxSize( 300,200 ) );
	
	bSizer57->Add( pTInitialImagePairListCtrl_, 1, wxALL|wxEXPAND, 3 );
	
	pMatchingListPanel_->SetSizer( bSizer57 );
	pMatchingListPanel_->Layout();
	bSizer57->Fit( pMatchingListPanel_ );
	pMatchingPreviewPanel_ = new wxPanel( pLowerVerticalSplitter_, ID_MATCHINGPREVIEWPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer58;
	bSizer58 = new wxBoxSizer( wxVERTICAL );
	
	pTPreviewCanvas_ = new PreviewCanvas(pMatchingPreviewPanel_);
	bSizer58->Add( pTPreviewCanvas_, 1, wxALL|wxEXPAND, 3 );
	
	wxBoxSizer* bSizer24;
	bSizer24 = new wxBoxSizer( wxHORIZONTAL );
	
	pTPreviewWithMatchesCheckBox_ = new wxCheckBox( pMatchingPreviewPanel_, ID_TPREVIEWWITHMATCHESCHECKBOX, wxT("Show Matches"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer24->Add( pTPreviewWithMatchesCheckBox_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	
	bSizer24->Add( 0, 0, 1, wxEXPAND, 5 );
	
	pTOpenPreviewWindow_ = new wxButton( pMatchingPreviewPanel_, ID_TOPENPREVIEWWINDOW, wxT("Open Preview Window"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer24->Add( pTOpenPreviewWindow_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	bSizer58->Add( bSizer24, 0, wxEXPAND, 3 );
	
	pMatchingPreviewPanel_->SetSizer( bSizer58 );
	pMatchingPreviewPanel_->Layout();
	bSizer58->Fit( pMatchingPreviewPanel_ );
	pLowerVerticalSplitter_->SplitVertically( pMatchingListPanel_, pMatchingPreviewPanel_, 0 );
	sbSizer20->Add( pLowerVerticalSplitter_, 1, wxEXPAND, 5 );
	
	bSizer521->Add( sbSizer20, 1, wxALL|wxEXPAND, 3 );
	
	pMatchesPanel_->SetSizer( bSizer521 );
	pMatchesPanel_->Layout();
	bSizer521->Fit( pMatchesPanel_ );
	pHorizontalSplitter_->SplitHorizontally( pKeypointsPanel_, pMatchesPanel_, 0 );
	bSizer49->Add( pHorizontalSplitter_, 1, wxEXPAND, 5 );
	
	m_sdbSizer5 = new wxStdDialogButtonSizer();
	m_sdbSizer5OK = new wxButton( pMatchingResultsPanel_, wxID_OK );
	m_sdbSizer5->AddButton( m_sdbSizer5OK );
	m_sdbSizer5->Realize();
	bSizer49->Add( m_sdbSizer5, 0, wxALL|wxEXPAND, 3 );
	
	pMatchingResultsPanel_->SetSizer( bSizer49 );
	pMatchingResultsPanel_->Layout();
	bSizer49->Fit( pMatchingResultsPanel_ );
	bSizer48->Add( pMatchingResultsPanel_, 1, wxEXPAND, 5 );
	
	this->SetSizer( bSizer48 );
	this->Layout();
	
	this->Centre( wxBOTH );
}

Regard3DMatchingResultsDialogBase::~Regard3DMatchingResultsDialogBase()
{
}

BEGIN_EVENT_TABLE( Regard3DDensificationDialogBase, wxDialog )
	EVT_INIT_DIALOG( Regard3DDensificationDialogBase::_wxFB_OnInitDialog )
	EVT_RADIOBOX( ID_DENSIFICATIONMETHODRADIOBOX, Regard3DDensificationDialogBase::_wxFB_OnDensificationMethodRadioBox )
	EVT_CHECKBOX( ID_USECMVSCHECKBOX, Regard3DDensificationDialogBase::_wxFB_OnUseCMVSCheckBox )
	EVT_COMMAND_SCROLL( ID_PMVSLEVELSLIDER, Regard3DDensificationDialogBase::_wxFB_OnPMVSLevelSliderScroll )
	EVT_COMMAND_SCROLL( ID_PMVSCELLSIZESLIDER, Regard3DDensificationDialogBase::_wxFB_OnPMVSCellSizeSliderScroll )
	EVT_COMMAND_SCROLL( ID_PMVSTHRESHOLDSLIDER, Regard3DDensificationDialogBase::_wxFB_OnPMVSThresholdSliderScroll )
	EVT_COMMAND_SCROLL( ID_PMVSWSIZESLIDER, Regard3DDensificationDialogBase::_wxFB_OnPMVSWSizeSliderScroll )
	EVT_COMMAND_SCROLL( ID_PMVSMINIMAGENUMSLIDER, Regard3DDensificationDialogBase::_wxFB_OnPMVSMinImageNumSliderScroll )
	EVT_COMMAND_SCROLL( ID_MVESCALESLIDER, Regard3DDensificationDialogBase::_wxFB_OnMVEScaleSliderScroll )
	EVT_COMMAND_SCROLL( ID_MVEFILTERWIDTHSLIDER, Regard3DDensificationDialogBase::_wxFB_OnMVEFilterWidthSliderScroll )
END_EVENT_TABLE()

Regard3DDensificationDialogBase::Regard3DDensificationDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxSize( 560,-1 ), wxDefaultSize );
	
	wxBoxSizer* bSizer43;
	bSizer43 = new wxBoxSizer( wxVERTICAL );
	
	pDensificationPanel_ = new wxPanel( this, ID_DENSIFICATIONPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer44;
	bSizer44 = new wxBoxSizer( wxVERTICAL );
	
	wxString pDensificationMethodRadioBox_Choices[] = { wxT("CMVS/PMVS"), wxT("MVE (Multi-View Environment)") };
	int pDensificationMethodRadioBox_NChoices = sizeof( pDensificationMethodRadioBox_Choices ) / sizeof( wxString );
	pDensificationMethodRadioBox_ = new wxRadioBox( pDensificationPanel_, ID_DENSIFICATIONMETHODRADIOBOX, wxT("Densification method"), wxDefaultPosition, wxDefaultSize, pDensificationMethodRadioBox_NChoices, pDensificationMethodRadioBox_Choices, 1, wxRA_SPECIFY_COLS );
	pDensificationMethodRadioBox_->SetSelection( 1 );
	bSizer44->Add( pDensificationMethodRadioBox_, 0, wxALL, 3 );
	
	pPMVSParamsBoxSizer_ = new wxStaticBoxSizer( new wxStaticBox( pDensificationPanel_, ID_PMVSPARAMSBOXSIZER, wxT("Parameters for CMVS/PMVS") ), wxVERTICAL );
	
	wxBoxSizer* bSizer46;
	bSizer46 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText29 = new wxStaticText( pDensificationPanel_, wxID_ANY, wxT("Number of threads:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText29->Wrap( -1 );
	bSizer46->Add( m_staticText29, 0, wxALIGN_CENTER_VERTICAL|wxALL|wxRIGHT, 3 );
	
	wxArrayString pNumberOfThreadsChoice_Choices;
	pNumberOfThreadsChoice_ = new wxChoice( pDensificationPanel_, ID_NUMBEROFTHREADSCHOICE, wxDefaultPosition, wxDefaultSize, pNumberOfThreadsChoice_Choices, 0 );
	pNumberOfThreadsChoice_->SetSelection( 0 );
	bSizer46->Add( pNumberOfThreadsChoice_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pPMVSParamsBoxSizer_->Add( bSizer46, 0, wxEXPAND, 5 );
	
	pUseCMVSCheckBox_ = new wxCheckBox( pDensificationPanel_, ID_USECMVSCHECKBOX, wxT("Use visibility information (CMVS)"), wxDefaultPosition, wxDefaultSize, 0 );
	pUseCMVSCheckBox_->SetValue(true); 
	pUseCMVSCheckBox_->SetToolTip( wxT("Use visibility information to speed up the computation. Recommended for large scenes.\nSwitching this off can improve densification, but is only recommended for small scenes.") );
	
	pPMVSParamsBoxSizer_->Add( pUseCMVSCheckBox_, 0, wxALL, 3 );
	
	wxBoxSizer* bSizer45;
	bSizer45 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText28 = new wxStaticText( pDensificationPanel_, wxID_ANY, wxT("Maximum number of images per cluster:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText28->Wrap( -1 );
	bSizer45->Add( m_staticText28, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pMaxImageTextCtrl_ = new wxTextCtrl( pDensificationPanel_, ID_MAXIMAGETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	pMaxImageTextCtrl_->SetValidator( wxTextValidator( wxFILTER_NUMERIC, &maxImage_ ) );
	
	bSizer45->Add( pMaxImageTextCtrl_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pPMVSParamsBoxSizer_->Add( bSizer45, 1, wxEXPAND, 5 );
	
	wxFlexGridSizer* fgSizer9;
	fgSizer9 = new wxFlexGridSizer( 5, 3, 0, 0 );
	fgSizer9->AddGrowableCol( 2 );
	fgSizer9->SetFlexibleDirection( wxHORIZONTAL );
	fgSizer9->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText30 = new wxStaticText( pDensificationPanel_, wxID_ANY, wxT("Level:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText30->Wrap( -1 );
	fgSizer9->Add( m_staticText30, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pPMVSLevelTextCtrl_ = new wxTextCtrl( pDensificationPanel_, ID_PMVSLEVELTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer9->Add( pPMVSLevelTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pPMVSLevelSlider_ = new wxSlider( pDensificationPanel_, ID_PMVSLEVELSLIDER, 1, 0, 6, wxDefaultPosition, wxDefaultSize, wxSL_AUTOTICKS|wxSL_BOTTOM|wxSL_HORIZONTAL );
	fgSizer9->Add( pPMVSLevelSlider_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText31 = new wxStaticText( pDensificationPanel_, wxID_ANY, wxT("Cell size:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText31->Wrap( -1 );
	fgSizer9->Add( m_staticText31, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pPMVSCellSizeTextCtrl_ = new wxTextCtrl( pDensificationPanel_, ID_PMVSCELLSIZETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer9->Add( pPMVSCellSizeTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pPMVSCellSizeSlider_ = new wxSlider( pDensificationPanel_, ID_PMVSCELLSIZESLIDER, 2, 1, 7, wxDefaultPosition, wxDefaultSize, wxSL_AUTOTICKS|wxSL_BOTTOM|wxSL_HORIZONTAL );
	fgSizer9->Add( pPMVSCellSizeSlider_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText32 = new wxStaticText( pDensificationPanel_, wxID_ANY, wxT("Threshold:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText32->Wrap( -1 );
	fgSizer9->Add( m_staticText32, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pPMVSThresholdTextCtrl_ = new wxTextCtrl( pDensificationPanel_, ID_PMVSTHRESHOLDTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer9->Add( pPMVSThresholdTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pPMVSThresholdSlider_ = new wxSlider( pDensificationPanel_, ID_PMVSTHRESHOLDSLIDER, 70, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	fgSizer9->Add( pPMVSThresholdSlider_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText33 = new wxStaticText( pDensificationPanel_, wxID_ANY, wxT("wsize:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText33->Wrap( -1 );
	fgSizer9->Add( m_staticText33, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pPMVSWSizeTextCtrl_ = new wxTextCtrl( pDensificationPanel_, ID_PMVSWSIZETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer9->Add( pPMVSWSizeTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pPMVSWSizeSlider_ = new wxSlider( pDensificationPanel_, ID_PMVSWSIZESLIDER, 7, 5, 20, wxDefaultPosition, wxDefaultSize, wxSL_AUTOTICKS|wxSL_HORIZONTAL );
	fgSizer9->Add( pPMVSWSizeSlider_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText34 = new wxStaticText( pDensificationPanel_, wxID_ANY, wxT("Min. image num:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText34->Wrap( -1 );
	fgSizer9->Add( m_staticText34, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pPMVSMinImageNumTextCtrl_ = new wxTextCtrl( pDensificationPanel_, ID_PMVSMINIMAGENUMTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer9->Add( pPMVSMinImageNumTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pPMVSMinImageNumSlider_ = new wxSlider( pDensificationPanel_, ID_PMVSMINIMAGENUMSLIDER, 3, 2, 6, wxDefaultPosition, wxDefaultSize, wxSL_AUTOTICKS|wxSL_HORIZONTAL );
	fgSizer9->Add( pPMVSMinImageNumSlider_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	pPMVSParamsBoxSizer_->Add( fgSizer9, 0, wxEXPAND, 3 );
	
	bSizer44->Add( pPMVSParamsBoxSizer_, 0, wxALL|wxEXPAND, 3 );
	
	pMVEParamsBoxSizer_ = new wxStaticBoxSizer( new wxStaticBox( pDensificationPanel_, ID_MVEPARAMSBOXSIZER, wxT("Parameters for MVE") ), wxVERTICAL );
	
	wxFlexGridSizer* fgSizer14;
	fgSizer14 = new wxFlexGridSizer( 2, 3, 0, 0 );
	fgSizer14->AddGrowableCol( 2 );
	fgSizer14->SetFlexibleDirection( wxHORIZONTAL );
	fgSizer14->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText53 = new wxStaticText( pDensificationPanel_, wxID_ANY, wxT("Scale:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText53->Wrap( -1 );
	fgSizer14->Add( m_staticText53, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pMVEScaleTextCtrl_ = new wxTextCtrl( pDensificationPanel_, ID_MVESCALETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer14->Add( pMVEScaleTextCtrl_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pMVEScaleSlider_ = new wxSlider( pDensificationPanel_, ID_MVESCALESLIDER, 2, 0, 6, wxDefaultPosition, wxDefaultSize, wxSL_AUTOTICKS|wxSL_HORIZONTAL );
	pMVEScaleSlider_->SetToolTip( wxT("Reconstruction on given scale. Smaller scale will increase detail, but also will take more time to compute.\nScale 0 is not recommended.") );
	
	fgSizer14->Add( pMVEScaleSlider_, 0, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText54 = new wxStaticText( pDensificationPanel_, wxID_ANY, wxT("Filter width:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText54->Wrap( -1 );
	fgSizer14->Add( m_staticText54, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pMVEFilterWidthTextCtrl_ = new wxTextCtrl( pDensificationPanel_, ID_MVEFILTERWIDTHTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer14->Add( pMVEFilterWidthTextCtrl_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pMVEFilterWidthSlider_ = new wxSlider( pDensificationPanel_, ID_MVEFILTERWIDTHSLIDER, 2, 1, 5, wxDefaultPosition, wxDefaultSize, wxSL_AUTOTICKS|wxSL_HORIZONTAL );
	pMVEFilterWidthSlider_->SetToolTip( wxT("Patch size for NCC based comparison.\nHigher values produce better results but take longer to compute.") );
	
	fgSizer14->Add( pMVEFilterWidthSlider_, 0, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	pMVEParamsBoxSizer_->Add( fgSizer14, 1, wxEXPAND, 3 );
	
	bSizer44->Add( pMVEParamsBoxSizer_, 0, wxALL|wxEXPAND, 3 );
	
	
	bSizer44->Add( 0, 0, 1, wxEXPAND, 5 );
	
	pStdDialogButtonSizer_ = new wxStdDialogButtonSizer();
	pStdDialogButtonSizer_OK = new wxButton( pDensificationPanel_, wxID_OK );
	pStdDialogButtonSizer_->AddButton( pStdDialogButtonSizer_OK );
	pStdDialogButtonSizer_Cancel = new wxButton( pDensificationPanel_, wxID_CANCEL );
	pStdDialogButtonSizer_->AddButton( pStdDialogButtonSizer_Cancel );
	pStdDialogButtonSizer_->Realize();
	bSizer44->Add( pStdDialogButtonSizer_, 0, wxALL|wxEXPAND, 3 );
	
	pDensificationPanel_->SetSizer( bSizer44 );
	pDensificationPanel_->Layout();
	bSizer44->Fit( pDensificationPanel_ );
	bSizer43->Add( pDensificationPanel_, 1, wxEXPAND, 3 );
	
	this->SetSizer( bSizer43 );
	this->Layout();
	
	this->Centre( wxBOTH );
}

Regard3DDensificationDialogBase::~Regard3DDensificationDialogBase()
{
}

BEGIN_EVENT_TABLE( Regard3DSurfaceDialogBase, wxDialog )
	EVT_INIT_DIALOG( Regard3DSurfaceDialogBase::_wxFB_OnInitDialog )
	EVT_RADIOBOX( ID_SURFACEGENERATIONMETHODRADIOBOX, Regard3DSurfaceDialogBase::_wxFB_OnSurfaceGenerationMethodRadioBox )
	EVT_COMMAND_SCROLL( ID_POISSONDEPTHSLIDER, Regard3DSurfaceDialogBase::_wxFB_OnPoissonDepthSliderScroll )
	EVT_COMMAND_SCROLL( ID_POISSONSAMPLESPERNODESLIDER, Regard3DSurfaceDialogBase::_wxFB_OnPoissonSamplesPerNodeSliderScroll )
	EVT_COMMAND_SCROLL( ID_POISSONPOINTWEIGHTSLIDER, Regard3DSurfaceDialogBase::_wxFB_OnPoissonPointWeightSliderScroll )
	EVT_COMMAND_SCROLL( ID_POISSONTRIMTHRESHOLDSLIDER, Regard3DSurfaceDialogBase::_wxFB_OnPoissonTrimThresholdSliderScroll )
	EVT_COMMAND_SCROLL( ID_FSSRREFINEOCTREELEVELSSLIDER, Regard3DSurfaceDialogBase::_wxFB_OnFSSRRefineOctreeLevelsSliderScroll )
	EVT_COMMAND_SCROLL( ID_FSSRSCALEFACTORMULTIPLIERSLIDER, Regard3DSurfaceDialogBase::_wxFB_OnFSSRScaleFactorMultiplierSliderScroll )
	EVT_COMMAND_SCROLL( ID_FSSRCONFIDENCETHRESHOLDSLIDER, Regard3DSurfaceDialogBase::_wxFB_OnFSSRConfidenceThresholdSliderScroll )
	EVT_COMMAND_SCROLL( ID_FSSRMINCOMPONENTSIZESLIDER, Regard3DSurfaceDialogBase::_wxFB_OnFSSRMinComponentSizeSliderScroll )
	EVT_RADIOBOX( ID_PCOLORIZATIONMETHODRADIOBOX_, Regard3DSurfaceDialogBase::_wxFB_OnColorizationMethodRadioBox )
	EVT_COMMAND_SCROLL( ID_COLVERTNUMBEROFNEIGHBOURSSLIDER, Regard3DSurfaceDialogBase::_wxFB_OnColVertNumberOfNeighboursSliderScroll )
END_EVENT_TABLE()

Regard3DSurfaceDialogBase::Regard3DSurfaceDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxSize( 600,-1 ), wxDefaultSize );
	
	wxBoxSizer* bSizer48;
	bSizer48 = new wxBoxSizer( wxVERTICAL );
	
	pSurfacePanel_ = new wxPanel( this, ID_SURFACEPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer49;
	bSizer49 = new wxBoxSizer( wxVERTICAL );
	
	wxString pSurfaceGenerationMethodRadioBox_Choices[] = { wxT("Poisson surface reconstruction"), wxT("Floating scale surface reconstruction") };
	int pSurfaceGenerationMethodRadioBox_NChoices = sizeof( pSurfaceGenerationMethodRadioBox_Choices ) / sizeof( wxString );
	pSurfaceGenerationMethodRadioBox_ = new wxRadioBox( pSurfacePanel_, ID_SURFACEGENERATIONMETHODRADIOBOX, wxT("Surface generation method"), wxDefaultPosition, wxDefaultSize, pSurfaceGenerationMethodRadioBox_NChoices, pSurfaceGenerationMethodRadioBox_Choices, 1, wxRA_SPECIFY_COLS );
	pSurfaceGenerationMethodRadioBox_->SetSelection( 0 );
	bSizer49->Add( pSurfaceGenerationMethodRadioBox_, 0, wxALL, 3 );
	
	wxStaticBoxSizer* pPoissonParamsBoxSizer_;
	pPoissonParamsBoxSizer_ = new wxStaticBoxSizer( new wxStaticBox( pSurfacePanel_, ID_POISSONPARAMSBOXSIZER, wxT("Poisson surface reconstruction parameters") ), wxVERTICAL );
	
	wxFlexGridSizer* fgSizer11;
	fgSizer11 = new wxFlexGridSizer( 4, 3, 0, 0 );
	fgSizer11->AddGrowableCol( 2 );
	fgSizer11->SetFlexibleDirection( wxHORIZONTAL );
	fgSizer11->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText39 = new wxStaticText( pSurfacePanel_, wxID_ANY, wxT("Depth:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText39->Wrap( -1 );
	fgSizer11->Add( m_staticText39, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pPoissonDepthTextCtrl_ = new wxTextCtrl( pSurfacePanel_, ID_POISSONDEPTHTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer11->Add( pPoissonDepthTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pPoissonDepthSlider_ = new wxSlider( pSurfacePanel_, ID_POISSONDEPTHSLIDER, 9, 6, 20, wxDefaultPosition, wxDefaultSize, wxSL_AUTOTICKS|wxSL_HORIZONTAL );
	fgSizer11->Add( pPoissonDepthSlider_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText52 = new wxStaticText( pSurfacePanel_, wxID_ANY, wxT("Samples per node:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText52->Wrap( -1 );
	fgSizer11->Add( m_staticText52, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pPoissonSamplesPerNodeTextCtrl_ = new wxTextCtrl( pSurfacePanel_, ID_POISSONSAMPLESPERNODETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer11->Add( pPoissonSamplesPerNodeTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pPoissonSamplesPerNodeSlider_ = new wxSlider( pSurfacePanel_, ID_POISSONSAMPLESPERNODESLIDER, 10, 10, 200, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	fgSizer11->Add( pPoissonSamplesPerNodeSlider_, 1, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText40 = new wxStaticText( pSurfacePanel_, wxID_ANY, wxT("Point weight:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText40->Wrap( -1 );
	fgSizer11->Add( m_staticText40, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pPoissonPointWeightTextCtrl_ = new wxTextCtrl( pSurfacePanel_, ID_POISSONPOINTWEIGHTTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer11->Add( pPoissonPointWeightTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pPoissonPointWeightSlider_ = new wxSlider( pSurfacePanel_, ID_POISSONPOINTWEIGHTSLIDER, 40, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	fgSizer11->Add( pPoissonPointWeightSlider_, 1, wxALL|wxEXPAND, 3 );
	
	m_staticText41 = new wxStaticText( pSurfacePanel_, wxID_ANY, wxT("Trim threshold:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText41->Wrap( -1 );
	fgSizer11->Add( m_staticText41, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pPoissonTrimThresholdTextCtrl_ = new wxTextCtrl( pSurfacePanel_, ID_POISSONTRIMTHRESHOLDTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer11->Add( pPoissonTrimThresholdTextCtrl_, 1, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pPoissonTrimThresholdSlider_ = new wxSlider( pSurfacePanel_, ID_POISSONTRIMTHRESHOLDSLIDER, 50, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	fgSizer11->Add( pPoissonTrimThresholdSlider_, 1, wxALIGN_CENTER|wxALL|wxEXPAND, 3 );
	
	pPoissonParamsBoxSizer_->Add( fgSizer11, 0, wxEXPAND, 5 );
	
	bSizer49->Add( pPoissonParamsBoxSizer_, 0, wxALL|wxEXPAND, 3 );
	
	pFSSRParamsBoxSizer_ = new wxStaticBoxSizer( new wxStaticBox( pSurfacePanel_, ID_FSSRPARAMSBOXSIZER, wxT("Floating scale surface reconstruction parameters") ), wxVERTICAL );
	
	wxFlexGridSizer* fgSizer15;
	fgSizer15 = new wxFlexGridSizer( 4, 3, 0, 0 );
	fgSizer15->AddGrowableCol( 2 );
	fgSizer15->SetFlexibleDirection( wxHORIZONTAL );
	fgSizer15->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText55 = new wxStaticText( pSurfacePanel_, wxID_ANY, wxT("Levels:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText55->Wrap( -1 );
	fgSizer15->Add( m_staticText55, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pFSSRRefineOctreeLevelsTextCtrl_ = new wxTextCtrl( pSurfacePanel_, ID_FSSRREFINEOCTREELEVELSTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer15->Add( pFSSRRefineOctreeLevelsTextCtrl_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pFSSRRefineOctreeLevelsSlider_ = new wxSlider( pSurfacePanel_, ID_FSSRREFINEOCTREELEVELSSLIDER, 0, 0, 5, wxDefaultPosition, wxDefaultSize, wxSL_AUTOTICKS|wxSL_HORIZONTAL );
	pFSSRRefineOctreeLevelsSlider_->SetToolTip( wxT("Refines octree with N levels.\nHigher values produce finer meshes but take longer to compute.") );
	
	fgSizer15->Add( pFSSRRefineOctreeLevelsSlider_, 0, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText56 = new wxStaticText( pSurfacePanel_, wxID_ANY, wxT("Scale factor multiplier:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText56->Wrap( -1 );
	fgSizer15->Add( m_staticText56, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pFSSRScaleFactorMultiplierTextCtrl_ = new wxTextCtrl( pSurfacePanel_, ID_FSSRSCALEFACTORMULTIPLIERTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer15->Add( pFSSRScaleFactorMultiplierTextCtrl_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pFSSRScaleFactorMultiplierSlider_ = new wxSlider( pSurfacePanel_, ID_FSSRSCALEFACTORMULTIPLIERSLIDER, 5, 0, 95, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	fgSizer15->Add( pFSSRScaleFactorMultiplierSlider_, 0, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText57 = new wxStaticText( pSurfacePanel_, wxID_ANY, wxT("Confidence threshold:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText57->Wrap( -1 );
	fgSizer15->Add( m_staticText57, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pFSSRConfidenceThresholdTextCtrl_ = new wxTextCtrl( pSurfacePanel_, ID_FSSRCONFIDENCETHRESHOLDTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer15->Add( pFSSRConfidenceThresholdTextCtrl_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pFSSRConfidenceThresholdSlider_ = new wxSlider( pSurfacePanel_, ID_FSSRCONFIDENCETHRESHOLDSLIDER, 10, 10, 200, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	fgSizer15->Add( pFSSRConfidenceThresholdSlider_, 0, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	m_staticText58 = new wxStaticText( pSurfacePanel_, wxID_ANY, wxT("Min. component size:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText58->Wrap( -1 );
	fgSizer15->Add( m_staticText58, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pFSSRMinComponentSizeTextCtrl_ = new wxTextCtrl( pSurfacePanel_, ID_FSSRMINCOMPONENTSIZETEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	fgSizer15->Add( pFSSRMinComponentSizeTextCtrl_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pFSSRMinComponentSizeSlider_ = new wxSlider( pSurfacePanel_, ID_FSSRMINCOMPONENTSIZESLIDER, 16, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	fgSizer15->Add( pFSSRMinComponentSizeSlider_, 0, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 3 );
	
	pFSSRParamsBoxSizer_->Add( fgSizer15, 1, wxEXPAND, 3 );
	
	bSizer49->Add( pFSSRParamsBoxSizer_, 0, wxALL|wxEXPAND, 3 );
	
	wxString pColorizationMethodRadioBox_Choices[] = { wxT("Colored vertices"), wxT("Textures") };
	int pColorizationMethodRadioBox_NChoices = sizeof( pColorizationMethodRadioBox_Choices ) / sizeof( wxString );
	pColorizationMethodRadioBox_ = new wxRadioBox( pSurfacePanel_, ID_PCOLORIZATIONMETHODRADIOBOX_, wxT("Colorization method"), wxDefaultPosition, wxDefaultSize, pColorizationMethodRadioBox_NChoices, pColorizationMethodRadioBox_Choices, 1, wxRA_SPECIFY_COLS );
	pColorizationMethodRadioBox_->SetSelection( 0 );
	bSizer49->Add( pColorizationMethodRadioBox_, 0, wxALL, 5 );
	
	pColVertParamsBoxSizer_ = new wxStaticBoxSizer( new wxStaticBox( pSurfacePanel_, ID_COLVERTPARAMSBOXSIZER, wxT("Parameters for colorizing vertices") ), wxVERTICAL );
	
	wxBoxSizer* bSizer491;
	bSizer491 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText51 = new wxStaticText( pSurfacePanel_, wxID_ANY, wxT("Number of neighbours:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText51->Wrap( -1 );
	bSizer491->Add( m_staticText51, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pColVertNumberOfNeighboursTextCtrl_ = new wxTextCtrl( pSurfacePanel_, ID_COLVERTNUMBEROFNEIGHBOURSTEXTCTRL, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	bSizer491->Add( pColVertNumberOfNeighboursTextCtrl_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pColVertNumberOfNeighboursSlider_ = new wxSlider( pSurfacePanel_, ID_COLVERTNUMBEROFNEIGHBOURSSLIDER, 3, 1, 6, wxDefaultPosition, wxDefaultSize, wxSL_AUTOTICKS|wxSL_HORIZONTAL );
	pColVertNumberOfNeighboursSlider_->SetToolTip( wxT("Determines how many nearest neighbours are used to determine the color of a vertex") );
	
	bSizer491->Add( pColVertNumberOfNeighboursSlider_, 1, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pColVertParamsBoxSizer_->Add( bSizer491, 0, wxEXPAND, 5 );
	
	bSizer49->Add( pColVertParamsBoxSizer_, 0, wxALL|wxEXPAND, 3 );
	
	pTexturizationParamsBoxSizer_ = new wxStaticBoxSizer( new wxStaticBox( pSurfacePanel_, ID_TEXTURIZATIONPARAMSBOXSIZER, wxT("Texturization parameters") ), wxVERTICAL );
	
	wxFlexGridSizer* fgSizer12;
	fgSizer12 = new wxFlexGridSizer( 4, 2, 0, 0 );
	fgSizer12->AddGrowableCol( 1 );
	fgSizer12->SetFlexibleDirection( wxHORIZONTAL );
	fgSizer12->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText42 = new wxStaticText( pSurfacePanel_, wxID_ANY, wxT("Photometric outlier removal:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText42->Wrap( -1 );
	fgSizer12->Add( m_staticText42, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	wxString pTextOutlierRemovalChoice_Choices[] = { wxT("None"), wxT("Gauss clamping"), wxT("Gauss damping") };
	int pTextOutlierRemovalChoice_NChoices = sizeof( pTextOutlierRemovalChoice_Choices ) / sizeof( wxString );
	pTextOutlierRemovalChoice_ = new wxChoice( pSurfacePanel_, ID_TEXTOUTLIERREMOVALCHOICE, wxDefaultPosition, wxDefaultSize, pTextOutlierRemovalChoice_NChoices, pTextOutlierRemovalChoice_Choices, 0 );
	pTextOutlierRemovalChoice_->SetSelection( 0 );
	fgSizer12->Add( pTextOutlierRemovalChoice_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	m_staticText43 = new wxStaticText( pSurfacePanel_, wxID_ANY, wxT("Geometric visibility test:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText43->Wrap( -1 );
	fgSizer12->Add( m_staticText43, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pTextGeomVisTestCheckBox_ = new wxCheckBox( pSurfacePanel_, ID_TEXTGEOMVISTESTCHECKBOX, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	pTextGeomVisTestCheckBox_->SetValue(true); 
	fgSizer12->Add( pTextGeomVisTestCheckBox_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	m_staticText44 = new wxStaticText( pSurfacePanel_, wxID_ANY, wxT("Global seam leveling:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText44->Wrap( -1 );
	fgSizer12->Add( m_staticText44, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pTextGlobalSeamLevCheckBox_ = new wxCheckBox( pSurfacePanel_, ID_TEXTGLOBALSEAMLEVCHECKBOX, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	pTextGlobalSeamLevCheckBox_->SetValue(true); 
	fgSizer12->Add( pTextGlobalSeamLevCheckBox_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	m_staticText45 = new wxStaticText( pSurfacePanel_, wxID_ANY, wxT("Local seam leveling:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText45->Wrap( -1 );
	fgSizer12->Add( m_staticText45, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 3 );
	
	pTextLocalSeamLevCheckBox_ = new wxCheckBox( pSurfacePanel_, ID_TEXTLOCALSEAMLEVCHECKBOX, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	pTextLocalSeamLevCheckBox_->SetValue(true); 
	fgSizer12->Add( pTextLocalSeamLevCheckBox_, 0, wxALIGN_CENTER_VERTICAL|wxALL, 3 );
	
	pTexturizationParamsBoxSizer_->Add( fgSizer12, 0, wxEXPAND, 5 );
	
	bSizer49->Add( pTexturizationParamsBoxSizer_, 0, wxALL|wxEXPAND, 3 );
	
	
	bSizer49->Add( 0, 0, 1, wxEXPAND, 5 );
	
	m_sdbSizer7 = new wxStdDialogButtonSizer();
	m_sdbSizer7OK = new wxButton( pSurfacePanel_, wxID_OK );
	m_sdbSizer7->AddButton( m_sdbSizer7OK );
	m_sdbSizer7Cancel = new wxButton( pSurfacePanel_, wxID_CANCEL );
	m_sdbSizer7->AddButton( m_sdbSizer7Cancel );
	m_sdbSizer7->Realize();
	bSizer49->Add( m_sdbSizer7, 0, wxALL|wxEXPAND, 3 );
	
	pSurfacePanel_->SetSizer( bSizer49 );
	pSurfacePanel_->Layout();
	bSizer49->Fit( pSurfacePanel_ );
	bSizer48->Add( pSurfacePanel_, 1, wxEXPAND, 3 );
	
	this->SetSizer( bSizer48 );
	this->Layout();
	
	this->Centre( wxBOTH );
}

Regard3DSurfaceDialogBase::~Regard3DSurfaceDialogBase()
{
}

BEGIN_EVENT_TABLE( Regard3DPropertiesDialogBase, wxDialog )
	EVT_INIT_DIALOG( Regard3DPropertiesDialogBase::_wxFB_OnInitDialog )
END_EVENT_TABLE()

Regard3DPropertiesDialogBase::Regard3DPropertiesDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxSize( 500,300 ), wxDefaultSize );
	
	wxBoxSizer* bSizer54;
	bSizer54 = new wxBoxSizer( wxVERTICAL );
	
	pPopertiesPanel_ = new wxPanel( this, ID_POPERTIESPANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer55;
	bSizer55 = new wxBoxSizer( wxVERTICAL );
	
	wxStaticBoxSizer* sbSizer23;
	sbSizer23 = new wxStaticBoxSizer( new wxStaticBox( pPopertiesPanel_, wxID_ANY, wxT("Default project path") ), wxVERTICAL );
	
	pDefaultProjectPathDirPicker_ = new wxDirPickerCtrl( pPopertiesPanel_, ID_DEFAULTPROJECTPATHDIRPICKER, wxEmptyString, wxT("Select a folder"), wxDefaultPosition, wxDefaultSize, wxDIRP_DEFAULT_STYLE|wxDIRP_DIR_MUST_EXIST|wxDIRP_USE_TEXTCTRL );
	sbSizer23->Add( pDefaultProjectPathDirPicker_, 1, wxALL|wxEXPAND, 3 );
	
	bSizer55->Add( sbSizer23, 0, wxALL|wxEXPAND, 3 );
	
	wxString pMouseButtonRadioBox_Choices[] = { wxT("Middle mouse button: Zoom, Right mouse button: Move"), wxT("Middle mouse button: Move, Right mouse button: Zoom") };
	int pMouseButtonRadioBox_NChoices = sizeof( pMouseButtonRadioBox_Choices ) / sizeof( wxString );
	pMouseButtonRadioBox_ = new wxRadioBox( pPopertiesPanel_, ID_MOUSEBUTTONRADIOBOX, wxT("Mouse button assignment"), wxDefaultPosition, wxDefaultSize, pMouseButtonRadioBox_NChoices, pMouseButtonRadioBox_Choices, 1, wxRA_SPECIFY_COLS );
	pMouseButtonRadioBox_->SetSelection( 0 );
	bSizer55->Add( pMouseButtonRadioBox_, 0, wxALL|wxEXPAND, 3 );
	
	wxString pMouseWheelRadioBox_Choices[] = { wxT("Mouse wheel up: Zoom in, Mouse wheel down: Zoom out"), wxT("Mouse wheel up: Zoom out, Mouse wheel down: Zoom in") };
	int pMouseWheelRadioBox_NChoices = sizeof( pMouseWheelRadioBox_Choices ) / sizeof( wxString );
	pMouseWheelRadioBox_ = new wxRadioBox( pPopertiesPanel_, ID_MOUSEWHEELRADIOBOX, wxT("Mouse wheel direction"), wxDefaultPosition, wxDefaultSize, pMouseWheelRadioBox_NChoices, pMouseWheelRadioBox_Choices, 1, wxRA_SPECIFY_COLS );
	pMouseWheelRadioBox_->SetSelection( 0 );
	bSizer55->Add( pMouseWheelRadioBox_, 0, wxALL|wxEXPAND, 3 );
	
	
	bSizer55->Add( 0, 0, 1, wxEXPAND, 5 );
	
	m_sdbSizer8 = new wxStdDialogButtonSizer();
	m_sdbSizer8OK = new wxButton( pPopertiesPanel_, wxID_OK );
	m_sdbSizer8->AddButton( m_sdbSizer8OK );
	m_sdbSizer8Cancel = new wxButton( pPopertiesPanel_, wxID_CANCEL );
	m_sdbSizer8->AddButton( m_sdbSizer8Cancel );
	m_sdbSizer8->Realize();
	bSizer55->Add( m_sdbSizer8, 0, wxALL|wxEXPAND, 3 );
	
	pPopertiesPanel_->SetSizer( bSizer55 );
	pPopertiesPanel_->Layout();
	bSizer55->Fit( pPopertiesPanel_ );
	bSizer54->Add( pPopertiesPanel_, 1, wxEXPAND, 3 );
	
	this->SetSizer( bSizer54 );
	this->Layout();
	
	this->Centre( wxBOTH );
}

Regard3DPropertiesDialogBase::~Regard3DPropertiesDialogBase()
{
}
