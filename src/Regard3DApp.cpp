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
#include "CommonIncludes.h"
#include "version.h"

#include "Regard3DApp.h"
#include "Regard3DMainFrame.h"
#include "CameraDBLookup.h"
#include "UserCameraDB.h"
#include "R3DExternalPrograms.h"
#include "R3DFontHandler.h"

#include <Eigen/Core>

#if defined(R3D_HAVE_TBB)
//#	include "tbb/tbbmalloc_proxy.h"
#endif

#include <osgDB/Registry>

#if defined(OSG_LIBRARY_STATIC)
	// Add references for static linking some plugins
	USE_OSGPLUGIN(ive)
	USE_OSGPLUGIN(osg)
	USE_OSGPLUGIN(osg2)
	USE_OSGPLUGIN(rgb)
//	USE_OSGPLUGIN(OpenFlight)
	USE_OSGPLUGIN(freetype)
	USE_OSGPLUGIN(p3d)
	USE_OSGPLUGIN(obj)
	USE_OSGPLUGIN(osgViewer)
	USE_OSGPLUGIN(ply)
	USE_OSGPLUGIN(png)
	USE_OSGPLUGIN(jpeg)
//	USE_OSGPLUGIN(vrml)

	
	USE_DOTOSGWRAPPER_LIBRARY(osg)
//	USE_DOTOSGWRAPPER_LIBRARY(osgFX)
//	USE_DOTOSGWRAPPER_LIBRARY(osgParticle)
//	USE_DOTOSGWRAPPER_LIBRARY(osgShadow)
//	USE_DOTOSGWRAPPER_LIBRARY(osgSim)
//	USE_DOTOSGWRAPPER_LIBRARY(osgTerrain)
	USE_DOTOSGWRAPPER_LIBRARY(osgText)
	USE_DOTOSGWRAPPER_LIBRARY(osgViewer)
//	USE_DOTOSGWRAPPER_LIBRARY(osgVolume)
//	USE_DOTOSGWRAPPER_LIBRARY(osgWidget)
	
	USE_SERIALIZER_WRAPPER_LIBRARY(osg)
//	USE_SERIALIZER_WRAPPER_LIBRARY(osgAnimation)
//	USE_SERIALIZER_WRAPPER_LIBRARY(osgFX)
	USE_SERIALIZER_WRAPPER_LIBRARY(osgManipulator)
//	USE_SERIALIZER_WRAPPER_LIBRARY(osgParticle)
//	USE_SERIALIZER_WRAPPER_LIBRARY(osgShadow)
//	USE_SERIALIZER_WRAPPER_LIBRARY(osgSim)
//	USE_SERIALIZER_WRAPPER_LIBRARY(osgTerrain)
	USE_SERIALIZER_WRAPPER_LIBRARY(osgText)
//	USE_SERIALIZER_WRAPPER_LIBRARY(osgVolume)
#endif


#if defined(R3D_WIN32)
#include <windows.h>

/**
 * Give this process a console of its own, hidden.
 *
 * OpenMVG's global SfM engine renders the graphs of its HTML report by calling
 * std::system("neato ..."), and on Windows std::system runs the command through
 * cmd.exe. A GUI application has no console, so every one of those calls made
 * cmd.exe allocate one, and a console window flashed up during triangulation.
 * With a console already present the child processes attach to ours instead of
 * creating their own, so nothing becomes visible.
 */
static void allocateHiddenConsole()
{
	// When started from a terminal that console already belongs to us. It is the
	// user's window, so it must be left alone - and it suppresses the flash anyway.
	if(GetConsoleWindow() != NULL)
		return;

	if(AllocConsole() == 0)
		return;

	HWND consoleWnd = GetConsoleWindow();
	if(consoleWnd != NULL)
		ShowWindow(consoleWnd, SW_HIDE);
}
#endif

/**
 * Appends a directory to this process' search path.
 *
 * Appended rather than prepended on purpose: the Graphviz directory also holds
 * its own DLLs, and PATH takes part in the DLL search, so putting it in front
 * could let those shadow libraries loaded later on. This also matches how
 * R3DDensificationProcess extends the path of the external tools.
 */
static void appendToSearchPath(const wxString &dir)
{
	if(dir.IsEmpty())
		return;

#if defined(R3D_WIN32)
	const wxString separator(wxT(";"));
#else
	const wxString separator(wxT(":"));
#endif

	wxString path;
	if(wxGetEnv(wxT("PATH"), &path) && !path.IsEmpty())
		wxSetEnv(wxT("PATH"), path + separator + dir);
	else
		wxSetEnv(wxT("PATH"), dir);
}

Regard3DApp::Regard3DApp() : wxApp()
{
#if defined(__WXX11__) || defined(__WXGTK__)
	XInitThreads();
#endif
}

// Main program equivalent, creating windows and returning main app frame
bool Regard3DApp::OnInit()
{
	wxString appName(wxT(REGARD3D_NAME));
	wxString vendorName(wxT(REGARD3D_VENDOR_NAME));

#if wxCHECK_VERSION(2, 9, 0)
	SetAppDisplayName(appName);
	SetVendorDisplayName(vendorName);
#endif
	appName.Replace(wxT(" "), wxEmptyString, true);
	SetAppName(appName);
	SetVendorName(vendorName);

#if wxUSE_IMAGE
	wxInitAllImageHandlers();
#endif

	Eigen::initParallel();

#if defined(R3D_HAVE_OPENMP)
	//omp_set_num_threads(1);	// omp_get_num_procs() + 1);	// +1 to fill delays in file I/O
#endif

#if defined(R3D_WIN32)
	allocateHiddenConsole();
#endif

	CameraDBLookup::getInstance().initialize();
	UserCameraDB::getInstance().initialize();
	R3DExternalPrograms::getInstance().initialize();

	// OpenMVG calls "neato" by name, so the bundled Graphviz has to be
	// reachable from this process, not just from the external tools
	appendToSearchPath(R3DExternalPrograms::getInstance().getGraphvizPath());
	R3DFontHandler::getInstance().initialize();

	pMainFrame_ = new Regard3DMainFrame(NULL);

	// Show the frame
	pMainFrame_->Show(true);

	SetTopWindow(pMainFrame_);

	return true;
}

int Regard3DApp::OnExit()
{
	return 0;
}

IMPLEMENT_APP(Regard3DApp)
