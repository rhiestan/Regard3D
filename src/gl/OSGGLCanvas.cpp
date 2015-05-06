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
#include "OSGGLCanvas.h"
#include "SharedGLContext.h"
#include "Regard3DMainFrame.h"
#include "Regard3DModelViewHelper.h"

// On OS X, WX_GL_DEPTH_SIZE is mandatory, otherwise bit size of depth buffer is 0
int attribList[] = { WX_GL_DEPTH_SIZE, 24, WX_GL_RGBA, WX_GL_DOUBLEBUFFER, 0};

OSGGLCanvas::OSGGLCanvas(wxWindow *parent, wxWindowID id)
	: wxGLCanvas(parent, id, attribList,
		wxDefaultPosition, wxDefaultSize, wxFULL_REPAINT_ON_RESIZE),
		isOK_(false), pMainFrame_(NULL), pModelViewHelper_(NULL),
		isButtonSwitched_(false), isWheelSwitched_(false)
{
	SharedGLContext *pSharedGLContext = new SharedGLContext(this);
}

OSGGLCanvas::~OSGGLCanvas()
{
	SharedGLContext::cleanup();
}

bool OSGGLCanvas::setup()
{
	if(IsShownOnScreen())
		SetCurrent(SharedGLContext::getInstance());
	return true;
}

void OSGGLCanvas::setGraphicsWindow(GraphicsWindowWX *gw)
{
	graphicsWindow_ = gw;
}

void OSGGLCanvas::setViewer(osgViewer::Viewer *pViewer)
{
	viewer_ = pViewer;
}

void OSGGLCanvas::setFieldOfView(float hfov, float vfov, int minDepth, int maxDepth)
{
	//glCamera_.setRotCenter(0, 0, static_cast<float>(maxDepth - minDepth)/4.0);
}

void OSGGLCanvas::setMainFrame(Regard3DMainFrame *pMainFrame)
{
	pMainFrame_ = pMainFrame;
}

void OSGGLCanvas::setModelViewHelper(Regard3DModelViewHelper *pModelViewHelper)
{
	pModelViewHelper_ = pModelViewHelper;
}

void OSGGLCanvas::cleanup()
{
	SetCurrent(SharedGLContext::getInstance());
	graphicsWindow_->detachFromGLCanvas();
}

void OSGGLCanvas::OnPaint( wxPaintEvent& WXUNUSED(event) )
{
	// must always be here
	wxPaintDC dc(this);

	if(viewer_ == NULL
		|| !isOK_)
		return;
	if(!graphicsWindow_->isRealized())
		return;

	viewer_->frame();
}

void OSGGLCanvas::OnSize(wxSizeEvent& WXUNUSED(event))
{
	int w, h;
	GetClientSize(&w, &h);

	if (graphicsWindow_.valid())
    {
        // update the window dimensions, in case the window has been resized.
        graphicsWindow_->getEventQueue()->windowResize(0, 0, w, h);
        graphicsWindow_->resized(0, 0, w, h);
/*
		// Set correct aspect ratio
		double fovy, aspectRatio, zNear, zFar;
		if(viewer_->getCamera()->getProjectionMatrixAsPerspective(fovy,
			aspectRatio, zNear, zFar))
		{
			viewer_->getCamera()->setProjectionMatrixAsPerspective(fovy, double(w) / double(h), zNear, zFar);
		}*/
		viewer_->getCamera()->setViewport(0, 0, w, h);
    }
}

void OSGGLCanvas::OnEraseBackground(wxEraseEvent& WXUNUSED(event))
{
    // Do nothing, to avoid flashing on MSW
}

void OSGGLCanvas::OnMouse(wxMouseEvent& event)
{
	int w, h;
	GetClientSize(&w, &h);
	wxPoint pt = event.GetPosition();
	int x = pt.x;
	int y = pt.y;	//h - pt.y - 1;
	bool doRepaint = false;

	if (graphicsWindow_.valid())
	{
		if(event.ButtonDown())
		{
			int button = event.GetButton(), buttonOSG = 0, buttonR3D = 0;
			if(button == wxMOUSE_BTN_LEFT)
			{
				buttonOSG = 1;
				buttonR3D = 0;
			}
			else if(button == wxMOUSE_BTN_MIDDLE)
			{
				if(isButtonSwitched_)
					buttonOSG = 2;
				else
					buttonOSG = 3;	// Exchange logic for middle and right mouse button
				buttonR3D = 1;
			}
			else if(button == wxMOUSE_BTN_RIGHT)
			{
				if(isButtonSwitched_)
					buttonOSG = 3;
				else
					buttonOSG = 2;	// Exchange logic for middle and right mouse button
				buttonR3D = 2;
			}
			graphicsWindow_->getEventQueue()->mouseButtonPress(x, y, buttonOSG);
			doRepaint = true;

			if(pModelViewHelper_ != NULL)
				pModelViewHelper_->buttonDown(buttonR3D);

			SetFocus();		// Set focus for mouse wheel
		}

		if(event.Dragging())
		{
			graphicsWindow_->getEventQueue()->mouseMotion(x, y);
			doRepaint = true;
		}
		if(event.ButtonUp())
		{
			int button = event.GetButton(), buttonOSG = 0, buttonR3D = 0;
			if(button == wxMOUSE_BTN_LEFT)
			{
				buttonOSG = 1;
				buttonR3D = 0;
			}
			else if(button == wxMOUSE_BTN_MIDDLE)
			{
				if(isButtonSwitched_)
					buttonOSG = 2;
				else
					buttonOSG = 3;	// Exchange logic for middle and right mouse button
				buttonR3D = 1;
			}
			else if(button == wxMOUSE_BTN_RIGHT)
			{
				if(isButtonSwitched_)
					buttonOSG = 3;
				else
					buttonOSG = 2;	// Exchange logic for middle and right mouse button
				buttonR3D = 2;
			}
			graphicsWindow_->getEventQueue()->mouseButtonRelease(x, y, buttonOSG);
			doRepaint = true;

			if(pModelViewHelper_ != NULL)
				pModelViewHelper_->buttonUp(buttonR3D);
		}

		int wheelRotation = event.GetWheelRotation();
		if(wheelRotation != 0)
		{
		    //int delta = wheelRotation / event.GetWheelDelta() * event.GetLinesPerAction();
			if(isWheelSwitched_)
			{
				graphicsWindow_->getEventQueue()->mouseScroll(
					wheelRotation > 0 ?	//delta<0 ? // orig: delta>0 ? 
					osgGA::GUIEventAdapter::SCROLL_UP : 
					osgGA::GUIEventAdapter::SCROLL_DOWN);
			}
			else
			{
				graphicsWindow_->getEventQueue()->mouseScroll(
					wheelRotation < 0 ?	//delta<0 ? // orig: delta>0 ? 
					osgGA::GUIEventAdapter::SCROLL_UP : 
					osgGA::GUIEventAdapter::SCROLL_DOWN);
			}
			doRepaint = true;
		}
	}
	if(doRepaint)
	{
		Refresh();
	}
}

void OSGGLCanvas::OnChar(wxKeyEvent &event)
{
#if wxUSE_UNICODE
    int key = event.GetUnicodeKey();
#else
    int key = event.GetKeyCode();
#endif

    if (graphicsWindow_.valid())
        graphicsWindow_->getEventQueue()->keyPress(key);

    // If this key event is not processed here, we should call
    // event.Skip() to allow processing to continue.
}

void OSGGLCanvas::OnKeyUp(wxKeyEvent &event)
{
#if wxUSE_UNICODE
    int key = event.GetUnicodeKey();
#else
    int key = event.GetKeyCode();
#endif

    if (graphicsWindow_.valid())
        graphicsWindow_->getEventQueue()->keyRelease(key);

    // If this key event is not processed here, we should call
    // event.Skip() to allow processing to continue.
}

void OSGGLCanvas::UseCursor(bool value)
{
    if (value)
    {
        // show the old cursor
        SetCursor(_oldCursor);
    }
    else
    {
        // remember the old cursor
        _oldCursor = GetCursor();

        // hide the cursor
        //    - can't find a way to do this neatly, so create a 1x1, transparent image
        wxImage image(1,1);
        image.SetMask(true);
        image.SetMaskColour(0, 0, 0);
        wxCursor cursor(image);
        SetCursor(cursor);

        // On wxGTK, only works as of version 2.7.0
        // (http://trac.wxwidgets.org/ticket/2946)
        // SetCursor( wxStockCursor( wxCURSOR_BLANK ) );
    }
}

void OSGGLCanvas::setOK()
{
	isOK_ = true;
}

void OSGGLCanvas::setIsMouseButtonSwitched(bool isSwitched)
{
	isButtonSwitched_ = isSwitched;
}

void OSGGLCanvas::setIsMouseWheelSwitched(bool isSwitched)
{
	isWheelSwitched_ = isSwitched;
}

BEGIN_EVENT_TABLE( OSGGLCanvas, wxGLCanvas )
	EVT_SIZE(OSGGLCanvas::OnSize)
	EVT_PAINT(OSGGLCanvas::OnPaint)
	EVT_ERASE_BACKGROUND(OSGGLCanvas::OnEraseBackground)
	EVT_MOUSE_EVENTS(OSGGLCanvas::OnMouse)
	EVT_CHAR                (OSGGLCanvas::OnChar)
	EVT_KEY_UP              (OSGGLCanvas::OnKeyUp)
END_EVENT_TABLE()
