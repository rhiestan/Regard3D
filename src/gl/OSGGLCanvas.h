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
#ifndef OSGGLCANVAS_H
#define OSGGLCANVAS_H

class Regard3DMainFrame;
class Regard3DModelViewHelper;

#include "GraphicsWindowWX.h"

class OSGGLCanvas: public wxGLCanvas
{
public:
	OSGGLCanvas(wxWindow *parent,
		wxWindowID id = wxID_ANY);
	virtual ~OSGGLCanvas();

	bool setup();
    void setGraphicsWindow(GraphicsWindowWX *gw);
	void setViewer(osgViewer::Viewer *pViewer);
	void setFieldOfView(float hfov, float vfov, int minDepth, int maxDepth);
	void setMainFrame(Regard3DMainFrame *pMainFrame);
	void setModelViewHelper(Regard3DModelViewHelper *pModelViewHelper);
	void cleanup();

	void OnPaint( wxPaintEvent& event );
	void OnSize(wxSizeEvent& event);
	void OnEraseBackground(wxEraseEvent& WXUNUSED(event));

	void OnMouse(wxMouseEvent& event);

    void OnChar(wxKeyEvent &event);
    void OnKeyUp(wxKeyEvent &event);

	void UseCursor(bool value);

	void setOK();

	void setIsMouseButtonSwitched(bool isSwitched);
	void setIsMouseWheelSwitched(bool isSwitched);

private:
	osg::ref_ptr<GraphicsWindowWX> graphicsWindow_;
	osg::ref_ptr<osgViewer::Viewer> viewer_;

    wxCursor _oldCursor;
	bool isOK_;

	Regard3DMainFrame *pMainFrame_;
	Regard3DModelViewHelper *pModelViewHelper_;

	bool isButtonSwitched_, isWheelSwitched_;

	DECLARE_EVENT_TABLE()
};

#endif
