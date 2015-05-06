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
#ifndef GRAPHICS_WINDOW_WX_H
#define GRAPHICS_WINDOW_WX_H

class OSGGLCanvas;

#include <osgViewer/Viewer>

class GraphicsWindowWX : public osgViewer::GraphicsWindow
{
public:
	GraphicsWindowWX(OSGGLCanvas *canvas);
	~GraphicsWindowWX();

	void init();
	void detachFromGLCanvas();

	//
	// GraphicsWindow interface
	//
	void grabFocus();
	void grabFocusIfPointerInWindow();
	void useCursor(bool cursorOn);

	bool makeCurrentImplementation();
	void swapBuffersImplementation();

	virtual bool valid() const;
	virtual bool realizeImplementation();
	virtual bool isRealizedImplementation() const;
	virtual void closeImplementation();
	virtual bool releaseContextImplementation();

private:
	OSGGLCanvas*  pGLCanvas_;

	bool initialized_, valid_, realized_;
};

#endif
