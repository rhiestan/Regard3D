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
#include "GraphicsWindowWX.h"
#include "OSGGLCanvas.h"
#include "SharedGLContext.h"
#include <osg/Version>


// This code is based on OpenSceneGraph's example osgviewerWX.cpp.
GraphicsWindowWX::GraphicsWindowWX(OSGGLCanvas *canvas)
	: initialized_(false), valid_(false), realized_(false)
{
	pGLCanvas_ = canvas;

	_traits = new GraphicsContext::Traits;

	wxPoint pos = pGLCanvas_->GetPosition();
	wxSize  size = pGLCanvas_->GetSize();

	_traits->x = pos.x;
	_traits->y = pos.y;
	_traits->width = size.x;
	_traits->height = size.y;

	init();
}

GraphicsWindowWX::~GraphicsWindowWX()
{
}

void GraphicsWindowWX::init()
{
	if(initialized_) return;

	if (valid())
	{
		setState( new osg::State );
		getState()->setGraphicsContext(this);

#if OSG_VERSION_GREATER_OR_EQUAL(3, 2, 0)
		if (_traits.valid() && _traits->sharedContext.valid())
#else
		if (_traits.valid() && _traits->sharedContext)
#endif
		{
			getState()->setContextID( _traits->sharedContext->getState()->getContextID() );
			incrementContextIDUsageCount( getState()->getContextID() );
		}
		else
		{
			getState()->setContextID( osg::GraphicsContext::createNewContextID() );
		}

		//getEventQueue()->syncWindowRectangleWithGraphcisContext();
	}

	initialized_ = true;
	valid_ = initialized_;
}

void GraphicsWindowWX::detachFromGLCanvas()
{
	pGLCanvas_ = NULL;
}

void GraphicsWindowWX::grabFocus()
{
	// focus the canvas
	pGLCanvas_->SetFocus();
}

void GraphicsWindowWX::grabFocusIfPointerInWindow()
{
	// focus this window, if the pointer is in the window
	wxPoint pos;
	if(wxFindWindowAtPointer(pos) == pGLCanvas_)
		pGLCanvas_->SetFocus();
}

void GraphicsWindowWX::useCursor(bool cursorOn)
{
	pGLCanvas_->UseCursor(cursorOn);
}

bool GraphicsWindowWX::makeCurrentImplementation()
{
	if(pGLCanvas_ == NULL 
		|| !pGLCanvas_->IsShownOnScreen())
		return false;

	pGLCanvas_->SetCurrent(SharedGLContext::getInstance());
	return true;
}

void GraphicsWindowWX::swapBuffersImplementation()
{
	if(!realized_ || pGLCanvas_ == NULL)
		return;

	GLint depthBits;
	glGetIntegerv(GL_DEPTH_BITS, &depthBits);
	int j = 27;

	pGLCanvas_->SwapBuffers();
}

bool GraphicsWindowWX::isRealizedImplementation() const
{
//	return pGLCanvas_->IsShownOnScreen();
	return realized_;
}

bool GraphicsWindowWX::valid() const
{
	return (pGLCanvas_ != NULL);
}

bool GraphicsWindowWX::realizeImplementation()
{
	if(pGLCanvas_ == NULL)
		return false;
	if(realized_)
		return true;

	pGLCanvas_->Show();

	realized_ = true;
	return true;
}

void GraphicsWindowWX::closeImplementation()
{
	initialized_ = false;
	valid_       = false;
	realized_    = false;
}

bool GraphicsWindowWX::releaseContextImplementation()
{
	if(pGLCanvas_ == NULL 
		|| !pGLCanvas_->IsShownOnScreen())
		return false;

	// Set no context as active (to allow context to be used in other thread)
	SharedGLContext::setNoActiveContext(pGLCanvas_);

	return true;
}
