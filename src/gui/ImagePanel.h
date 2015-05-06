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

#ifndef IMAGEPANEL_H
#define IMAGEPANEL_H

// OpenCV
#include "opencv2/core/core.hpp"

class ImagePanel: public wxScrolledWindow
{
public:
	ImagePanel(wxWindow *pParent);
	virtual ~ImagePanel();

	void setImage(const wxImage &img);
	double getZoomFactor() { return zoomFactor_; }
	void setZoomFactor(double newZoomFactor, int centerX = -1, int centerY = -1, bool isZoomIn = false);

protected:
	virtual void OnPaint( wxPaintEvent& event );
	virtual void OnEraseBackground(wxEraseEvent& event);

	virtual void OnMouseWheel(wxMouseEvent& event);

private:
	wxImage img_;
	cv::Mat imgCV_;
	double zoomFactor_;

	static const int pixelsPerScrollUnit_;

	DECLARE_EVENT_TABLE()
};


#endif
