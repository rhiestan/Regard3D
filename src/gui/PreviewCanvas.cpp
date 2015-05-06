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
#include "PreviewCanvas.h"
#include <cmath>

// For double buffering
#include <wx/dcbuffer.h>

PreviewCanvas::PreviewCanvas(wxWindow *pParent)
	: wxWindow(pParent, wxID_ANY),
	isPreviewImageValid_(false)
{
	// We draw the background by ourselves
#if wxCHECK_VERSION(2, 9, 0)
	SetBackgroundStyle(wxBG_STYLE_PAINT);
#endif
}

PreviewCanvas::~PreviewCanvas()
{
}

void PreviewCanvas::OnPaint(wxPaintEvent& WXUNUSED(event))
{
	wxAutoBufferedPaintDC dc(this);
	PrepareDC(dc);

	dc.Clear();

	if(!isPreviewImageValid_
		|| !previewImage_.IsOk())
		return;

	// Get dimensions of the canvas
	int w, h;
	GetClientSize(&w, &h);
	
	// Resize image
	double widthFactor = static_cast<double>(w) / static_cast<double>(previewImage_.GetWidth());
	double heightFactor = static_cast<double>(h) / static_cast<double>(previewImage_.GetHeight());
	double scaleFactor = heightFactor;

	if(widthFactor < heightFactor)
		scaleFactor = widthFactor;

	int newWidth = static_cast<int>(std::floor(scaleFactor * static_cast<double>(previewImage_.GetWidth())) + 0.5);
	int newHeight = static_cast<int>(std::floor(scaleFactor * static_cast<double>(previewImage_.GetHeight())) + 0.5);

	wxImage scaledImage = previewImage_.Scale(newWidth, newHeight, wxIMAGE_QUALITY_HIGH);

	wxBitmap bmp(scaledImage);	// Convert to wxBitmap
	dc.DrawBitmap(bmp, 0, 0);	// Draw bitmap
}

void PreviewCanvas::previewImage(const wxImage &image)
{
	previewImage_ = image;
	isPreviewImageValid_ = true;
	Refresh();
}

void PreviewCanvas::clearPreviewImage()
{
	if(previewImage_.IsOk())
		previewImage_.Destroy();
	isPreviewImageValid_ = false;
	Refresh();
}

BEGIN_EVENT_TABLE(PreviewCanvas, wxWindow)
	EVT_PAINT(PreviewCanvas::OnPaint)
END_EVENT_TABLE()
