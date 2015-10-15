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
#include "ImagePanel.h"
#include "OpenCVHelper.h"
#include <cmath>

// For double buffering
#include <wx/dcbuffer.h>

// OpenCV
#include "opencv2/imgproc/imgproc.hpp"

// Workaround for issue http://trac.wxwidgets.org/ticket/15684
// Is fixed in trunk, will probably be in 3.1
//#define R3D_USE_SCROLLBAR_MOUSEWHEEL_WORKAROUND

const int ImagePanel::pixelsPerScrollUnit_ = 10;

ImagePanel::ImagePanel(wxWindow *pParent)
	: wxScrolledWindow(pParent),
	zoomFactor_(1.0)
{
#if wxCHECK_VERSION(2, 9, 0)
	SetBackgroundStyle(wxBG_STYLE_PAINT);
#else
	SetBackgroundStyle(wxBG_STYLE_CUSTOM);
#endif
}

ImagePanel::~ImagePanel()
{
}

void ImagePanel::setImage(const wxImage &img)
{
	img_ = img;

	// Convert image to OpenCV
	OpenCVHelper::convertWxImageToCVMat(img, imgCV_);
/*	imgCV_.create(img.GetHeight(), img.GetWidth(), CV_8UC3);
	cv::Vec3b *cvPtr = imgCV_.ptr<cv::Vec3b>(0);
	unsigned char *wxPtr = img.GetData();
	for(int y = 0; y < img.GetHeight(); y++)
	{
		for(int x = 0; x < img.GetWidth(); x++)
		{
			(*cvPtr)[2] = *(wxPtr++);	// OpenCV usually stores BGR, not RGB
			(*cvPtr)[1] = *(wxPtr++);
			(*cvPtr)[0] = *(wxPtr++);
			cvPtr++;
		}
	}*/
}

void ImagePanel::setZoomFactor(double newZoomFactor, int centerXIn, int centerYIn, bool isZoomIn)
{
	// Determine center of displayed image
	int viewStartX = 0, viewStartY = 0;
	GetViewStart(&viewStartX, &viewStartY);
	int viewWidth = 0, viewHeight = 0;
	GetClientSize(&viewWidth, &viewHeight);

	bool mouseWheel = false;
	int centerX = centerXIn, centerY = centerYIn;
	if(centerX < 0 && centerY < 0)
	{
		// Use center of visible image
		centerX = static_cast<double>(viewStartX * pixelsPerScrollUnit_)
			+ static_cast<double>(viewWidth) / 2.0;
		centerY = static_cast<double>(viewStartY * pixelsPerScrollUnit_)
			+ static_cast<double>(viewHeight) / 2.0;
	}
	else
	{
		mouseWheel = true;
		// Use centerXIn/centerYIn (mouse cursor)
		int xx, yy;
		CalcUnscrolledPosition(centerXIn, centerYIn, &xx, &yy);
		centerX = xx;
		centerY = yy;
	}

	int centerXOffset =	centerX - viewStartX * pixelsPerScrollUnit_;	// in screen coordinates
	int centerYOffset =	centerY - viewStartY * pixelsPerScrollUnit_;	// in screen coordinates
	double centerX_OrigZoom = centerX / zoomFactor_;	// In pixels of original image
	double centerY_OrigZoom = centerY / zoomFactor_;

	zoomFactor_ = newZoomFactor;

	// Calculate virtual image size
	double virtualWidth = zoomFactor_ * static_cast<double>(img_.GetWidth());
	double virtualHeight = zoomFactor_ * static_cast<double>(img_.GetHeight());

	int virtualWidthInt = static_cast<int>( virtualWidth + 0.5 );
	int virtualHeightInt = static_cast<int>( virtualHeight + 0.5 );

	// Convert to scroll units, rounding up
	int virtualWidthSU = (virtualWidthInt + pixelsPerScrollUnit_ - 1) / pixelsPerScrollUnit_;
	int virtualHeightSU = (virtualHeightInt + pixelsPerScrollUnit_ - 1) / pixelsPerScrollUnit_;
/*
	char buf[1024];
	sprintf_s(buf, 1024, "sz: %f %f %d %d %d %d %d %d\n", virtualWidth, virtualHeight, 
		virtualWidthInt, virtualHeightInt, virtualWidthSU, virtualHeightSU,
		virtualWidthSU * pixelsPerScrollUnit_, virtualHeightSU * pixelsPerScrollUnit_);
	OutputDebugStringA(buf);
*/
	// Scroll such that the previous center is still centered (or, the pixel under the mouse cursor remains there)
	double newCenterX = zoomFactor_ * centerX_OrigZoom;		// In pixels of zoomed image
	double newCenterY = zoomFactor_ * centerY_OrigZoom;
//	double newViewStartX_px = newCenterX - static_cast<double>(viewWidth) / 2.0;
//	double newViewStartY_px = newCenterY - static_cast<double>(viewHeight) / 2.0;
	double newViewStartX_px = newCenterX - centerXOffset;
	double newViewStartY_px = newCenterY - centerYOffset;
	int newViewStartX = static_cast<int>(0.5 + newViewStartX_px / static_cast<double>(pixelsPerScrollUnit_));
	int newViewStartY = static_cast<int>(0.5 + newViewStartY_px / static_cast<double>(pixelsPerScrollUnit_));

#if defined(R3D_USE_SCROLLBAR_MOUSEWHEEL_WORKAROUND)
	if(mouseWheel)
	{
		if(isZoomIn)
			newViewStartY = std::min(newViewStartY + 3, virtualHeightSU - 1);
		else
			newViewStartY = std::max(newViewStartY - 3, 0);
	}
#endif

	SetScrollbars(pixelsPerScrollUnit_, pixelsPerScrollUnit_,
		virtualWidthSU, virtualHeightSU, newViewStartX, newViewStartY);
}

void ImagePanel::OnPaint( wxPaintEvent& event )
{
	wxAutoBufferedPaintDC dc(this);
	DoPrepareDC(dc);

	dc.Clear();

	if(!img_.IsOk())
		return;

	// Determine visible area of the scrolled window
	int viewStartX = 0, viewStartY = 0;
	GetViewStart(&viewStartX, &viewStartY);
	viewStartX *= pixelsPerScrollUnit_;		// Convert to pixels
	viewStartY *= pixelsPerScrollUnit_;		// Convert to pixels
	int viewWidth = 0, viewHeight = 0;
	GetClientSize(&viewWidth, &viewHeight);
	int viewStartXOrig = static_cast<int>(static_cast<double>(viewStartX) / zoomFactor_);
	int viewStartYOrig = static_cast<int>(static_cast<double>(viewStartY) / zoomFactor_);
	int viewWidthOrig = static_cast<int>( static_cast<double>(viewWidth + 2) / zoomFactor_) + 2;	// + 2 for margins
	int viewHeightOrig = static_cast<int>( static_cast<double>(viewHeight + 2) / zoomFactor_) + 2;	// (one for zoom > 1, one for zoom < 1)


/*
	char buf[1024];
	sprintf_s(buf, 1024, "OnPaint: %d %d %d %d\n", viewStartX, viewStartY, viewWidth, viewHeight);
	OutputDebugStringA(buf);
*/
	// Calculate virtual image size
	double virtualWidth = zoomFactor_ * static_cast<double>(img_.GetWidth());
	double virtualHeight = zoomFactor_ * static_cast<double>(img_.GetHeight());

//	int virtualWidthInt = static_cast<int>( virtualWidth + 0.5 );
//	int virtualHeightInt = static_cast<int>( virtualHeight + 0.5 );

	// Resize image
	int interp = cv::INTER_NEAREST;
	if(zoomFactor_ < 1.0)
		interp = cv::INTER_AREA;
	wxImage scaledImage;
	cv::Mat scaledImageCV;
	
	if(img_.GetWidth() > viewWidthOrig
		|| img_.GetHeight() > viewHeightOrig)
	{
		viewStartXOrig = std::max(0, viewStartXOrig);
		viewStartYOrig = std::max(0, viewStartYOrig);

		// Get only part of the image
		viewWidthOrig = std::min( viewWidthOrig, img_.GetWidth() - viewStartXOrig );		// One dimension might still be bigger than the image
		viewHeightOrig = std::min( viewHeightOrig, img_.GetHeight() - viewStartYOrig );
		viewWidth = static_cast<int>( static_cast<double>(viewWidthOrig) * zoomFactor_ + 0.5 );
		viewHeight = static_cast<int>( static_cast<double>(viewHeightOrig) * zoomFactor_ + 0.5 );
//		wxImage partImage = img_.GetSubImage(wxRect(viewStartXOrig, viewStartYOrig, viewWidthOrig, viewHeightOrig));
//		scaledImage = partImage.Scale(viewWidth, viewHeight);

		cv::Mat partImage(imgCV_, cv::Rect(viewStartXOrig, viewStartYOrig, viewWidthOrig, viewHeightOrig));
		cv::resize(partImage, scaledImageCV, cv::Size(viewWidth, viewHeight), 0, 0, interp);

		viewStartX = static_cast<int>(static_cast<double>(viewStartXOrig) * zoomFactor_);
		viewStartY = static_cast<int>(static_cast<double>(viewStartYOrig) * zoomFactor_);
	}
	else
	{
		int newWidth = static_cast<int>(zoomFactor_ * static_cast<double>(img_.GetWidth()) + 0.5);
		int newHeight = static_cast<int>(zoomFactor_ * static_cast<double>(img_.GetHeight()) + 0.5);
		//scaledImage = img_.Scale(newWidth, newHeight, quality);
		cv::resize(imgCV_, scaledImageCV, cv::Size(newWidth, newHeight), 0, 0, interp);
	}

	OpenCVHelper::convertCVMatToWxImage(scaledImageCV, scaledImage);
/*
	// Copy back to wxImage
	scaledImage.Create(scaledImageCV.cols, scaledImageCV.rows);
	cv::Vec3b *cvPtr = scaledImageCV.ptr<cv::Vec3b>(0);
	unsigned char *wxPtr = scaledImage.GetData();
	for(int y = 0; y < scaledImage.GetHeight(); y++)
	{
		for(int x = 0; x < scaledImage.GetWidth(); x++)
		{
			*(wxPtr++) = (*cvPtr)[2];
			*(wxPtr++) = (*cvPtr)[1];
			*(wxPtr++) = (*cvPtr)[0];
			cvPtr++;
		}
	}
*/
	wxBitmap bmp(scaledImage);	// Convert to wxBitmap
	dc.DrawBitmap(bmp, viewStartX, viewStartY);	// Draw bitmap

/*
	char buf[1024];
	sprintf_s(buf, 1024, "op: %d %d %d %d\n", viewStartX, viewStartY, 
		viewStartX + scaledImage.GetWidth(), viewStartY + scaledImage.GetHeight());
	OutputDebugStringA(buf);
*/
}

void ImagePanel::OnEraseBackground(wxEraseEvent& WXUNUSED(event))
{
}

void ImagePanel::OnMouseWheel(wxMouseEvent& event)
{
	// Propagate to parent
	event.ResumePropagation(2);
	event.Skip();
}

BEGIN_EVENT_TABLE(ImagePanel, wxScrolledWindow)
	EVT_PAINT(ImagePanel::OnPaint)
	EVT_ERASE_BACKGROUND(ImagePanel::OnEraseBackground)
	EVT_MOUSEWHEEL(ImagePanel::OnMouseWheel)
END_EVENT_TABLE()
