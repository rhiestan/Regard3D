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

#ifndef PREVIEWCANVAS_H
#define PREVIEWCANVAS_H

class PreviewCanvas: public wxWindow
{
public:
	PreviewCanvas(wxWindow *pParent);
	virtual ~PreviewCanvas();

	void previewImage(const wxImage &image);
	void clearPreviewImage();
	bool getIsPreviewImageValid() const { return (isPreviewImageValid_ && previewImage_.IsOk()); }
	const wxImage &getPreviewImage() const { return previewImage_; }

protected:
	void OnPaint(wxPaintEvent& event);

private:

	bool isPreviewImageValid_;
	wxImage previewImage_;

	DECLARE_EVENT_TABLE()
};

#endif
