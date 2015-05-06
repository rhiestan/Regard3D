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
#include "OpenCVHelper.h"


void OpenCVHelper::convertWxImageToCVMat(const wxImage &img, cv::Mat &cvimg)
{
	cvimg.create(img.GetHeight(), img.GetWidth(), CV_8UC3);
	cv::Vec3b *cvPtr = cvimg.ptr<cv::Vec3b>(0);
	unsigned char *wxPtr = img.GetData();
	for(int y = 0; y < img.GetHeight(); y++)
	{
		for(int x = 0; x < img.GetWidth(); x++)
		{
/*			img_cv.at<cv::Vec3b>(y, x)[2] = localPreviewImage.GetRed(x, y);
			img_cv.at<cv::Vec3b>(y, x)[1] = localPreviewImage.GetGreen(x, y);
			img_cv.at<cv::Vec3b>(y, x)[0] = localPreviewImage.GetBlue(x, y);*/
			(*cvPtr)[2] = *(wxPtr++);	// OpenCV usually stores BGR, not RGB
			(*cvPtr)[1] = *(wxPtr++);
			(*cvPtr)[0] = *(wxPtr++);
			cvPtr++;
		}
	}
}

void OpenCVHelper::convertCVMatToWxImage(const cv::Mat &cvimg, wxImage &img)
{
	if(img.GetWidth() != cvimg.cols
		|| img.GetHeight() != cvimg.rows)
	{
		img.Create(cvimg.cols, cvimg.rows, false);
	}
	const cv::Vec3b *cvPtr = cvimg.ptr<cv::Vec3b>(0);
	unsigned char *wxPtr = img.GetData();
	for(int y = 0; y < img.GetHeight(); y++)
	{
		for(int x = 0; x < img.GetWidth(); x++)
		{
/*			localPreviewImage.SetRGB(x, y, img_outcv.at<cv::Vec3b>(y, x)[2],
				img_outcv.at<cv::Vec3b>(y, x)[1],
				img_outcv.at<cv::Vec3b>(y, x)[0]);*/
			*(wxPtr++) = (*cvPtr)[2];
			*(wxPtr++) = (*cvPtr)[1];
			*(wxPtr++) = (*cvPtr)[0];
			cvPtr++;
		}
	}
}
