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

// OpenCV Includes
#include "opencv2/core/eigen.hpp" //To Convert Eigen matrix to cv matrix
#include "opencv2/imgproc/imgproc.hpp"

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
	if(!img.IsOk()
		|| img.GetWidth() != cvimg.cols
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

openMVG::image::Image<openMVG::image::RGBColor> OpenCVHelper::createThumbnail(
	const openMVG::image::Image<openMVG::image::RGBColor> &orig, int thWidth, int thHeight)
{
	const int width = orig.Width();
	const int height = orig.Height();
	const float image_aspect = static_cast<float>(width) / static_cast<float>(height);
	const float thumb_aspect = static_cast<float>(thWidth) / static_cast<float>(thHeight);

	int rescale_width, rescale_height;
	if(image_aspect > thumb_aspect)
	{
		rescale_width = std::ceil(thHeight * image_aspect);
		rescale_height = thHeight;
	}
	else
	{
		rescale_width = thWidth;
		rescale_height = std::ceil(thWidth / image_aspect);
	}

	// Convert image to OpenCV data
	cv::Mat cvimg(height, width, CV_8UC3);
	cv::Vec3b *cvPtr = cvimg.ptr<cv::Vec3b>(0);
	for(int y = 0; y < height; y++)
	{
		for(int x = 0; x < width; x++)
		{
			(*cvPtr)[2] = orig(y, x).r();	// OpenCV usually stores BGR, not RGB
			(*cvPtr)[1] = orig(y, x).g();
			(*cvPtr)[0] = orig(y, x).b();
			cvPtr++;
		}
	}

	// Resize
	cv::Mat cvoutimg;
	cv::resize(cvimg, cvoutimg, cv::Size(thWidth, thHeight), 0, 0, cv::INTER_AREA);

	// Convert back to openMVG::image
	cvPtr = cvoutimg.ptr<cv::Vec3b>(0);
	openMVG::image::Image<openMVG::image::RGBColor> outImg;
	outImg.resize(thWidth, thHeight);
	for(int y = 0; y < thHeight; y++)
	{
		for(int x = 0; x < thWidth; x++)
		{
			outImg(y, x).r() = (*cvPtr)[2];	// OpenCV usually stores BGR, not RGB
			outImg(y, x).g() = (*cvPtr)[1];
			outImg(y, x).b() = (*cvPtr)[0];
			cvPtr++;
		}
	}

	return outImg;
}
