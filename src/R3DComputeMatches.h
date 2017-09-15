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
#ifndef R3DCOMPUTEMATCHES_H
#define R3DCOMPUTEMATCHES_H

class Regard3DMainFrame;

#include "ImageInfo.h"
#include "Regard3DFeatures.h"
#include "R3DProject.h"

#include "openMVG/matching/indMatch.hpp"

class R3DComputeMatches
{
public:
	R3DComputeMatches();
	virtual ~R3DComputeMatches();

	enum R3DFeatureExtractor
	{
		R3DFESIFT = 0,
		R3DFESURF,
		R3DFEORB,
		R3DFEBRISK,
		R3DFEFAST,
		R3DFETEST,
		R3DFER3DF
	};

	void setMainFrame(Regard3DMainFrame *pMainFrame);
	void addImages(const ImageInfoVector &iiv);

	bool computeMatches(Regard3DFeatures::R3DFParams &params, bool svgOutput,
		const R3DProjectPaths &paths, int cameraModel);
	int computeMatchesOpenCV_NLOPT_step(double factor);
	static double myfunc(unsigned n, const double *x, double *grad, void *my_func_data);
	bool computeMatchesOpenCV_NLOPT();

	void updateProgress(float progress, const wxString &msg);


	struct R3DComputeMatchesStatistics
	{
		std::vector<int> numberOfKeypoints_;
		openMVG::matching::PairWiseMatches putativeMatches_;
		openMVG::matching::PairWiseMatches fundamentalMatches_;
		openMVG::matching::PairWiseMatches essentialMatches_;
		openMVG::matching::PairWiseMatches homographyMatches_;
	};
	const R3DComputeMatchesStatistics &getStatistics() { return statistics_; }

private:
	ImageInfoVector imageInfoVector_;
	Regard3DMainFrame *pMainFrame_;

	R3DComputeMatchesStatistics statistics_;
};

#endif
