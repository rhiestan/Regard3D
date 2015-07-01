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

#ifndef R3DSMALLTASKSTHREAD_H
#define R3DSMALLTASKSTHREAD_H

class Regard3DMainFrame;
class Regard3DModelViewHelper;

#include <osg/ref_ptr>
#include <osg/Node>

#include "R3DProject.h"

/**
 * This class contains several methods to be executed in a separate thread.
 *
 * Since the methods are small, all are contained in this class to
 * simplify handling. The methods are mostly unrelated, except that they
 * all are executed in a separate thread.
 * All methods are called from the main frame and return a message back
 * to the main frame as soon as they are finished.
 */
class R3DSmallTasksThread: public wxThread
{
public:
	enum SmallTaskType
	{
		STTLoadModel = 0,
		STTLoadSurfaceModel,
		STTExportToPMVS,
		STTExportToMeshLab,
		STTExportToExternalMVS,
		STTExportToPointCloud,
		STTExportSurface,
		STTColorizeSurface,
		STTCombineDenseModels,
		STTExportOldSfM_Output
	};

	R3DSmallTasksThread();
	virtual ~R3DSmallTasksThread();

	SmallTaskType getType() const { return type_; }
	osg::ref_ptr<osg::Node> getLoadedModel() const { return model_; }
	R3DProject::Densification *getDensification() const { return pDensification_; }
	R3DProject::Surface *getSurface() const { return pSurface_; }

	void setMainFrame(Regard3DMainFrame *pMainFrame);
	void stopThread();

	void loadModel(const wxString &filename, Regard3DModelViewHelper *pRegard3DModelViewHelper);
	void loadSurfaceModel(const wxString &filename, Regard3DModelViewHelper *pRegard3DModelViewHelper);
	void exportToPMVS(R3DProject::Densification *pDensification);
	void exportDensificationToMeshLab(R3DProject::Densification *pDensification, const wxString &pathname);
	void exportTriangulationToExternalMVS(R3DProject::Triangulation *pTriangulation, const wxString &pathname);
	void exportDensificationToPointCloud(R3DProject::Densification *pDensification, const wxString &filename);
	void exportSurface(R3DProject::Surface *pSurface, const wxString &filename);
	void colorizeSurface(R3DProject::Surface *pSurface);
	void combineDenseModels(R3DProject::Densification *pDensification, int numberOfClusters);
	void exportOldSfM_Output(R3DProject::Densification *pDensification);

protected:
	virtual wxThread::ExitCode Entry();

	void sendFinishedEvent();

private:
	Regard3DMainFrame *pMainFrame_;

	SmallTaskType type_;

	// For loadModel and loadSurfaceModel
	wxString filename_;
	Regard3DModelViewHelper *pRegard3DModelViewHelper_;
	osg::ref_ptr<osg::Node> model_;

	R3DProject::Densification *pDensification_;
	R3DProject::Triangulation *pTriangulation_;
	R3DProject::Surface *pSurface_;
	wxString pathname_;
	int numberOfClusters_;
};

#endif
