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
#include "R3DSmallTasksThread.h"
#include "Regard3DMainFrame.h"
#include "Regard3DModelViewHelper.h"
#include "OpenMVGHelper.h"
#include "R3DModelOperations.h"

R3DSmallTasksThread::R3DSmallTasksThread() :
	wxThread(wxTHREAD_JOINABLE),
	pMainFrame_(NULL), type_(R3DSmallTasksThread::STTLoadModel),
	pRegard3DModelViewHelper_(NULL),
	pDensification_(NULL), pTriangulation_(NULL), pSurface_(NULL),
	numberOfClusters_(0)
{
}

R3DSmallTasksThread::~R3DSmallTasksThread()
{
}

void R3DSmallTasksThread::setMainFrame(Regard3DMainFrame *pMainFrame)
{
	pMainFrame_ = pMainFrame;
}

void R3DSmallTasksThread::stopThread()
{
	wxThread::ExitCode exitCode = this->Wait();
}

void R3DSmallTasksThread::loadModel(const wxString &filename, Regard3DModelViewHelper *pRegard3DModelViewHelper)
{
	type_ = R3DSmallTasksThread::STTLoadModel;
	filename_ = filename;
	pRegard3DModelViewHelper_ = pRegard3DModelViewHelper;

	this->Create();
	this->Run();
}

void R3DSmallTasksThread::loadSurfaceModel(const wxString &filename, Regard3DModelViewHelper *pRegard3DModelViewHelper)
{
	type_ = R3DSmallTasksThread::STTLoadSurfaceModel;
	filename_ = filename;
	pRegard3DModelViewHelper_ = pRegard3DModelViewHelper;

	this->Create();
	this->Run();
}

void R3DSmallTasksThread::exportToPMVS(R3DProject::Densification *pDensification)
{
	type_ = R3DSmallTasksThread::STTExportToPMVS;
	pDensification_ = pDensification;

	this->Create();
	this->Run();
}

void R3DSmallTasksThread::exportDensificationToMeshLab(R3DProject::Densification *pDensification, const wxString &pathname)
{
	type_ = R3DSmallTasksThread::STTExportToMeshLab;
	pDensification_ = pDensification;
	pathname_ = pathname;

	this->Create();
	this->Run();
}

void R3DSmallTasksThread::exportTriangulationToCMPMVS(R3DProject::Triangulation *pTriangulation, const wxString &pathname)
{
	type_ = R3DSmallTasksThread::STTExportToCMPMVS;
	pTriangulation_ = pTriangulation;
	pathname_ = pathname;

	this->Create();
	this->Run();
}

void R3DSmallTasksThread::exportDensificationToPointCloud(R3DProject::Densification *pDensification, const wxString &filename)
{
	type_ = R3DSmallTasksThread::STTExportToPointCloud;
	pDensification_ = pDensification;
	filename_ = filename;

	this->Create();
	this->Run();
}

void R3DSmallTasksThread::exportSurface(R3DProject::Surface *pSurface, const wxString &filename)
{
	type_ = R3DSmallTasksThread::STTExportSurface;
	pSurface_ = pSurface;
	filename_ = filename;

	this->Create();
	this->Run();
}

void R3DSmallTasksThread::colorizeSurface(R3DProject::Surface *pSurface)
{
	type_ = R3DSmallTasksThread::STTColorizeSurface;
	pSurface_ = pSurface;

	this->Create();
	this->Run();
}

void R3DSmallTasksThread::combineDenseModels(R3DProject::Densification *pDensification, int numberOfClusters)
{
	type_ = R3DSmallTasksThread::STTCombineDenseModels;
	pDensification_ = pDensification;
	numberOfClusters_ = numberOfClusters;

	this->Create();
	this->Run();
}

wxThread::ExitCode R3DSmallTasksThread::Entry()
{
	if(type_ == STTLoadModel)
	{
		model_ = pRegard3DModelViewHelper_->loadModel(filename_);
	}
	else if(type_ == STTLoadSurfaceModel)
	{
		model_ = pRegard3DModelViewHelper_->loadSurfaceModel(filename_);
	}
	else if(type_ == STTExportToPMVS)
	{
		OpenMVGHelper::exportToPMVS(pDensification_);
	}
	else if(type_ == STTExportToMeshLab)
	{
		OpenMVGHelper::exportToMeshLab(pDensification_, pathname_);
	}
	else if(type_ == STTExportToCMPMVS)
	{
		OpenMVGHelper::exportToCMPMVS(pTriangulation_, pathname_);
	}
	else if(type_ == STTExportToPointCloud)
	{
		R3DModelOperations::exportToPointCloud(pDensification_, filename_);
	}
	else if(type_ == STTExportSurface)
	{
		R3DModelOperations::exportSurface(pSurface_, filename_);
	}
	else if(type_ == STTColorizeSurface)
	{
		R3DModelOperations::colorizeSurface(pSurface_);
	}
	else if(type_ == STTCombineDenseModels)
	{
		R3DModelOperations::combineDenseModels(pDensification_, numberOfClusters_);
	}

	sendFinishedEvent();

	return 0;
}

void R3DSmallTasksThread::sendFinishedEvent()
{
	if(pMainFrame_ != NULL)
		pMainFrame_->sendSmallTaskFinishedEvent();
}
