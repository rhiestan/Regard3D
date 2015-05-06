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
#ifndef REGARD3DMODELVIEWHELPER_H
#define REGARD3DMODELVIEWHELPER_H

#include <osg/ref_ptr>
#include <osg/Node>

namespace osg
{
	class Drawable;
	class Node;
}
namespace osgViewer
{
	class Viewer;
}

#include <wx/thread.h>

class Regard3DModelViewHelper
{
public:
	Regard3DModelViewHelper();
	virtual ~Regard3DModelViewHelper();

	osg::ref_ptr<osg::Node> createEmptyModel();
	osg::ref_ptr<osg::Node> createRegard3DTextModel();
	osg::ref_ptr<osg::Node> loadModel(const wxString &filename);
	osg::ref_ptr<osg::Node> loadSurfaceModel(const wxString &filename);

	void setViewer(osgViewer::Viewer *pViewer);
	void buttonDown(int buttonIndex);
	void buttonUp(int buttonIndex);
	void showTrackball(bool showTrackball);
	void setPointSize(double pointSize);
	void showTexture(bool showTexture);
	void enableLighting(bool enableLighting);
	void polygonMode(int polygonMode);
	void shadingModel(int shadingModel);

	bool getIsButtonPressed();
	bool getShowTrackball();
	double getPointSize();
	bool getShowTexture();
	bool getEnableLighting();
	int getPolygonMode();
	int getShadingModel();

protected:
	osg::Node *createRotationSphere();
	static osg::Drawable *createRotationSphereCircle(int plane);

private:
	wxMutex mutex_;
	bool isButtonPressed_;
	bool showTrackball_;
	double pointSize_;
	bool showTexture_;
	bool enableLighting_;
	int polygonMode_;
	int shadingModel_;

	osgViewer::Viewer *pViewer_;
};

#endif
