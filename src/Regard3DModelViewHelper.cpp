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
#include "Regard3DModelViewHelper.h"
#include "R3DFontHandler.h"
#include "version.h"

// OpenSceneGraph
#include <osgViewer/Viewer>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Point>
#include <osg/Drawable>
#include <osg/NodeCallback>
#include <osg/PolygonMode>
#include <osg/ShadeModel>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgUtil/Optimizer>
#include <osgText/Style>
#include <osgText/Text3D>

// PointCloudLibrary
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/TextureMesh.h>

// Asset Importer Library
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

/**
 * Updates the point size.
 */
class PointSizeUpdater: public osg::NodeCallback
{
public:
	PointSizeUpdater(Regard3DModelViewHelper *pMVHelper)
		: osg::NodeCallback(),
		pMVHelper_(pMVHelper)
	{
	}
	virtual ~PointSizeUpdater() { }

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		double pointSize = 1.0;
		osg::Geode *pGeode = node->asGeode();
		if(pMVHelper_ != NULL && pGeode != NULL)
		{
			pointSize = pMVHelper_->getPointSize();

			osg::StateSet *pStateSet = pGeode->getOrCreateStateSet();
			if(pStateSet != NULL)
			{
				osg::StateAttribute *pPointSA = pStateSet->getAttribute(osg::StateAttribute::POINT);
				osg::Point *pPoint = NULL;
				bool newCreated = false;
				if(pPointSA != NULL)
				{
					pPoint = dynamic_cast<osg::Point*>(pPointSA);
				}
				else
				{
					pPoint = new osg::Point();
					newCreated = true;
				}
				pPoint->setSize(pointSize);
				if(newCreated)
					pStateSet->setAttribute(pPoint);
			}
		}
		traverse(node, nv);
	}

private:
	Regard3DModelViewHelper *pMVHelper_;
};

/**
 * Switches the trackball sphere on and off.
 */
class SphereSwitchUpdater: public osg::NodeCallback
{
public:
	SphereSwitchUpdater(Regard3DModelViewHelper *pMVHelper)
		: osg::NodeCallback(),
		pMVHelper_(pMVHelper)
	{
	}
	virtual ~SphereSwitchUpdater() { }

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		bool showTrackball = true;
		osg::Switch *pSwitch = node->asSwitch();
		if(pMVHelper_ != NULL && pSwitch != NULL)
		{
			showTrackball = pMVHelper_->getShowTrackball();
			pSwitch->setValue(0, showTrackball);
		}
		traverse(node, nv);
	}

private:
	Regard3DModelViewHelper *pMVHelper_;
};

/**
 * Updates the opacity of the trackball sphere.
 */
class SphereOpacityUpdater: public osg::NodeCallback
{
public:
	SphereOpacityUpdater(Regard3DModelViewHelper *pMVHelper)
		: osg::NodeCallback(),
		pMVHelper_(pMVHelper)
	{
	}
	virtual ~SphereOpacityUpdater() { }

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		bool isButtonPressed = false;
		bool showTrackball = true;
		double pointSize = 1.0;
		osg::Geode *pGeode = node->asGeode();
		if(pMVHelper_ != NULL && pGeode != NULL)
		{
			isButtonPressed = pMVHelper_->getIsButtonPressed();
			pointSize = pMVHelper_->getPointSize();
			showTrackball = pMVHelper_->getShowTrackball();

			// Update opacity of rotation sphere
			double newOpacity = 0.4;
			if(isButtonPressed)
				newOpacity = 0.8;
			int numDr = pGeode->getNumDrawables();
			for(int j = 0; j < numDr; j++)
			{
				osg::Drawable *pDrawable = pGeode->getDrawable(j);
				osg::Geometry *pGeom = pDrawable->asGeometry();
				if(pGeom != NULL)
				{
					osg::Array *pColorArray = pGeom->getColorArray();
					osg::Vec4Array *pColors = dynamic_cast<osg::Vec4Array*>(pColorArray);
					int numElts = pColorArray->getNumElements();
					for(int k = 0; k < numElts; k++)
					{
						(*pColors)[k][3] = newOpacity;
					}
					pColorArray->dirty();
				}
				pGeom->dirtyDisplayList();
			}
		}
		traverse(node, nv);
	}

private:
	Regard3DModelViewHelper *pMVHelper_;
};

/**
 * Updates the position of the trackball sphere.
 *
 * This updater moves to trackball sphere in the virtual
 * space, such that it remains in the center of the trackball rotation
 * and the size is adjusted to the zoom.
 */
class SpherePosUpdater: public osg::NodeCallback
{
public:
	SpherePosUpdater(osgViewer::Viewer *pViewer)
		: osg::NodeCallback(),
		pViewer_(pViewer)
	{
	}
	virtual ~SpherePosUpdater() { }

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		osg::PositionAttitudeTransform *pTrackballPos = dynamic_cast<osg::PositionAttitudeTransform *>(node);
		if(pTrackballPos != NULL && pViewer_ != NULL)
		{
			//osg::Camera *pViewerCamera = pViewer_->getCamera();
			osg::Vec3d center;
			double scale = 1.0;
			osgGA::CameraManipulator *pManip = pViewer_->getCameraManipulator();
			osgGA::TrackballManipulator *pTrackballManip = dynamic_cast<osgGA::TrackballManipulator *>(pManip);
			if(pTrackballManip != NULL)
			{
				center = pTrackballManip->getCenter();
				scale = pTrackballManip->getDistance();
			}
			pTrackballPos->setPosition(center);

			osg::Vec3d scalevec(scale, scale, scale);
			pTrackballPos->setScale(scalevec);
		}

		traverse(node, nv);
	}

private:
	osgViewer::Viewer *pViewer_;
};

/**
 * Updates state set of the camera for texturing, lighting and polygon mode.
 *
 * Very similar to the class osgGA::StateSetManipulator.
 */
class StateSetUpdater: public osg::NodeCallback
{
public:
	StateSetUpdater(Regard3DModelViewHelper *pMVHelper)
		: osg::NodeCallback(),
		pMVHelper_(pMVHelper), maxNumOfTextureUnits_(4)
	{
	}
	virtual ~StateSetUpdater() { }

	virtual void operator()(osg::Node *pNode, osg::NodeVisitor *nv)
	{
		osg::StateSet *pStateSet = pNode->getOrCreateStateSet();

		if(pStateSet != NULL && pMVHelper_ != NULL)
		{
			bool showTexture = pMVHelper_->getShowTexture();
			bool enableLighting = pMVHelper_->getEnableLighting();
			int polygonMode = pMVHelper_->getPolygonMode();
			int shadingModel = pMVHelper_->getShadingModel();

			unsigned int mode = osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF;
			if(showTexture)
				mode = osg::StateAttribute::INHERIT|osg::StateAttribute::ON;

			for( unsigned int ii=0; ii < maxNumOfTextureUnits_; ii++ )
			{
#if !defined(OSG_GLES1_AVAILABLE) && !defined(OSG_GLES2_AVAILABLE)
				pStateSet->setTextureMode( ii, GL_TEXTURE_1D, mode );
#endif
				pStateSet->setTextureMode( ii, GL_TEXTURE_2D, mode );
				pStateSet->setTextureMode( ii, GL_TEXTURE_3D, mode );
				pStateSet->setTextureMode( ii, GL_TEXTURE_RECTANGLE, mode );
				pStateSet->setTextureMode( ii, GL_TEXTURE_CUBE_MAP, mode);
			}

			if(enableLighting)
				pStateSet->setMode(GL_LIGHTING, osg::StateAttribute::ON);
			else
				pStateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

			osg::PolygonMode* polyModeObj = getOrCreatePolygonMode(pStateSet);
			osg::PolygonMode::Mode pm = osg::PolygonMode::FILL;
			if(polygonMode == 1)
				pm = osg::PolygonMode::LINE;
			else if(polygonMode == 2)
				pm = osg::PolygonMode::POINT;
			polyModeObj->setMode(osg::PolygonMode::FRONT_AND_BACK, pm);

			osg::ShadeModel* pShadeModel = getOrCreateShadeModel(pStateSet);
			if(shadingModel == 0)
				pShadeModel->setMode(osg::ShadeModel::SMOOTH);
			else
				pShadeModel->setMode(osg::ShadeModel::FLAT);
			
		}

		traverse(pNode, nv);
	}

	// Copied from osgGA::StateSetManipulator::getOrCreatePolygonMode()
	static osg::PolygonMode* getOrCreatePolygonMode(osg::StateSet *pStateSet)
	{
		osg::PolygonMode* polyModeObj = dynamic_cast<osg::PolygonMode*>(pStateSet->getAttribute(osg::StateAttribute::POLYGONMODE));
		if (!polyModeObj)
		{
			polyModeObj = new osg::PolygonMode;
			pStateSet->setAttribute(polyModeObj);
		}
		return polyModeObj;
	}

	static osg::ShadeModel* getOrCreateShadeModel(osg::StateSet *pStateSet)
	{
		osg::ShadeModel* pShadeModel = dynamic_cast<osg::ShadeModel*>(pStateSet->getAttribute(osg::StateAttribute::SHADEMODEL));
		if (!pShadeModel)
		{
			pShadeModel = new osg::ShadeModel;
			pStateSet->setAttribute(pShadeModel);
		}
		return pShadeModel;
	}

private:
	Regard3DModelViewHelper *pMVHelper_;
	int maxNumOfTextureUnits_;
};


Regard3DModelViewHelper::Regard3DModelViewHelper()
	: mutex_(), isButtonPressed_(false),
	showTrackball_(true), pointSize_(1.0),
	showTexture_(true), enableLighting_(true),
	polygonMode_(0), shadingModel_(0),
	pViewer_(NULL)
{
}

Regard3DModelViewHelper::~Regard3DModelViewHelper()
{
}

osg::ref_ptr<osg::Node> Regard3DModelViewHelper::createEmptyModel()
{
	osg::ref_ptr<osg::Group> root = new osg::Group;

	osg::Node *pRotSphereNode = createRotationSphere();
	root->addChild(pRotSphereNode);

	return root;
}

osg::ref_ptr<osg::Node> Regard3DModelViewHelper::createRegard3DTextModel()
{
	osg::ref_ptr<osg::Group> root = new osg::Group;

	if(R3DFontHandler::getInstance().getFont().get() != NULL)
	{
		osg::ref_ptr<osgText::Style> style = new osgText::Style;
		style->setThicknessRatio(0.2f);
/*		osg::ref_ptr<osgText::Bevel> bevel = new osgText::Bevel;
		//bevel->roundedBevel2(0.25);
		//bevel->flatBevel(0.25f);
		//bevel->roundedBevel(0.25f);
		style->setBevel(bevel.get());*/

		osgText::Text3D* text3D = new osgText::Text3D;
		text3D->setFont(R3DFontHandler::getInstance().getFont().get());
		text3D->setStyle(style.get());
		text3D->setCharacterSize(1.0f);
		text3D->setDrawMode(osgText::Text3D::TEXT);
		text3D->setAxisAlignment(osgText::Text3D::XZ_PLANE);
		wxString text(wxT(REGARD3D_NAME));
//		text.Append(wxT(" "));
//		text.Append(wxString(REGARD3D_VERSION_STRING, *wxConvCurrent));
		text3D->setText( text.wc_str() );
		osg::Vec4 color(1.0f, 0.62f, 0.24f, 1.0f);
		text3D->setColor(color);

		osg::Geode* geode = new osg::Geode;
		geode->addDrawable(text3D);

		root->addChild(geode);
	}

	osg::Node *pRotSphereNode = createRotationSphere();
	root->addChild(pRotSphereNode);

	return root;
}


osg::ref_ptr<osg::Node> Regard3DModelViewHelper::loadModel(const wxString &filename)
{
	// See: http://roboticcreatures.wordpress.com/2011/12/29/loading-3d-point-clouds-pcd-format-in-openscenegraph/
	// https://github.com/adasta/osgpcl

	osg::ref_ptr<osg::Group> root = new osg::Group;
/*
	// OSG version doesn't load the colors from openMVG-generated PLY files
	osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile(std::string(filename.mb_str()));
	if (!loadedModel)
	{
		//std::cout << argv[0] <<": No data loaded." << std::endl;
		//return false;
	}
	else
	{

		// optimize the scene graph, remove redundant nodes and state etc.
//		osgUtil::Optimizer optimizer;
//		optimizer.optimize(loadedModel.get());

		loadedModel->setUpdateCallback(new PointSizeUpdater(this));

		root->addChild(loadedModel.get());
	}
*/
	pcl::PointCloud<pcl::PointXYZRGB> cloud;

	if (pcl::io::loadPLYFile<pcl::PointXYZRGB> (std::string(filename.mb_str()), cloud) == -1)
	{
		//std::cerr << "Couldn't read file " << pcd_file << std::endl;
	}
	else
	{
		osg::ref_ptr<osg::Geode> geode(new osg::Geode());
		osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());

		osg::ref_ptr<osg::Vec3Array> vertices(new osg::Vec3Array());
		osg::ref_ptr<osg::Vec4Array> colors(new osg::Vec4Array());

		for (size_t i = 0; i < cloud.points.size(); i++)
		{
			vertices->push_back( osg::Vec3(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z) );
			uint32_t rgb_val = *(reinterpret_cast<uint32_t *>( &(cloud.points[i].rgb) ) );	// "Cast" it to uint32

			int r = (rgb_val >> 16) & 0xff;
			int g = (rgb_val >> 8) & 0xff;
			int b = rgb_val & 0xff;
			colors->push_back(osg::Vec4f(static_cast<float>(r)/255.0f,
				static_cast<float>(g)/255.0f,
				static_cast<float>(b)/255.0f, 1.0f));
		}

		vertices->setDataVariance(osg::Object::STATIC);
		colors->setDataVariance(osg::Object::STATIC);
		geometry->setVertexArray(vertices.get());
		geometry->setColorArray(colors.get());
		geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
		geometry->setDataVariance(osg::Object::STATIC);

		geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));
		osg::StateSet* state = geometry->getOrCreateStateSet();
		state->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
		state->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );

		geode->addDrawable(geometry.get());
		geode->setUpdateCallback(new PointSizeUpdater(this));

		root->addChild(geode.get());
	}

	osg::Node *pRotSphereNode = createRotationSphere();
	root->addChild(pRotSphereNode);

	return root;
}


static osg::Vec3 calcNormal(osg::Vec3 &v1, osg::Vec3 &v2, osg::Vec3 &v3)
{
	osg::Vec3 u, v, normal;

	u = v2 - v1;
	v = v3 - v1;
	normal = u^v;
	normal.normalize();

	return normal;
}

/**
 * This method loads surface models from PLY and OBJ files.
 *
 * There are three versions:
 * 1) Uses assimp methods and creates osg models
 * 2) Uses pcl methods and creates osg models
 * 3) Directly load using osg methods
 *
 */
osg::ref_ptr<osg::Node> Regard3DModelViewHelper::loadSurfaceModel(const wxString &filename)
{
	osg::ref_ptr<osg::Group> root = new osg::Group;

	wxFileName fn(filename);

	bool isOK = false;
	if(fn.GetExt().IsSameAs(wxT("obj"), false))
		isOK = loadSurfaceModelAssImp(root, filename, false);
	else
		isOK = loadSurfaceModelOSG(root, filename, false);
//	isOK = loadSurfaceModelPCL(root, filename, false);

	osg::Node *pRotSphereNode = createRotationSphere();
	root->addChild(pRotSphereNode);

	return root;
}

void Regard3DModelViewHelper::setViewer(osgViewer::Viewer *pViewer)
{
	pViewer_ = pViewer;
}

void Regard3DModelViewHelper::buttonDown(int buttonIndex)
{
	wxMutexLocker lock(mutex_);
	if(buttonIndex == 0)
		isButtonPressed_ = true;
}

void Regard3DModelViewHelper::buttonUp(int buttonIndex)
{
	wxMutexLocker lock(mutex_);
	if(buttonIndex == 0)
		isButtonPressed_ = false;
}

void Regard3DModelViewHelper::showTrackball(bool showTrackball)
{
	wxMutexLocker lock(mutex_);
	showTrackball_ = showTrackball;
}

void Regard3DModelViewHelper::setPointSize(double pointSize)
{
	wxMutexLocker lock(mutex_);
	pointSize_ = pointSize;
}

void Regard3DModelViewHelper::showTexture(bool showTexture)
{
	wxMutexLocker lock(mutex_);
	showTexture_ = showTexture;
}

void Regard3DModelViewHelper::enableLighting(bool enableLighting)
{
	wxMutexLocker lock(mutex_);
	enableLighting_ = enableLighting;
}

void Regard3DModelViewHelper::polygonMode(int polygonMode)
{
	wxMutexLocker lock(mutex_);
	polygonMode_ = polygonMode;
}

void Regard3DModelViewHelper::shadingModel(int shadingModel)
{
	wxMutexLocker lock(mutex_);
	shadingModel_ = shadingModel;
}

bool Regard3DModelViewHelper::getIsButtonPressed()
{
	wxMutexLocker lock(mutex_);
	return isButtonPressed_;
}

bool Regard3DModelViewHelper::getShowTrackball()
{
	wxMutexLocker lock(mutex_);
	return showTrackball_;
}

double Regard3DModelViewHelper::getPointSize()
{
	wxMutexLocker lock(mutex_);
	return pointSize_;
}

bool Regard3DModelViewHelper::getShowTexture()
{
	wxMutexLocker lock(mutex_);
	return showTexture_;
}

bool Regard3DModelViewHelper::getEnableLighting()
{
	wxMutexLocker lock(mutex_);
	return enableLighting_;
}

int Regard3DModelViewHelper::getPolygonMode()
{
	wxMutexLocker lock(mutex_);
	return polygonMode_;
}

int Regard3DModelViewHelper::getShadingModel()
{
	wxMutexLocker lock(mutex_);
	return shadingModel_;
}

/**
 * This method creates the rotation sphere.
 *
 * It helps the user to see the rotation center of the trackball manipulator.
 * It is switched on and off using the SphereSwitchUpdater,
 * the opacity is changed using the SphereOpacityUpdater, and
 * the position and size is adjusted using the SpherePosUpdater.
 */
osg::Node *Regard3DModelViewHelper::createRotationSphere()
{
	osg::Geode *pGeodeSph = new osg::Geode;
	pGeodeSph->addDrawable( createRotationSphereCircle(0) );
	pGeodeSph->addDrawable( createRotationSphereCircle(1) );
	pGeodeSph->addDrawable( createRotationSphereCircle(2) );
	osg::StateSet* pStateSph = pGeodeSph->getOrCreateStateSet();
	pStateSph->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
	pStateSph->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
	pStateSph->setMode( GL_BLEND, osg::StateAttribute::ON );
	pStateSph->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	pStateSph->setAttribute(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
	pGeodeSph->setUpdateCallback(new SphereOpacityUpdater(this));
	osg::Switch *pSphereSwitch = new osg::Switch;
	pSphereSwitch->addChild(pGeodeSph, true);
	pSphereSwitch->setUpdateCallback(new SphereSwitchUpdater(this));

	osg::PositionAttitudeTransform *pTrackballPos = new osg::PositionAttitudeTransform();
	pTrackballPos->setDataVariance(osg::Object::DYNAMIC);
	pTrackballPos->addChild( pSphereSwitch );

	if(pViewer_ != NULL)
		pTrackballPos->setUpdateCallback(new SpherePosUpdater(pViewer_));

	return pTrackballPos;
}

osg::Drawable *Regard3DModelViewHelper::createRotationSphereCircle(int plane)
{
	const int division = 60;
	const double radius = 0.2;

	osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());

	osg::ref_ptr<osg::Vec3Array> vertices(new osg::Vec3Array());
	osg::ref_ptr<osg::Vec4Array> colors(new osg::Vec4Array());

	for(int i = 0; i < division; i++)
	{
		double angle = static_cast<double>(i) * 2.0 * M_PI / static_cast<double>(division);
		double sr = sin(angle) * radius;
		double cr = cos(angle) * radius;
		double x, y, z;
		if(plane == 0)
		{
			x = sr;
			y = cr;
			z = 0;
		}
		else if(plane == 1)
		{
			x = sr;
			y = 0;
			z = cr;
		}
		else
		{
			x = 0;
			y = sr;
			z = cr;
		}
		vertices->push_back( osg::Vec3( x, y, z ) );
	}

	vertices->setDataVariance(osg::Object::STATIC);
	colors->push_back( osg::Vec4( 1.0f, 1.0f, 1.0f, 0.4f ) );
	colors->setDataVariance(osg::Object::DYNAMIC);
	geometry->setVertexArray(vertices.get());
	geometry->setColorArray(colors.get());
	geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 0, vertices->size()));

	return geometry.release();
}

/**
 * Version using OpenSceneGraph.
 *
 *  Works fine for PLY and OBJ, fastest for PLY.
 */
bool Regard3DModelViewHelper::loadSurfaceModelOSG(osg::ref_ptr<osg::Group> root, const wxString &filename, bool debugOutput)
{
	osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile(std::string(filename.mb_str()));
	if (!loadedModel)
	{
		return false;
	}
	else
	{
		// For debugging purposes
		if(debugOutput)
			osgDB::writeNodeFile(*loadedModel, std::string("testout.osg"));

		// optimize the scene graph, remove redundant nodes and state etc.
		// Commented out as this takes a long time but doesn't improve anything here
//		osgUtil::Optimizer optimizer;
//		optimizer.optimize(loadedModel.get());

		loadedModel->setUpdateCallback(new StateSetUpdater(this));

		root->addChild(loadedModel.get());
	}

	return true;
}

/**
 * Version using the asset importer library (assimp).
 *
 * Works fine for PLY and OBJ, fastest for OBJ.
 */
bool Regard3DModelViewHelper::loadSurfaceModelAssImp(osg::ref_ptr<osg::Group> root, const wxString &filename, bool debugOutput)
{
	Assimp::Importer importer;
	const aiScene *pScene = importer.ReadFile( std::string(filename.mb_str()),
		aiProcess_JoinIdenticalVertices);
//		aiProcess_JoinIdenticalVertices | aiProcess_GenSmoothNormals);	// Calculating normals with assimp is quite slow, we do it ourselves
	if(pScene == NULL)
	{
		return false;
	}
	else
	{
		for(int i = 0; i < static_cast<int>(pScene->mNumMeshes); i++)
		{
			aiMesh *pMesh = pScene->mMeshes[i];
			if(pMesh != NULL)
			{
				osg::ref_ptr<osg::Geode> geode(new osg::Geode());
				osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());

				osg::ref_ptr<osg::Vec3Array> vertices(new osg::Vec3Array()), normals(new osg::Vec3Array());
				osg::ref_ptr<osg::Vec4Array> colors(new osg::Vec4Array());
				osg::ref_ptr<osg::Vec2Array> texcoords(new osg::Vec2Array());
				osg::ref_ptr<osg::DrawElementsUInt> drawElements(new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES));
				drawElements->setDataVariance(osg::Object::STATIC);

				int numVertices = static_cast<int>(pMesh->mNumVertices);
				bool hasColor = false;
				if(pMesh->GetNumColorChannels() > 0
					&& pMesh->HasVertexColors(0))
					hasColor = true;
				bool hasTexture = false;
				if(pMesh->GetNumUVChannels() > 0
					&& pMesh->HasTextureCoords(0))
					hasTexture = true;
				aiColor4D *pColors = NULL;
				if(hasColor)
					pColors = pMesh->mColors[0];

				vertices->reserve(numVertices);
				normals->reserve(numVertices);
				if(hasTexture)
					texcoords->reserve(numVertices);
				if(hasColor)
					colors->reserve(numVertices);

				for(int j = 0; j < numVertices; j++)
				{
					const aiVector3D &vec = pMesh->mVertices[j];
//					vertices->push_back( osg::Vec3(vec.x, -vec.z, vec.y) );
					vertices->push_back( osg::Vec3(vec.x, vec.y, vec.z) );
					normals->push_back( osg::Vec3(0, 0, 0) );

					if(hasColor)
					{
						const aiColor4D &color = pColors[j];
						colors->push_back(osg::Vec4f(color.r,
							color.g,
							color.b, 1.0f));
					}

					if(hasTexture)
					{
						const aiVector3D &textureCoord = pMesh->mTextureCoords[0][j];
						texcoords->push_back(osg::Vec2(textureCoord.x, textureCoord.y));
					}
				}

				int numFaces = static_cast<int>(pMesh->mNumFaces);
				drawElements->reserveElements(static_cast<unsigned int>(3*numFaces));
				for(int j = 0; j < numFaces; j++)
				{
					const aiFace &face = pMesh->mFaces[j];
					for(unsigned int k = 0; k < face.mNumIndices; k++)
					{
						unsigned int index = face.mIndices[k];
						drawElements->addElement(index);
					}
				}

				// Calculate normals
				for(int j = 0; j < numFaces; j++)
				{
					const aiFace &face = pMesh->mFaces[j];
					// We handle only triangles
					if(face.mNumIndices == 3)
					{
						osg::Vec3 normal = calcNormal( (*vertices)[face.mIndices[0]],
							(*vertices)[face.mIndices[1]],
							(*vertices)[face.mIndices[2]] );

						// Add calculated normal to all vertices of the current triangle
						for(size_t j = 0; j < 3; j++)
							(*normals)[face.mIndices[j]] += normal;
					}
					else
					{
						assert(false);
					}
				}

				// Normalize normals to length 1
				for (size_t i = 0; i < normals->size(); i++)
				{
					(*normals)[i].normalize();
				}

				vertices->setDataVariance(osg::Object::STATIC);
				normals->setDataVariance(osg::Object::STATIC);
				normals->setBinding(osg::Array::BIND_PER_VERTEX);
				geometry->setVertexArray(vertices.get());
				geometry->setNormalArray(normals.get());
				geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
				geometry->setDataVariance(osg::Object::STATIC);
				geometry->setUseVertexBufferObjects(true);

				if(hasColor)
				{
					colors->setDataVariance(osg::Object::STATIC);
					colors->setBinding(osg::Array::BIND_PER_VERTEX);
					geometry->setColorArray(colors.get());
					geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
				}

				if(hasTexture)
				{
					texcoords->setDataVariance(osg::Object::STATIC);
					geometry->setTexCoordArray(0, texcoords, osg::Array::BIND_PER_VERTEX);

					osg::ref_ptr<osg::Texture2D> texture(new osg::Texture2D());
					//texture->setDataVariance(osg::Object::DYNAMIC);
					unsigned int usedMaterial = pMesh->mMaterialIndex;
					if(usedMaterial < pScene->mNumMaterials)
					{
						const aiMaterial *pMaterial = pScene->mMaterials[usedMaterial];
						unsigned int textureCount = pMaterial->GetTextureCount(aiTextureType_DIFFUSE);
						for(unsigned int j = 0; j < textureCount; j++)
						{
							aiString path;
							aiTextureMapping mapping;
							unsigned int uvindex;
							float blend;
							aiTextureOp op;
							aiTextureMapMode mapmode[3];
							pMaterial->GetTexture(aiTextureType_DIFFUSE, j, &path, &mapping, &uvindex, &blend, &op, mapmode);

							wxFileName textureFN(filename);		// Use path from filename
							textureFN.SetFullName( wxString( path.C_Str(), wxConvUTF8 ) );
							std::string textureFN_str(textureFN.GetFullPath().mb_str());
							osg::ref_ptr<osg::Image> image(osgDB::readImageFile(textureFN_str.c_str()));
							if(image.get() == NULL)
								return false;
							texture->setImage(image.get());
							texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
							texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
							texture->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
						}
					}
					osg::StateSet *pStateSet = geometry->getOrCreateStateSet();
					pStateSet->setTextureAttributeAndModes(0, texture.get(), osg::StateAttribute::ON);
				}

				geometry->addPrimitiveSet(drawElements.get());

				geode->addDrawable(geometry.get());
				geode->setUpdateCallback(new StateSetUpdater(this));

				root->addChild(geode.get());
			}
		}


		// For debugging purposes
		if(debugOutput)
			osgDB::writeNodeFile(*root, std::string("testout.osg"));
	}

	return true;
}

/**
 * Version using PointCloudLibrary (PCL).
 *
 * Works fine for PLY (but slow), doesn't work for OBJ.
 */
bool Regard3DModelViewHelper::loadSurfaceModelPCL(osg::ref_ptr<osg::Group> root, const wxString &filename, bool debugOutput)
{
	bool loadWithTexture = false;
	wxFileName fn(filename);
	if(fn.GetExt().IsSameAs(wxT("obj"), false))
		loadWithTexture = true;

	if(loadWithTexture)
	{
		pcl::TextureMesh surfaceModel;
		if(pcl::io::load(std::string(filename.mb_str()), surfaceModel) == -1)
			return false;

		// Convert points from surface to pcl::PointCloud
		pcl::PointCloud<pcl::PointXYZRGB> surfacePoints;
		pcl::fromPCLPointCloud2(surfaceModel.cloud, surfacePoints);

		size_t numTriangles = 0;
		osg::ref_ptr<osg::Geode> geode(new osg::Geode());
		osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());

		osg::ref_ptr<osg::Vec3Array> vertices(new osg::Vec3Array()), normals(new osg::Vec3Array());
		osg::ref_ptr<osg::Vec2Array> texcoords(new osg::Vec2Array());

		vertices->reserve(surfacePoints.points.size());
		normals->reserve(surfacePoints.points.size());
		texcoords->reserve(surfacePoints.points.size());
		for (size_t i = 0; i < surfacePoints.points.size(); i++)
		{
			vertices->push_back( osg::Vec3(surfacePoints.points[i].x, surfacePoints.points[i].y, surfacePoints.points[i].z) );
			normals->push_back( osg::Vec3(0, 0, 0) );
		}

		// Calculate normals
		for(size_t i = 0; i < surfaceModel.tex_polygons.size(); i++)
		{
			const std::vector<pcl::Vertices> &vertexArray = surfaceModel.tex_polygons[i];
			const std::vector<Eigen::Vector2f> &texCoordArray = surfaceModel.tex_coordinates[i];
			const pcl::TexMaterial &texMaterial = surfaceModel.tex_materials[i];
			numTriangles += vertexArray.size();

			for(size_t j = 0; j < vertexArray.size(); j++)
			{
				const pcl::Vertices &surfaceVertices = vertexArray[j];

				// We handle only triangles
				if(surfaceVertices.vertices.size() == 3)
				{
					osg::Vec3 normal = calcNormal( (*vertices)[surfaceVertices.vertices[0]],
						(*vertices)[surfaceVertices.vertices[1]],
						(*vertices)[surfaceVertices.vertices[2]] );

					// Add calculated normal to all vertices of the current triangle
					for(size_t j = 0; j < 3; j++)
						(*normals)[surfaceVertices.vertices[j]] += normal;
				}
				else
				{
					assert(false);
				}
			}
		}

		// Normalize normals to length 1
		for (size_t i = 0; i < surfacePoints.points.size(); i++)
		{
			(*normals)[i].normalize();
		}

		// Copy texture coordinates  TODO: These come out wrong, why?
		for(size_t i = 0; i < surfaceModel.tex_polygons.size(); i++)
		{
			const std::vector<Eigen::Vector2f> &texCoordArray = surfaceModel.tex_coordinates[i];

			for(size_t j = 0; j < texCoordArray.size(); j++)
			{
				const Eigen::Vector2f &curTexCoord = texCoordArray[j];
				texcoords->push_back(osg::Vec2(curTexCoord.x(), curTexCoord.y()));
			}
		}

		vertices->setDataVariance(osg::Object::STATIC);
		normals->setDataVariance(osg::Object::STATIC);
		normals->setBinding(osg::Array::BIND_PER_VERTEX);
		texcoords->setDataVariance(osg::Object::STATIC);
		geometry->setVertexArray(vertices.get());
		geometry->setNormalArray(normals.get());
		geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
		geometry->setDataVariance(osg::Object::STATIC);
		geometry->setTexCoordArray(0, texcoords, osg::Array::BIND_PER_VERTEX);

		osg::ref_ptr<osg::Texture2D> texture(new osg::Texture2D());
		texture->setDataVariance(osg::Object::DYNAMIC);
		for(size_t i = 0; i < surfaceModel.tex_polygons.size(); i++)
		{
			const pcl::TexMaterial &texMaterial = surfaceModel.tex_materials[i];
			osg::ref_ptr<osg::Image> image(osgDB::readImageFile(texMaterial.tex_file));
			if(image.get() == NULL)
				return false;
			texture->setImage(image.get());
			texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
			texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
			texture->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
		}
		osg::StateSet *pStateSet = geometry->getOrCreateStateSet();
		pStateSet->setTextureAttributeAndModes(0, texture.get(), osg::StateAttribute::ON);

		osg::ref_ptr<osg::DrawElementsUInt> drawElements(new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES));
		drawElements->setDataVariance(osg::Object::STATIC);
		drawElements->reserveElements(static_cast<unsigned int>(3*numTriangles));

		// Copy indices
		for(size_t i = 0; i < surfaceModel.tex_polygons.size(); i++)
		{
			const std::vector<pcl::Vertices> &vertexArray = surfaceModel.tex_polygons[i];
			const std::vector<Eigen::Vector2f> &texCoordArray = surfaceModel.tex_coordinates[i];
			const pcl::TexMaterial &texMaterial = surfaceModel.tex_materials[i];
			numTriangles += vertexArray.size();

			for(size_t j = 0; j < vertexArray.size(); j++)
			{
				const pcl::Vertices &surfaceVertices = vertexArray[j];
				if(surfaceVertices.vertices.size() == 3)
				{
				}
				for(size_t j = 0; j < surfaceVertices.vertices.size(); j++)
				{
					drawElements->addElement(surfaceVertices.vertices[j]);
				}
			}
		}
		geometry->addPrimitiveSet(drawElements.get());
		geode->addDrawable(geometry.get());
		geode->setUpdateCallback(new StateSetUpdater(this));

		root->addChild(geode.get());

		// For debugging
		if(debugOutput)
			osgDB::writeNodeFile(*root, std::string("testout.osg"));
	}
	else
	{
		pcl::PolygonMesh surfaceModel;
		if(pcl::io::load(std::string(filename.mb_str()), surfaceModel) == -1)
			return false;

		// Convert points from surface to pcl::PointCloud
		pcl::PointCloud<pcl::PointXYZRGB> surfacePoints;
		pcl::fromPCLPointCloud2(surfaceModel.cloud, surfacePoints);


		osg::ref_ptr<osg::Geode> geode(new osg::Geode());
		osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());

		osg::ref_ptr<osg::Vec3Array> vertices(new osg::Vec3Array()), normals(new osg::Vec3Array());
		osg::ref_ptr<osg::Vec4Array> colors(new osg::Vec4Array());
		vertices->reserve(surfacePoints.points.size());
		normals->reserve(surfacePoints.points.size());
		colors->reserve(surfacePoints.points.size());
		for (size_t i = 0; i < surfacePoints.points.size(); i++)
		{
			vertices->push_back( osg::Vec3(surfacePoints.points[i].x, surfacePoints.points[i].y, surfacePoints.points[i].z) );
			normals->push_back( osg::Vec3(0, 0, 0) );
			uint32_t rgb_val = *(reinterpret_cast<uint32_t *>( &(surfacePoints.points[i].rgb) ) );	// "Cast" it to uint32

			int r = (rgb_val >> 16) & 0xff;
			int g = (rgb_val >> 8) & 0xff;
			int b = rgb_val & 0xff;
			colors->push_back(osg::Vec4f(static_cast<float>(r)/255.0f,
				static_cast<float>(g)/255.0f,
				static_cast<float>(b)/255.0f, 1.0f));
		}

		// Calculate normals
		for(size_t i = 0; i < surfaceModel.polygons.size(); i++)
		{
			const pcl::Vertices &surfaceVertices = surfaceModel.polygons[i];
			// We handle only triangles
			if(surfaceVertices.vertices.size() == 3)
			{
				osg::Vec3 normal = calcNormal( (*vertices)[surfaceVertices.vertices[0]],
					(*vertices)[surfaceVertices.vertices[1]],
					(*vertices)[surfaceVertices.vertices[2]] );

				// Add calculated normal to all vertices of the current triangle
				for(size_t j = 0; j < 3; j++)
					(*normals)[surfaceVertices.vertices[j]] += normal;
			}
			else
			{
				assert(false);
			}
		}

		// Normalize normals to length 1
		for (size_t i = 0; i < surfacePoints.points.size(); i++)
		{
			(*normals)[i].normalize();
		}

		vertices->setDataVariance(osg::Object::STATIC);
		normals->setDataVariance(osg::Object::STATIC);
		normals->setBinding(osg::Array::BIND_PER_VERTEX);
		colors->setDataVariance(osg::Object::STATIC);
		colors->setBinding(osg::Array::BIND_PER_VERTEX);
		geometry->setVertexArray(vertices.get());
		geometry->setNormalArray(normals.get());
		geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
		geometry->setColorArray(colors.get());
		geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
		geometry->setDataVariance(osg::Object::STATIC);

		osg::ref_ptr<osg::DrawElementsUInt> drawElements(new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES));
		drawElements->setDataVariance(osg::Object::STATIC);
		drawElements->reserveElements(static_cast<unsigned int>(3*surfaceModel.polygons.size()));

		// Copy indices
		for(size_t i = 0; i < surfaceModel.polygons.size(); i++)
		{
			const pcl::Vertices &surfaceVertices = surfaceModel.polygons[i];
			if(surfaceVertices.vertices.size() == 3)
			{
			}
			for(size_t j = 0; j < surfaceVertices.vertices.size(); j++)
			{
				drawElements->addElement(surfaceVertices.vertices[j]);
			}
		}

		geometry->addPrimitiveSet(drawElements.get());

		geode->addDrawable(geometry.get());
		geode->setUpdateCallback(new StateSetUpdater(this));

		root->addChild(geode.get());

		// For debugging
		if(debugOutput)
			osgDB::writeNodeFile(*root, std::string("testout.osg"));
	}

	return true;
}
