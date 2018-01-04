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

#ifndef R3DPROJECT_H
#define R3DPROJECT_H

class wxTreeCtrl;

#include <boost/serialization/vector.hpp>
#include "ImageInfo.h"
#include "PreviewGeneratorThread.h"

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include <wx/treebase.h>

/**
 * Paths class for passing along to other classes and methods.
 *
 * This class contains all paths required to do work.
 */
struct R3DProjectPaths
{
	wxString absoluteProjectPath_;
	wxString absoluteImagePath_;
	wxString absoluteMatchesPath_;
	wxString absoluteOutPath_;
	wxString absoluteSurfacePath_;
	std::string relativeImagePath_;
	std::string relativeMatchesPath_;
	std::string relativeOutPath_;
	std::string relativeSfmOutPath_;
	std::string relativeMVESceneDir_;
	std::string listsTxtFilename_;
	std::string matchesSfmDataFilename_;
	std::string matchesPutitativeFilename_;
	std::string matchesFFilename_;
	std::string matchesEFilename_;
	std::string matchesHFilename_;
	std::string relativeTriSfmDataFilename_;
	std::string relativePMVSOutPath_;
	std::string relativeDenseModelName_;
	std::string relativeDensificationPath_;
	std::string relativeSurfacePath_;

	int pictureSetId_, computeMatchesId_, triangulationId_;
	int densificationId_, surfaceId_;
};


/**
 * Project class.
 */
class R3DProject
{
public:
	R3DProject();
	virtual ~R3DProject();

	bool newProject(const wxString &projectFilename);

	bool loadProject(const wxString &projectFilename);

	bool isValidProject() { return isValidProject_; }
	bool isSaved() { return isSaved_; }

	bool save();

	void closeProject();

	bool ensureImageFilesArePresent();

	const wxString &getProjectName() { return projectName_; }
	const wxString &getProjectPath() { return projectPath_; }
	

	enum R3DObjectState
	{
		OSInvalid = 0,
		OSRunning, OSFailed, OSFinished
	};

	/**
	 * Base class for all project entities.
	 */
	class Object
	{
	public:
		Object(): id_(0), parentId_(0), runningId_(0) { }
		virtual ~Object() { }

		virtual Object &copy(const Object &o)
		{
			id_ = o.id_;
			parentId_ = o.parentId_;
			runningId_ = o.runningId_;
			name_ = o.name_;
			orientation_ = o.orientation_;
			return *this;
		}
		virtual std::string getBasePathname() = 0;

		// id is unique within the project, while runningId is only unique among siblings (used for naming directories, for example)
		int id_, parentId_, runningId_;
		wxString name_, orientation_;
	};


	enum R3DSurfaceType
	{
		STPoissonRecon = 0,
		STFSSRecon
	};
	enum R3DColorizationType
	{
		CTColoredVertices = 0,
		CTTextures
	};

	/**
	 * Defines a surface/texture generation step.
	 */
	class Surface: public Object
	{
	public:
		Surface();
		Surface(const Surface &o);
		virtual ~Surface();
		Surface &copy(const Surface &o);
		Surface & operator=(const Surface &o);

		virtual std::string getBasePathname();

		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version);

		R3DSurfaceType surfaceType_;
		int poissonDepth_;
		float poissonSamplesPerNode_;
		float poissonPointWeight_;
		float poissonTrimThreshold_;
		int fssrRefineOctreeLevels_;
		float fssrScaleFactorMultiplier_;
		float fssrConfidenceThreshold_;
		int fssrMinComponentSize_;
		R3DColorizationType colorizationType_;
		int colVertNumNeighbours_;
		int textOutlierRemovalType_;
		bool textGeometricVisibilityTest_;
		bool textGlobalSeamLeveling_;
		bool textLocalSeamLeveling_;
		wxString finalSurfaceFilename_;
		wxString runningTime_;
		R3DObjectState state_;
	};

	enum R3DDensificationType
	{
		DTCMVSPMVS = 0,
		DTMVE,
		DTCMPMVS,
		DTSMVS
	};

	/**
	 * Defines a densification step.
	 */
	class Densification: public Object
	{
	public:
		Densification();
		Densification(const Densification &o);
		virtual ~Densification();
		Densification &copy(const Densification &o);
		Densification & operator=(const Densification &o);

		virtual std::string getBasePathname();

		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version);

		R3DDensificationType densificationType_;
		int pmvsNumThreads_;
		bool useCMVS_;
		int pmvsMaxClusterSize_;
		int pmvsLevel_, pmvsCSize_;
		float pmvsThreshold_;
		int pmvsWSize_, pmvsMinImageNum_;
		int mveScale_, mveFilterWidth_;
		int smvsInputScale_, smvsOutputScale_;
		bool smvsEnableShadingBasedOptimization_;
		bool smvsEnableSemiGlobalMatching_;
		float smvsAlpha_;
		wxString finalDenseModelName_;
		wxString runningTime_;

		R3DObjectState state_;

		std::vector<Surface> surfaces_;
	};

	enum R3DTriangulationVersion
	{
		R3DTV_0_7 = 0,		// openMVG 0.7
		R3DTV_0_8,			// openMVG 0.8.x
		R3DTV_Illegal
	};
	/**
	 * Defines a triangulation step.
	 */
	class Triangulation: public Object
	{
	public:
		Triangulation();
		Triangulation(const Triangulation &o);
		virtual ~Triangulation();
		Triangulation &copy(const Triangulation &o);
		Triangulation & operator=(const Triangulation &o);

		virtual std::string getBasePathname();

		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version);

		R3DTriangulationVersion version_;
		size_t initialImageIndexA_, initialImageIndexB_;
		bool global_, globalMSTBasedRot_;
		bool refineIntrinsics_;
		int rotAveraging_, transAveraging_;
		R3DObjectState state_;
		wxString resultCameras_, resultNumberOfTracks_;
		wxString resultResidualErrors_, runningTime_;

		std::vector<Densification> densifications_;
	};

	/**
	 * Defines a compute matches step.
	 */
	class ComputeMatches: public Object
	{
	public:
		ComputeMatches();
		ComputeMatches(const ComputeMatches &o);
		virtual ~ComputeMatches();
		ComputeMatches &copy(const ComputeMatches &o);
		ComputeMatches & operator=(const ComputeMatches &o);

		virtual std::string getBasePathname();

		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version);

		std::vector<Triangulation> triangulations_;
		wxString featureDetector_, descriptorExtractor_;
		float threshold_, distRatio_;
		int cameraModel_, matchingAlgorithm_;
		R3DObjectState state_;
		std::vector<int> numberOfKeypoints_;
		wxString runningTime_;
	};

	/**
	 * Defines a picture set.
	 */
	class PictureSet: public Object
	{
	public:
		PictureSet();
		PictureSet(const PictureSet &o);
		virtual ~PictureSet();
		PictureSet &copy(const PictureSet &o);
		PictureSet & operator=(const PictureSet &o);

		const ImageInfoVector &getImageInfoVector() const { return imageList_; }
		virtual std::string getBasePathname();

//	private:
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version);

		// Image list
		ImageInfoVector imageList_;
		int highestLocalImageNr_;

		std::vector<ComputeMatches> computeMatches_;
	};



	/**
	 * Data structure used for mapping items in wxTreeCtrl and R3DProject::Object.
	 */
	class R3DTreeItem: public wxTreeItemData
	{
	public:

		enum R3DTreeItemType
		{
			TypeInvalid = 0, TypeProject,
			TypePictureSet, TypeComputeMatches, TypeTriangulation,
			TypeDensification, TypeSurface
		};

		R3DTreeItem(): wxTreeItemData(), type_(TypeInvalid), id_(0) { }
		R3DTreeItem(R3DTreeItemType type, int id): wxTreeItemData(), type_(type), id_(id) { }
		virtual ~R3DTreeItem() { }

		R3DTreeItemType getType() const { return type_; }
		void setType(R3DTreeItemType type) { type_ = type; }
		int getID() const { return id_; }
		void setID(int id) { id_ = id; }

	private:
		R3DTreeItemType type_;
		int id_;
	};

	void populateTreeControl(wxTreeCtrl *pTreeCtrl);
	int addPictureSet(PictureSet &ps);
	void updatePictureSet(PictureSet &ps, PictureSet *oldPS);
	int clonePictureSet(PictureSet *pPictureSet);
	int addComputeMatches(R3DProject::PictureSet *pPictureSet,
		const wxString &featureDetector, const wxString &descriptorExtractor,
		float keypointSensitivity, float keypointMatchingRatio, int cameraModel, int matchingAlgorithm);
	int addTriangulation(R3DProject::ComputeMatches *pComputeMatches, size_t initialImageIndexA, size_t initialImageIndexB,
		bool global, int rotAveraging, int transAveraging, bool refineIntrinsics);
	int addDensification(R3DProject::Triangulation *pTriangulation);
	int addSurface(R3DProject::Densification *pDensification);
	void removePictureSet(PictureSet *pPictureSet);
	void removeComputeMatches(ComputeMatches *pComputeMatches);
	void removeTriangulation(Triangulation *pTriangulation);
	void removeDensification(Densification *pDensification);
	void removeSurface(Surface *pSurface);
	Object *getObjectByTypeAndID(R3DProject::R3DTreeItem::R3DTreeItemType type, int id);
	bool getProjectPathsCM(R3DProjectPaths &paths, ComputeMatches *pComputeMatches);
	bool getProjectPathsTri(R3DProjectPaths &paths, Triangulation *pTriangulation);
	bool getProjectPathsDns(R3DProjectPaths &paths, Densification *pDensification);
	bool getProjectPathsSrf(R3DProjectPaths &paths, Surface *pSurface);

	bool importAllImages(const R3DProjectPaths &paths);
	bool writeImageListTXT(const R3DProjectPaths &paths);
	bool writeSfmData(const R3DProjectPaths &paths, int cameraModel);
	void ensureSfmDataExists(const R3DProjectPaths &paths, int cameraModel);

	void prepareComputeMatches(const R3DProjectPaths &paths, float threshold, float distRatio);
	void prepareTriangulation(R3DProject::Triangulation *pTriangulation);
	void prepareDensification(R3DProject::Densification *pDensification);
	void prepareSurface(R3DProject::Surface *pSurface);

	wxString getImportedFNFromOrigFN(const ImageInfoVector &iiv, const wxString &origFilename);
	PreviewInfo prepareKeypointPreview(int computeMatchesId, const wxString &origFilename);
	PreviewInfo prepareMatchesPreview(int computeMatchesId, size_t index1, size_t index2);

	void setShownItemId(int id);
	int getShownItemId() const { return shownItemId_; }

	static void setInstance(R3DProject *pProject);
	static R3DProject *getInstance();

private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);

	// Attributes that define the state of the project in memory (non-persistent)
	bool isSaved_, isValidProject_;
	int shownItemId_;		// ID of the currently shown object (in 3D view)

	// The project name
	wxString projectName_;

	// Paths (projectPath is absolute, the rest is relative)
	wxString projectPath_;
	wxString imagePath_;
	wxString matchesPath_;
	wxString outPath_;
	wxString pmvsPath_;

	// Highest ID used up to now
	int highestID_;

	std::vector<PictureSet> pictureSets_;

	static R3DProject *pInstance_;
};

#endif
