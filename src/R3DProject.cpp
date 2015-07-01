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
#include <boost/serialization/vector.hpp>
#include "R3DProject.h"

#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>

#include <wx/ffile.h>
#include <wx/dir.h>
#include <wx/treectrl.h>

// boost
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/version.hpp>

#if !wxCHECK_VERSION(2, 9, 0)
// wxWidgets before 2.9.0 does not support recursive delete
// Use boost::filesystem instead
#include <boost/filesystem.hpp>
#endif

R3DProject *R3DProject::pInstance_ = NULL;

R3DProject::R3DProject()
	: isSaved_(false), isValidProject_(false),
	highestID_(0), shownItemId_(-1)
{
}

R3DProject::~R3DProject()
{
}

bool R3DProject::newProject(const wxString &projectFilename)
{
	isSaved_ = false;
	wxFileName projectFN(projectFilename);
	projectPath_ = projectFN.GetPath(wxPATH_GET_VOLUME);
	projectName_ = projectFN.GetName();

	imagePath_ = wxT("in");
	matchesPath_ = wxT("matches");
	outPath_ = wxT("out");
	pmvsPath_ = wxT("PMVS");

	highestID_ = 0;
	shownItemId_ = -1;
	pictureSets_.clear();

	// Check whether projectPath exists
	wxFileName projectPath(projectFN);
	projectPath.SetFullName(wxEmptyString);
	if(!projectPath.DirExists())
		return false;

	isValidProject_ = true;

	save();

	// Change current directory of this process to project path
	wxFileName::SetCwd(projectPath.GetFullPath());

	return true;
}

bool R3DProject::loadProject(const wxString &projectFilename)
{
	isSaved_ = false;
	isValidProject_ = false;
	shownItemId_ = -1;
	highestID_ = 0;
	pictureSets_.clear();

	wxFFile fileIn(projectFilename, wxT("rb"));
	if(!fileIn.IsOpened())
		return false;

	// Determine file size
	fileIn.SeekEnd();
	wxFileOffset fileSize = fileIn.Tell();
	fileIn.Seek(0);

	std::string data;
	data.resize(fileSize);
	fileIn.Read(&(data[0]), fileSize);
	fileIn.Close();

	std::istringstream istr(data);
	boost::archive::xml_iarchive ia(istr);
	try
	{
		ia >> boost::serialization::make_nvp("R3DProject", *this);
	}
	catch(boost::archive::archive_exception &WXUNUSED(e))
	{
#if defined(_DEBUG) && defined(_MSC_VER)
		DebugBreak();
#endif
		return false;
	}

	wxFileName projFN(projectFilename);
	projectPath_ = projFN.GetPath(wxPATH_GET_VOLUME);

	// Change current directory of this process to project path
	wxFileName::SetCwd(projectPath_);

	isSaved_ = true;
	isValidProject_ = true;
	return true;
}

bool R3DProject::save()
{
	if(!isValidProject_)
		return false;

	std::ostringstream ostr;
	boost::archive::xml_oarchive oa(ostr);

	try
	{
		oa << boost::serialization::make_nvp("R3DProject", *this);
	}
	catch(boost::archive::archive_exception &WXUNUSED(e))
	{
#if defined(_DEBUG) && defined(_MSC_VER)
		DebugBreak();
#endif
		return false;
	}
	std::string data = ostr.str();

	wxFileName fnOut(projectPath_, projectName_, wxT("r3d"));
	wxFFile fileOut(fnOut.GetFullPath(), wxT("wb"));
	if(!fileOut.IsOpened())
		return false;

	fileOut.Write(data.c_str(), data.size());
	fileOut.Close();
	isSaved_ = true;

	return true;
}

void R3DProject::closeProject()
{
	isValidProject_ = isSaved_ = false;
	projectName_.Clear();
	pictureSets_.clear();
	highestID_ = 0;
	shownItemId_ = -1;
}

bool R3DProject::ensureImageFilesArePresent()
{
	for(size_t i = 0; i < pictureSets_.size(); i++)
	{
		const ImageInfoVector &iiv = pictureSets_[i].getImageInfoVector();
		ImageInfoVector::const_iterator iter = iiv.begin();
		while(iter != iiv.end())
		{
			const ImageInfo &ii = (*iter);

			if(!wxFileName::FileExists(ii.filename_))
			{
				isValidProject_ = false;
				return false;
			}

			iter++;
		}
	}
	return true;
}

void R3DProject::populateTreeControl(wxTreeCtrl *pTreeCtrl)
{
	// Keep selected item
	R3DProject::R3DTreeItem::R3DTreeItemType type = R3DProject::R3DTreeItem::TypeProject;
	int id = 0;
	wxTreeItemId selId = pTreeCtrl->GetSelection();
	if(selId.IsOk())
	{
		wxTreeItemData *pData = pTreeCtrl->GetItemData(selId);
		if(pData != NULL)
		{
			R3DProject::R3DTreeItem *pTreeItem = dynamic_cast<R3DProject::R3DTreeItem*>(pData);
			if(pTreeItem != NULL)
			{
				type = pTreeItem->getType();
				id = pTreeItem->getID();
			}
		}
	}

	pTreeCtrl->DeleteAllItems();
	if(isValidProject_)
	{
		wxTreeItemId selectId;
		wxTreeItemId rootId = pTreeCtrl->AddRoot(wxString(wxT("Project: ")) + projectName_, -1, -1, new R3DTreeItem(R3DTreeItem::TypeProject, 0));
		if(type == R3DProject::R3DTreeItem::TypeProject)
			selectId = rootId;

		for(size_t i = 0; i < pictureSets_.size(); i++)
		{
			PictureSet &ps = pictureSets_[i];
			wxString psname = ps.name_;
			if(psname.IsEmpty())
				psname = wxString::Format(wxT("%d"), ps.runningId_);
			wxTreeItemId subItemId1 = pTreeCtrl->AppendItem(rootId, wxString(wxT("Picture set ")) + psname,
				-1, -1, new R3DTreeItem(R3DTreeItem::TypePictureSet, ps.id_));
			if(type == R3DProject::R3DTreeItem::TypePictureSet && id == ps.id_)
				selectId = subItemId1;
			if(ps.id_ == shownItemId_)
				pTreeCtrl->SetItemBold(subItemId1, true);

			for(size_t j = 0; j < ps.computeMatches_.size(); j++)
			{
				ComputeMatches &cm = ps.computeMatches_[j];
				wxString cmStr = cm.name_;	//wxString::Format(wxT("thresh %3g/dist %3g"), cm.threshold_, cm.distRatio_);
				if(cmStr.IsEmpty())
					cmStr = wxString::Format(wxT("%d"), cm.runningId_);
				wxTreeItemId subItemId2 = pTreeCtrl->AppendItem(subItemId1, wxString(wxT("Matches ")) + cmStr,
					-1, -1, new R3DTreeItem(R3DTreeItem::TypeComputeMatches, cm.id_));

				if(type == R3DProject::R3DTreeItem::TypeComputeMatches && id == cm.id_)
					selectId = subItemId2;
				if(cm.id_ == shownItemId_)
					pTreeCtrl->SetItemBold(subItemId2, true);

				for(size_t k = 0; k < cm.triangulations_.size(); k++)
				{
					Triangulation &tri = cm.triangulations_[k];
					wxString triStr = tri.name_;
/*					if(tri.global_)
					{
						triStr = wxString(wxT("Global"));
					}
					else
					{
						triStr = wxString::Format(wxT("Incremental, initial pair %d/%d"), tri.initialImageIndexA_,
							tri.initialImageIndexB_);
					}*/
					if(triStr.IsEmpty())
						triStr = wxString::Format(wxT("%d"), tri.runningId_);
					wxTreeItemId subItemId3 = pTreeCtrl->AppendItem(subItemId2, wxString(wxT("Triangulation ")) + triStr,
						-1, -1, new R3DTreeItem(R3DTreeItem::TypeTriangulation, tri.id_));

					if(type == R3DProject::R3DTreeItem::TypeTriangulation && id == tri.id_)
						selectId = subItemId3;
					if(tri.id_ == shownItemId_)
						pTreeCtrl->SetItemBold(subItemId3, true);

					for(size_t l = 0; l < tri.densifications_.size(); l++)
					{
						Densification &dns = tri.densifications_[l];
						wxString dnsStr = dns.name_;
						if(dnsStr.IsEmpty())
							dnsStr = wxString::Format(wxT("%d"), dns.runningId_);

						wxTreeItemId subItemId4 = pTreeCtrl->AppendItem(subItemId3, wxString(wxT("Densification ")) + dnsStr,
							-1, -1, new R3DTreeItem(R3DTreeItem::TypeDensification, dns.id_));

						if(type == R3DProject::R3DTreeItem::TypeDensification && id == dns.id_)
							selectId = subItemId4;
						if(dns.id_ == shownItemId_)
							pTreeCtrl->SetItemBold(subItemId4, true);


						for(size_t m = 0; m < dns.surfaces_.size(); m++)
						{
							Surface &srf = dns.surfaces_[m];
							wxString srfStr = srf.name_;
							if(srfStr.IsEmpty())
								srfStr = wxString::Format(wxT("%d"), srf.runningId_);

							wxTreeItemId subItemId5 = pTreeCtrl->AppendItem(subItemId4, wxString(wxT("Surface ")) + srfStr,
								-1, -1, new R3DTreeItem(R3DTreeItem::TypeSurface, srf.id_));

							if(type == R3DProject::R3DTreeItem::TypeSurface && id == srf.id_)
								selectId = subItemId5;
							if(srf.id_ == shownItemId_)
								pTreeCtrl->SetItemBold(subItemId5, true);

						}

					}
				}
			}
		}

		pTreeCtrl->ExpandAll();

		// Select previously selected item
		if(selectId.IsOk())
			pTreeCtrl->SelectItem(selectId);
	}
}

int R3DProject::addPictureSet(R3DProject::PictureSet &ps)
{
	// Determine new running id
	int runningId = 0;
	for(size_t i = 0; i < pictureSets_.size(); i++)
		runningId = std::max( runningId, pictureSets_[i].runningId_ + 1 );

	highestID_++;
	ps.id_ = highestID_;
	ps.parentId_ = 0;
	ps.runningId_ = runningId;
	pictureSets_.push_back(ps);

	return highestID_;
}

void R3DProject::updatePictureSet(R3DProject::PictureSet &ps, R3DProject::PictureSet *oldPS)
{
	ImageInfoVector &iiv = oldPS->imageList_;
	for(size_t i = 0; i < iiv.size(); i++)
	{
		ImageInfo &ii = iiv[i];
		// If the image was imported, remove it
		if(ii.isImported_)
		{
			wxFileName imageFN(imagePath_, ii.importedFilename_);
			wxRemoveFile(imageFN.GetFullPath());
			ii.isImported_ = false;
		}
	}

	oldPS->name_ =  ps.name_;
	oldPS->imageList_ = ps.imageList_;
	ImageInfoVector &iiv2 = oldPS->imageList_;
	for(size_t i = 0; i < iiv2.size(); i++)
	{
		ImageInfo &ii = iiv2[i];
		ii.isImported_ = false;
		ii.importedFilename_.Clear();
	}
}

int R3DProject::clonePictureSet(R3DProject::PictureSet *pPictureSet)
{
	// Determine new running id
	int runningId = 0;
	for(size_t i = 0; i < pictureSets_.size(); i++)
		runningId = std::max( runningId, pictureSets_[i].runningId_ + 1 );

	PictureSet ps(*pPictureSet);	// Copy all attributes
	ps.computeMatches_.clear();		// Clear all compute matches objects (and derivatives)
	ImageInfoVector &iiv = ps.imageList_;
	for(size_t i = 0; i < iiv.size(); i++)
	{
		iiv[i].isImported_ = false;			// Clear isImported flag
		iiv[i].importedFilename_.Clear();
	}

	highestID_++;
	ps.id_ = highestID_;
	ps.parentId_ = 0;
	ps.runningId_ = runningId;
	pictureSets_.push_back(ps);

	return highestID_;
}

int R3DProject::addComputeMatches(R3DProject::PictureSet *pPictureSet,
	const wxString &featureDetector, const wxString &descriptorExtractor,
	float keypointSensitivity, float keypointMatchingRatio)
{
	if(pPictureSet == NULL)
		return -1;

	// Determine new running id
	int runningId = 0;
	for(size_t i = 0; i < pPictureSet->computeMatches_.size(); i++)
		runningId = std::max( runningId, pPictureSet->computeMatches_[i].runningId_ + 1 );

	highestID_++;
	ComputeMatches cm;
	cm.id_ = highestID_;
	cm.parentId_ = pPictureSet->id_;
	cm.runningId_ = runningId;
	cm.featureDetector_ = featureDetector;
	cm.descriptorExtractor_ = descriptorExtractor;
	cm.threshold_ = keypointSensitivity;
	cm.distRatio_ = keypointMatchingRatio;
	cm.state_ = OSInvalid;
	pPictureSet->computeMatches_.push_back(cm);

	return highestID_;
}

int R3DProject::addTriangulation(R3DProject::ComputeMatches *pComputeMatches, size_t initialImageIndexA, size_t initialImageIndexB,
	bool global, int rotAveraging, int transAveraging, bool refineIntrinsics)
{
	if(pComputeMatches == NULL)
		return -1;

	// Determine new running id
	int runningId = 0;
	for(size_t i = 0; i < pComputeMatches->triangulations_.size(); i++)
		runningId = std::max( runningId, pComputeMatches->triangulations_[i].runningId_ + 1 );

	highestID_++;
	Triangulation tri;
	tri.id_ = highestID_;
	tri.parentId_ = pComputeMatches->id_;
	tri.runningId_ = runningId;
	tri.version_ = R3DTV_0_8;
	tri.initialImageIndexA_ = initialImageIndexA;
	tri.initialImageIndexB_ = initialImageIndexB;
	tri.global_ = global;
	tri.globalMSTBasedRot_ = false;		// Was used for previous versions
	tri.rotAveraging_ = rotAveraging;
	tri.transAveraging_ = transAveraging;
	tri.refineIntrinsics_ = refineIntrinsics;
	tri.state_ = OSInvalid;
	pComputeMatches->triangulations_.push_back(tri);

	return highestID_;
}

int R3DProject::addDensification(R3DProject::Triangulation *pTriangulation)
{
	if(pTriangulation == NULL)
		return -1;

	// Determine new running id
	int runningId = 0;
	for(size_t i = 0; i < pTriangulation->densifications_.size(); i++)
		runningId = std::max( runningId, pTriangulation->densifications_[i].runningId_ + 1 );

	highestID_++;
	Densification dns;
	dns.id_ = highestID_;
	dns.parentId_ = pTriangulation->id_;
	dns.runningId_ = runningId;
	dns.state_ = OSInvalid;
	pTriangulation->densifications_.push_back(dns);

	return highestID_;
}

int R3DProject::addSurface(R3DProject::Densification *pDensification)
{
	if(pDensification == NULL)
		return -1;

	// Determine new running id
	int runningId = 0;
	for(size_t i = 0; i < pDensification->surfaces_.size(); i++)
		runningId = std::max( runningId, pDensification->surfaces_[i].runningId_ + 1 );

	highestID_++;
	Surface srf;
	srf.id_ = highestID_;
	srf.parentId_ = pDensification->id_;
	srf.runningId_ = runningId;
	srf.state_ = OSInvalid;
	pDensification->surfaces_.push_back(srf);

	return highestID_;
}

void R3DProject::removePictureSet(R3DProject::PictureSet *pPictureSet)
{
	int id = pPictureSet->id_;
	wxString pictureSetPathRunning = wxString( pPictureSet->getBasePathname().c_str(), wxConvLibc)
		+ wxString::Format(wxT("_%d"), pPictureSet->runningId_);

	std::vector<PictureSet>::iterator iter = pictureSets_.begin();
	while(iter != pictureSets_.end())
	{
		if(iter->id_ == id)
		{
			// Found
			pictureSets_.erase(iter);
			iter = pictureSets_.begin();
		}
		else
			iter++;
	}

	wxFileName psPath(projectPath_, wxT(""));
	psPath.AppendDir(pictureSetPathRunning);
#if wxCHECK_VERSION(2, 9, 0)
	psPath.Rmdir(wxPATH_RMDIR_RECURSIVE);
#else
	boost::system::error_code ec;
	boost::filesystem::remove_all(boost::filesystem::path(psPath.GetPath().c_str()), ec);
#endif
}

void R3DProject::removeComputeMatches(R3DProject::ComputeMatches *pComputeMatches)
{
	// Find parent PictureSet
	R3DProject::PictureSet *pPictureSet = NULL;
	R3DProject::Object *pObject = getObjectByTypeAndID(R3DProject::R3DTreeItem::TypePictureSet, pComputeMatches->parentId_);
	if(pObject != NULL)
	{
		pPictureSet = dynamic_cast<R3DProject::PictureSet *>(pObject);
	}
	if(pPictureSet == NULL)
		return;

	R3DProjectPaths paths;
	if(!getProjectPathsCM(paths, pComputeMatches))
		return;

	int id = pComputeMatches->id_;
	// From here on, don't use pComputeMatches anymore
	std::vector<ComputeMatches>::iterator iter = pPictureSet->computeMatches_.begin();
	while(iter != pPictureSet->computeMatches_.end())
	{
		if(iter->id_ == id)
		{
			// Found
			pPictureSet->computeMatches_.erase(iter);
			iter = pPictureSet->computeMatches_.begin();
		}
		else
			iter++;
	}

	wxFileName cmPath(paths.absoluteMatchesPath_, wxT(""));
	cmPath.RemoveLastDir();		// Get parent of "matches" path
#if wxCHECK_VERSION(2, 9, 0)
	cmPath.Rmdir(wxPATH_RMDIR_RECURSIVE);
#else
	boost::system::error_code ec;
	boost::filesystem::remove_all(boost::filesystem::path(cmPath.GetPath().c_str()), ec);
#endif
}

void R3DProject::removeTriangulation(R3DProject::Triangulation *pTriangulation)
{
	// Find parent ComputeMatches
	R3DProject::ComputeMatches *pComputeMatches = NULL;
	R3DProject::Object *pObject = getObjectByTypeAndID(R3DProject::R3DTreeItem::TypeComputeMatches, pTriangulation->parentId_);
	if(pObject != NULL)
	{
		pComputeMatches = dynamic_cast<R3DProject::ComputeMatches *>(pObject);
	}
	if(pComputeMatches == NULL)
		return;

	R3DProjectPaths paths;
	if(!getProjectPathsTri(paths, pTriangulation))
		return;

	int id = pTriangulation->id_;
	// From here on, don't use pTriangulation anymore
	std::vector<Triangulation>::iterator iter = pComputeMatches->triangulations_.begin();
	while(iter != pComputeMatches->triangulations_.end())
	{
		if(iter->id_ == id)
		{
			// Found
			pComputeMatches->triangulations_.erase(iter);
			iter = pComputeMatches->triangulations_.begin();
		}
		else
			iter++;
	}

	wxFileName triPath(paths.absoluteOutPath_, wxT(""));
	triPath.RemoveLastDir();	// Get parent of "out" path
#if wxCHECK_VERSION(2, 9, 0)
	triPath.Rmdir(wxPATH_RMDIR_RECURSIVE);
#else
	boost::system::error_code ec;
	boost::filesystem::remove_all(boost::filesystem::path(triPath.GetPath().c_str()), ec);
#endif
}

void R3DProject::removeDensification(R3DProject::Densification *pDensification)
{
	// Find parent Triangulation
	R3DProject::Triangulation *pTriangulation = NULL;
	R3DProject::Object *pObject = getObjectByTypeAndID(R3DProject::R3DTreeItem::TypeTriangulation, pDensification->parentId_);
	if(pObject != NULL)
	{
		pTriangulation = dynamic_cast<R3DProject::Triangulation *>(pObject);
	}
	if(pTriangulation == NULL)
		return;

	R3DProjectPaths paths;
	if(!getProjectPathsDns(paths, pDensification))
		return;

	int id = pDensification->id_;
	// From here on, don't use pDensification anymore
	std::vector<Densification>::iterator iter = pTriangulation->densifications_.begin();
	while(iter != pTriangulation->densifications_.end())
	{
		if(iter->id_ == id)
		{
			// Found
			pTriangulation->densifications_.erase(iter);
			iter = pTriangulation->densifications_.begin();
		}
		else
			iter++;
	}

	wxFileName dnsPath(wxString(paths.relativePMVSOutPath_.c_str(), wxConvLibc), wxEmptyString);
	dnsPath.RemoveLastDir();	// Get parent of "PMVS" path
#if wxCHECK_VERSION(2, 9, 0)
	dnsPath.Rmdir(wxPATH_RMDIR_RECURSIVE);
#else
	boost::system::error_code ec;
	boost::filesystem::remove_all(boost::filesystem::path(dnsPath.GetPath().c_str()), ec);
#endif
}

void R3DProject::removeSurface(R3DProject::Surface *pSurface)
{
	// Find parent Densification
	R3DProject::Densification *pDensification = NULL;
	R3DProject::Object *pObject = getObjectByTypeAndID(R3DProject::R3DTreeItem::TypeDensification, pSurface->parentId_);
	if(pObject != NULL)
	{
		pDensification = dynamic_cast<R3DProject::Densification *>(pObject);
	}
	if(pDensification == NULL)
		return;

	R3DProjectPaths paths;
	if(!getProjectPathsSrf(paths, pSurface))
		return;

	int id = pSurface->id_;
	// From here on, don't use pSurface anymore
	std::vector<Surface>::iterator iter = pDensification->surfaces_.begin();
	while(iter != pDensification->surfaces_.end())
	{
		if(iter->id_ == id)
		{
			// Found
			pDensification->surfaces_.erase(iter);
			iter = pDensification->surfaces_.begin();
		}
		else
			iter++;
	}

	wxFileName srfPath(paths.absoluteSurfacePath_, wxT(""));
#if wxCHECK_VERSION(2, 9, 0)
	srfPath.Rmdir(wxPATH_RMDIR_RECURSIVE);
#else
	boost::system::error_code ec;
	boost::filesystem::remove_all(boost::filesystem::path(srfPath.GetPath().c_str()), ec);
#endif
}

R3DProject::Object *R3DProject::getObjectByTypeAndID(R3DProject::R3DTreeItem::R3DTreeItemType type, int id)
{
	if(type == R3DProject::R3DTreeItem::TypeProject)
		return NULL;

	for(size_t i = 0; i < pictureSets_.size(); i++)
	{
		PictureSet &ps = pictureSets_[i];
		if(type == R3DProject::R3DTreeItem::TypePictureSet)
		{
			if(ps.id_ == id)
				return &ps;
		}
		else
		{
			// Continue searching for ComputeMatches
			for(size_t j = 0; j < ps.computeMatches_.size(); j++)
			{
				ComputeMatches &cm = ps.computeMatches_[j];
				if(type == R3DProject::R3DTreeItem::TypeComputeMatches)
				{
					if(cm.id_ == id)
						return &cm;
				}
				else
				{
					// Continue searching for Triangulations
					for(size_t k = 0; k < cm.triangulations_.size(); k++)
					{
						Triangulation &tri = cm.triangulations_[k];
						if(type == R3DProject::R3DTreeItem::TypeTriangulation)
						{
							if(tri.id_ == id)
								return &tri;
						}
						else
						{
							// Continue searching for Densifications
							for(size_t l = 0; l < tri.densifications_.size(); l++)
							{
								Densification &dns = tri.densifications_[l];
								if(type == R3DProject::R3DTreeItem::TypeDensification)
								{
									if(dns.id_ == id)
										return &dns;
								}
								else
								{
									// Continue searching for Surfaces
									for(size_t m = 0; m < dns.surfaces_.size(); m++)
									{
										Surface &srf = dns.surfaces_[m];
										if(type == R3DProject::R3DTreeItem::TypeSurface)
										{
											if(srf.id_ == id)
												return &srf;
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}

	return NULL;
}

bool R3DProject::getProjectPathsCM(R3DProjectPaths &paths, R3DProject::ComputeMatches *pComputeMatches)
{
	R3DProject::PictureSet *pPictureSet = NULL;
	R3DProject::Object *pObject = getObjectByTypeAndID(R3DProject::R3DTreeItem::TypePictureSet, pComputeMatches->parentId_);
	if(pObject != NULL)
	{
		pPictureSet = dynamic_cast<R3DProject::PictureSet *>(pObject);
	}
	if(pPictureSet == NULL)
		return false;

	paths.pictureSetId_ = pPictureSet->id_;
	paths.computeMatchesId_ = pComputeMatches->id_;
	paths.triangulationId_ = 0;			// Not needed/not known for compute matches step
	paths.densificationId_ = 0;
	paths.surfaceId_ = 0;
	paths.absoluteProjectPath_ = projectPath_;

	wxString pictureSetPathRunning = wxString( pPictureSet->getBasePathname().c_str(), wxConvLibc)
		+ wxString::Format(wxT("_%d"), pPictureSet->runningId_);
	wxString matchingPathRunning = wxString( pComputeMatches->getBasePathname().c_str(), wxConvLibc)
		+ wxString::Format(wxT("_%d"), pComputeMatches->runningId_);

	wxFileName imagePathFN(projectPath_, wxT(""));
	imagePathFN.AppendDir(pictureSetPathRunning);
	imagePathFN.AppendDir(imagePath_);
	paths.absoluteImagePath_ = imagePathFN.GetPath(wxPATH_GET_VOLUME);

	wxFileName matchesPathFN(projectPath_, wxT(""));
	matchesPathFN.AppendDir(pictureSetPathRunning);
	matchesPathFN.AppendDir(matchingPathRunning);
	matchesPathFN.AppendDir(matchesPath_);
	paths.absoluteMatchesPath_ = matchesPathFN.GetPath(wxPATH_GET_VOLUME);

	paths.absoluteOutPath_.Clear();		// Not needed/not known for compute matches step

	// Calculate relative paths
	if(imagePathFN.MakeRelativeTo(projectPath_))
		paths.relativeImagePath_ = std::string(imagePathFN.GetPath(wxPATH_GET_VOLUME).mb_str(wxConvLibc));
	else
		return false;

	if(matchesPathFN.MakeRelativeTo(projectPath_))
		paths.relativeMatchesPath_ = std::string(matchesPathFN.GetPath(wxPATH_GET_VOLUME).mb_str(wxConvLibc));
	else
		return false;

	paths.relativeOutPath_.clear();		// Not needed/not known for compute matches step
	paths.relativeSfmOutPath_.clear();	// Not needed/not known for compute matches step
	paths.relativeTriSfmDataFilename_.clear();

	wxFileName listsTxtFN(matchesPathFN);
	listsTxtFN.SetFullName(wxT("lists.txt"));
	paths.listsTxtFilename_ = std::string(listsTxtFN.GetFullPath().mb_str(wxConvLibc));

	wxFileName matchesSfmDataFN(matchesPathFN);
	matchesSfmDataFN.SetFullName(wxT("sfm_data.bin"));
	paths.matchesSfmDataFilename_ = std::string(matchesSfmDataFN.GetFullPath().mb_str(wxConvLibc));

	wxFileName matchesPutativeFN(matchesPathFN);
	matchesPutativeFN.SetFullName(wxT("matches.putative.txt"));
	paths.matchesPutitativeFilename_ = std::string(matchesPutativeFN.GetFullPath().mb_str(wxConvLibc));

	wxFileName matchesFFN(matchesPathFN);
	matchesFFN.SetFullName(wxT("matches.f.txt"));
	paths.matchesFFilename_ = std::string(matchesFFN.GetFullPath().mb_str(wxConvLibc));

	wxFileName matchesEFN(matchesPathFN);
	matchesEFN.SetFullName(wxT("matches.e.txt"));
	paths.matchesEFilename_ = std::string(matchesEFN.GetFullPath().mb_str(wxConvLibc));

	wxFileName matchesHFN(matchesPathFN);
	matchesHFN.SetFullName(wxT("matches.h.txt"));
	paths.matchesHFilename_ = std::string(matchesHFN.GetFullPath().mb_str(wxConvLibc));

	return true;
}

bool R3DProject::getProjectPathsTri(R3DProjectPaths &paths, R3DProject::Triangulation *pTriangulation)
{
	R3DProject::ComputeMatches *pComputeMatches = NULL;
	R3DProject::Object *pObject = getObjectByTypeAndID(R3DProject::R3DTreeItem::TypeComputeMatches, pTriangulation->parentId_);
	if(pObject != NULL)
	{
		pComputeMatches = dynamic_cast<R3DProject::ComputeMatches *>(pObject);
	}
	if(pComputeMatches == NULL)
		return false;

	// Set paths from compute matches
	if(!getProjectPathsCM(paths, pComputeMatches))
		return false;

	wxString triangulationPathRunning = wxString( pTriangulation->getBasePathname().c_str(), wxConvLibc)
		+ wxString::Format(wxT("_%d"), pTriangulation->runningId_);

	paths.triangulationId_ = pTriangulation->id_;

	wxFileName outPathFN(paths.absoluteMatchesPath_, wxT(""));
	outPathFN.RemoveLastDir();
	outPathFN.AppendDir(triangulationPathRunning);
	outPathFN.AppendDir(outPath_);

	paths.absoluteOutPath_ = outPathFN.GetPath(wxPATH_GET_VOLUME);

	if(outPathFN.MakeRelativeTo(projectPath_))
		paths.relativeOutPath_ = std::string(outPathFN.GetPath(wxPATH_GET_VOLUME).mb_str(wxConvLibc));
	else
		return false;

	wxFileName relativeTriSfmDataFN(outPathFN);
	relativeTriSfmDataFN.SetFullName(wxT("sfm_data.bin"));
	paths.relativeTriSfmDataFilename_ = std::string(relativeTriSfmDataFN.GetFullPath().mb_str(wxConvLibc));

	wxFileName outPathSfMFN(outPathFN);
	outPathSfMFN.AppendDir(wxT("SfM_output"));
	paths.relativeSfmOutPath_ = std::string(outPathSfMFN.GetPath(wxPATH_GET_VOLUME).mb_str(wxConvLibc));

	wxFileName outPathMVESceneDirFN(outPathFN);
	outPathMVESceneDirFN.AppendDir(wxT("MVE_SCENE_DIR"));
	paths.relativeMVESceneDir_  = std::string(outPathMVESceneDirFN.GetPath(wxPATH_GET_VOLUME).mb_str(wxConvLibc));

	return true;
}

bool R3DProject::getProjectPathsDns(R3DProjectPaths &paths, R3DProject::Densification *pDensification)
{
	R3DProject::Triangulation *pTriangulation = NULL;
	R3DProject::Object *pObject = getObjectByTypeAndID(R3DProject::R3DTreeItem::TypeTriangulation, pDensification->parentId_);
	if(pObject != NULL)
	{
		pTriangulation = dynamic_cast<R3DProject::Triangulation *>(pObject);
	}
	if(pTriangulation == NULL)
		return false;

	// Set paths from triangulation
	if(!getProjectPathsTri(paths, pTriangulation))
		return false;

	wxString densificationPathRunning = wxString( pDensification->getBasePathname().c_str(), wxConvLibc)
		+ wxString::Format(wxT("_%d"), pDensification->runningId_);

	paths.densificationId_ = pDensification->id_;

	wxFileName outPathFN(wxString(paths.relativeOutPath_.c_str(), wxConvLibc), wxEmptyString);
	outPathFN.RemoveLastDir();
	outPathFN.AppendDir(densificationPathRunning);

	paths.relativeDensificationPath_ = std::string(outPathFN.GetPath(wxPATH_GET_VOLUME).mb_str(wxConvLibc));

	outPathFN.AppendDir(pmvsPath_);

	paths.relativePMVSOutPath_ = std::string(outPathFN.GetPath(wxPATH_GET_VOLUME).mb_str(wxConvLibc));

	if(pDensification->densificationType_ == R3DProject::DTCMVSPMVS)
	{
		wxFileName denseModelFN(outPathFN);
		denseModelFN.AppendDir(wxT("models"));
		denseModelFN.SetFullName(pDensification->finalDenseModelName_);
		paths.relativeDenseModelName_ = std::string(denseModelFN.GetFullPath().mb_str(wxConvLibc));
	}
	else if(pDensification->densificationType_ == R3DProject::DTMVE)
	{
		wxFileName denseModelFN(wxString(paths.relativeDensificationPath_.c_str(), wxConvLibc), wxT(""));
		denseModelFN.SetFullName(pDensification->finalDenseModelName_);
		paths.relativeDenseModelName_ = std::string(denseModelFN.GetFullPath().mb_str(wxConvLibc));
	}

	return true;
}

bool R3DProject::getProjectPathsSrf(R3DProjectPaths &paths, R3DProject::Surface *pSurface)
{
	// Find parent Densification
	R3DProject::Densification *pDensification = NULL;
	R3DProject::Object *pObject = getObjectByTypeAndID(R3DProject::R3DTreeItem::TypeDensification, pSurface->parentId_);
	if(pObject != NULL)
	{
		pDensification = dynamic_cast<R3DProject::Densification *>(pObject);
	}
	if(pDensification == NULL)
		return false;

	// Set paths from densification
	if(!getProjectPathsDns(paths, pDensification))
		return false;

	wxString surfacePathRunning = wxString( pSurface->getBasePathname().c_str(), wxConvLibc)
		+ wxString::Format(wxT("_%d"), pSurface->runningId_);

	paths.surfaceId_ = pSurface->id_;

	wxFileName surfacePathFN(wxString(paths.relativePMVSOutPath_.c_str(), wxConvLibc), wxT(""));
	surfacePathFN.RemoveLastDir();
	surfacePathFN.AppendDir(surfacePathRunning);

	paths.relativeSurfacePath_ = std::string(surfacePathFN.GetPath(wxPATH_GET_VOLUME).mb_str(wxConvLibc));
	surfacePathFN.MakeAbsolute(projectPath_);
	paths.absoluteSurfacePath_ = surfacePathFN.GetPath(wxPATH_GET_VOLUME);

	return true;
}

bool R3DProject::importAllImages(const R3DProjectPaths &paths)
{
	R3DProject::PictureSet *pPictureSet = NULL;
	R3DProject::Object *pObject = getObjectByTypeAndID(R3DProject::R3DTreeItem::TypePictureSet, paths.pictureSetId_);
	if(pObject != NULL)
	{
		pPictureSet = dynamic_cast<R3DProject::PictureSet *>(pObject);
	}
	if(pPictureSet == NULL)
		return false;

	std::vector<ImageInfo>::iterator iter = pPictureSet->imageList_.begin();
	while(iter != pPictureSet->imageList_.end())
	{
		ImageInfo &ii = (*iter);

		bool doImport = false;
		if(!ii.isImported_)
		{
			doImport = true;
		}
		else
		{
			// Compare file sizes of external and imported file
			wxULongLong fileSizeExt = wxFileName::GetSize(ii.filename_);
			wxFileName importedFN(paths.absoluteImagePath_, ii.importedFilename_);			//imagePath_, ii.importedFilename_);
			wxULongLong fileSizeInt = importedFN.GetSize();
			if(fileSizeInt != fileSizeExt)
				doImport = true;
		}

		if(doImport)
		{
			// Create local filename
			wxString newLocalFilename(wxString::Format(wxT("image%06d.jpg"), pPictureSet->highestLocalImageNr_));
			pPictureSet->highestLocalImageNr_++;
			ii.importedFilename_ = newLocalFilename;
			wxFileName imageFN(paths.absoluteImagePath_, ii.importedFilename_);			//imagePath_, ii.importedFilename_);
			if(!wxCopyFile(ii.filename_, imageFN.GetFullPath()))
				return false;
			ii.isImported_ = true;
		}
		iter++;
	}

	return true;
}

bool R3DProject::writeImageListTXT(const R3DProjectPaths &paths)
{
	R3DProject::PictureSet *pPictureSet = NULL;
	R3DProject::Object *pObject = getObjectByTypeAndID(R3DProject::R3DTreeItem::TypePictureSet, paths.pictureSetId_);
	if(pObject != NULL)
	{
		pPictureSet = dynamic_cast<R3DProject::PictureSet *>(pObject);
	}
	if(pPictureSet == NULL)
		return false;

	// Create lists.txt
	std::ofstream listTXT(paths.listsTxtFilename_.c_str());		//getListsTxtFilename().c_str());
	if ( listTXT )
	{
		ImageInfoVector::iterator iter = pPictureSet->imageList_.begin();		//imageList_.begin();
		while(iter != pPictureSet->imageList_.end())		//imageList_.end())
		{
			ImageInfo &ii = (*iter);
			double focal = ii.focalLength_;
			int width = ii.imageWidth_;
			int height = ii.imageHeight_;
			std::string sCamName = std::string(ii.cameraMaker_.mb_str());
			std::string sCamModel = std::string(ii.cameraModel_.mb_str());
			std::string imgFilenameBase = std::string(ii.importedFilename_.mb_str());	// Use imported name (can safely be converted to C-string)

			if(ii.focalLength_ > 0 && ii.sensorWidth_ > 0)
			{
				// The camera model was found in the database so we can compute its approximated focal length
				double ccdw = ii.sensorWidth_;
				focal = std::max ( width, height ) * focal / ccdw;
				listTXT << imgFilenameBase << ";" << width << ";" << height << ";" << focal << ";" << sCamName << ";" << sCamModel << std::endl;
			}
			else
			{
				if(sCamName.empty() && sCamModel.empty())
					listTXT << imgFilenameBase << ";" << width << ";" << height << std::endl;
				else
					listTXT << imgFilenameBase << ";" << width << ";" << height << ";" << sCamName << ";" << sCamModel << std::endl;
			}
			iter++;
		}
	}
	else
	{
		return false;
	}
	listTXT.close();

	return true;
}

#if !defined(R3D_USE_OPENMVG_PRE08)
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#endif

bool R3DProject::writeSfmData(const R3DProjectPaths &paths)
{
#if !defined(R3D_USE_OPENMVG_PRE08)

	openMVG::cameras::EINTRINSIC e_User_camera_model(openMVG::cameras::PINHOLE_CAMERA_RADIAL3);		// TODO: Make configurable
	bool b_Group_camera_model = true;							// TODO: Make configurable

	openMVG::sfm::SfM_Data sfm_data;
	sfm_data.s_root_path = paths.relativeImagePath_;
	openMVG::sfm::Views &views = sfm_data.views;
	openMVG::sfm::Intrinsics &intrinsics = sfm_data.intrinsics;

	R3DProject::PictureSet *pPictureSet = NULL;
	R3DProject::Object *pObject = getObjectByTypeAndID(R3DProject::R3DTreeItem::TypePictureSet, paths.pictureSetId_);
	if(pObject != NULL)
	{
		pPictureSet = dynamic_cast<R3DProject::PictureSet *>(pObject);
	}
	if(pPictureSet == NULL)
		return false;

	ImageInfoVector::iterator iter = pPictureSet->imageList_.begin();
	while(iter != pPictureSet->imageList_.end())
	{
		ImageInfo &ii = (*iter);
		double focal = -1.0;
		int width = ii.imageWidth_;
		int height = ii.imageHeight_;
		std::string sCamName = std::string(ii.cameraMaker_.mb_str());
		std::string sCamModel = std::string(ii.cameraModel_.mb_str());
		std::string imgFilenameBase = std::string(ii.importedFilename_.mb_str());	// Use imported name (can safely be converted to C-string)
		double ppx = static_cast<double>(width) / 2.0;
		double ppy = static_cast<double> (height) / 2.0;

		if(ii.focalLength_ > 0 && ii.sensorWidth_ > 0)
		{
			// The camera model was found in the database so we can compute its approximated focal length
			double ccdw = ii.sensorWidth_;
			focal = std::max(width, height) * ii.focalLength_ / ccdw;
		}

		// Build intrinsic parameter related to the view
		std::shared_ptr<openMVG::cameras::IntrinsicBase> intrinsic(NULL);

		if(focal > 0 && ppx > 0 && ppy > 0 && width > 0 && height > 0)
		{
			// Create the desired camera type
			switch(e_User_camera_model)
			{
			case openMVG::cameras::PINHOLE_CAMERA:
				intrinsic = std::make_shared<openMVG::cameras::Pinhole_Intrinsic>
					(width, height, focal, ppx, ppy);
				break;
			case openMVG::cameras::PINHOLE_CAMERA_RADIAL1:
				intrinsic = std::make_shared<openMVG::cameras::Pinhole_Intrinsic_Radial_K1>
					(width, height, focal, ppx, ppy, 0.0); // setup no distortion as initial guess
				break;
			case openMVG::cameras::PINHOLE_CAMERA_RADIAL3:
				intrinsic = std::make_shared<openMVG::cameras::Pinhole_Intrinsic_Radial_K3>
					(width, height, focal, ppx, ppy, 0.0, 0.0, 0.0);  // setup no distortion as initial guess
				break;
			default:
				std::cout << "Unknown camera model: " << (int)e_User_camera_model << std::endl;
			}
		}

		// Build the view corresponding to the image
		openMVG::sfm::View v(imgFilenameBase, views.size(), views.size(), views.size(), width, height);

		// Add intrinsic related to the image (if any)
		if(intrinsic == NULL)
		{
			//Since the view have invalid intrinsic data
			// (export the view, with an invalid intrinsic field value)
			v.id_intrinsic = openMVG::UndefinedIndexT;
		}
		else
		{
			// Add the intrinsic to the sfm_container
			intrinsics[v.id_intrinsic] = intrinsic;
		}

		// Add the view to the sfm_container
		views[v.id_view] = std::make_shared<openMVG::sfm::View>(v);


		iter++;
	}

	// Group camera that share common properties if desired (leads to more faster & stable BA).
	if(b_Group_camera_model)
	{
		// Group camera model that share common optics camera properties
		// They must share (camera model, image size, & camera parameters)
		// Grouping is simplified by using a hash function over the camera intrinsic.

		// Build hash & build a set of the hash in order to maintain unique Ids
		std::set<size_t> hash_index;
		std::vector<size_t> hash_value;

		for(openMVG::sfm::Intrinsics::const_iterator iterIntrinsic = intrinsics.begin();
			iterIntrinsic != intrinsics.end();
			++iterIntrinsic)
		{
			const openMVG::cameras::IntrinsicBase * intrinsicData = iterIntrinsic->second.get();
			const size_t hashVal = intrinsicData->hashValue();
			hash_index.insert(hashVal);
			hash_value.push_back(hashVal);
		}

		// From hash_value(s) compute the new index (old to new indexing)
		openMVG::Hash_Map<openMVG::IndexT, openMVG::IndexT> old_new_reindex;
		size_t i = 0;
		for(openMVG::sfm::Intrinsics::const_iterator iterIntrinsic = intrinsics.begin();
			iterIntrinsic != intrinsics.end();
			++iterIntrinsic, ++i)
		{
			old_new_reindex[iterIntrinsic->first] = std::distance(hash_index.begin(), hash_index.find(hash_value[i]));
		}
		//--> Copy & modify Ids & replace
		//     - for the Intrinsic params
		//     - for the View
		openMVG::sfm::Intrinsics intrinsic_updated;
		for(openMVG::sfm::Intrinsics::const_iterator iterIntrinsic = intrinsics.begin();
			iterIntrinsic != intrinsics.end();
			++iterIntrinsic)
		{
			intrinsic_updated[old_new_reindex[iterIntrinsic->first]] = intrinsics[iterIntrinsic->first];
		}
		intrinsics.swap(intrinsic_updated); // swap camera intrinsics
		// Update intrinsic ids
		for(openMVG::sfm::Views::iterator iterView = views.begin();
			iterView != views.end();
			++iterView)
		{
			openMVG::sfm::View * v = iterView->second.get();
			v->id_intrinsic = old_new_reindex[v->id_intrinsic];
		}
	}

	// Store SfM_Data views & intrinsic data
	if(Save(
		sfm_data,
		paths.matchesSfmDataFilename_.c_str(),
		openMVG::sfm::ESfM_Data(openMVG::sfm::VIEWS | openMVG::sfm::INTRINSICS))
		)
		return true;
	else
		return false;
#endif
	return true;
}

void R3DProject::ensureSfmDataExists(const R3DProjectPaths &paths)
{
	wxFileName sfm_DataFN(wxString(paths.matchesSfmDataFilename_.c_str(), wxConvLibc));
	if(!sfm_DataFN.FileExists())
		writeSfmData(paths);
}

static inline bool isEqualEps(float x, float y, float eps)
{
	return std::abs(x - y) <= eps * std::abs(x);
}

void R3DProject::prepareComputeMatches(const R3DProjectPaths &paths, float threshold, float distRatio)
{
	// Make sure directories exist
	if(!wxFileName::DirExists(paths.absoluteImagePath_))
#if wxCHECK_VERSION(2, 9, 0)
		wxFileName::Mkdir(paths.absoluteImagePath_, wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL);
#else
		wxFileName::Mkdir(paths.absoluteImagePath_, 0777, wxPATH_MKDIR_FULL);
#endif
	if(!wxFileName::DirExists(paths.absoluteMatchesPath_))
#if wxCHECK_VERSION(2, 9, 0)
		wxFileName::Mkdir(paths.absoluteMatchesPath_, wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL);
#else
		wxFileName::Mkdir(paths.absoluteMatchesPath_, 0777, wxPATH_MKDIR_FULL);
#endif

/*	const float eps = 1.0e-5f;
	if(lastComputingMatchesRunFinished_
		&& isEqualEps(threshold, cmThresholdLR_, eps)
		&& isEqualEps(distRatio, cmDistRatioLR_, eps))
	{
		// Running with same parameters -> don't delete old features and descriptors
		lastComputingMatchesRunFinished_ = false;
		return;
	}*/

	// Delete all files in the matches directory
	wxArrayString fileList;
	wxDir matchesDir(paths.absoluteMatchesPath_);		//getAbsoluteMatchesPath());
	if(matchesDir.IsOpened())
	{
		wxString filename;
		bool cont = matchesDir.GetFirst(&filename, wxEmptyString, wxDIR_FILES | wxDIR_HIDDEN);
		while(cont)
		{
			fileList.Add(filename);
			cont = matchesDir.GetNext(&filename);
		}

		wxFileName fn(paths.absoluteMatchesPath_);		//getAbsoluteMatchesPath(), wxT(""));
		for(size_t i = 0; i < fileList.Count(); i++)
		{
			filename = fileList[i];
			fn.SetFullName(filename);
			wxRemoveFile(fn.GetFullPath());
		}
	}

	save();
}

void R3DProject::prepareTriangulation(R3DProject::Triangulation *pTriangulation)
{
	// Delete all files and directories in the out directory
	R3DProjectPaths paths;
	if(!getProjectPathsTri(paths, pTriangulation))
		return;

	// Make sure directories exist
	if(!wxFileName::DirExists(paths.absoluteOutPath_))
#if wxCHECK_VERSION(2, 9, 0)
		wxFileName::Mkdir(paths.absoluteOutPath_, wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL);
#else
		wxFileName::Mkdir(paths.absoluteOutPath_, 0777, wxPATH_MKDIR_FULL);
#endif
//	if(!wxFileName::DirExists(paths.relativeSfmOutPath_))
//		wxFileName::Mkdir(paths.relativeSfmOutPath_, wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL);

	wxArrayString fileList, dirList;
	wxDir outDir(paths.absoluteOutPath_);
	if(outDir.IsOpened())
	{
		wxString filename;
		bool cont = outDir.GetFirst(&filename,  wxEmptyString, wxDIR_FILES | wxDIR_HIDDEN);
		while(cont)
		{
			fileList.Add(filename);
			cont = outDir.GetNext(&filename);
		}

		wxFileName fn(paths.absoluteOutPath_, wxT(""));
		for(size_t i = 0; i < fileList.Count(); i++)
		{
			filename = fileList[i];
			fn.SetFullName(filename);
			wxRemoveFile(fn.GetFullPath());
		}

		wxString dirname;
		cont = outDir.GetFirst(&dirname,  wxEmptyString, wxDIR_DIRS);
		while(cont)
		{
			dirList.Add(dirname);
			cont = outDir.GetNext(&dirname);
		}

		for(size_t i = 0; i < dirList.Count(); i++)
		{
			wxFileName fnDir(paths.absoluteOutPath_, wxT(""));
			dirname = dirList[i];
			fnDir.AppendDir(dirname);
#if wxCHECK_VERSION(2, 9, 0)
			fnDir.Rmdir(wxPATH_RMDIR_RECURSIVE);	// Recursively delete directory
#else
			boost::system::error_code ec;
			boost::filesystem::remove_all(boost::filesystem::path(fnDir.GetPath().c_str()), ec);
#endif



		}
	}

	save();
}

void R3DProject::prepareDensification(R3DProject::Densification *pDensification)
{
	R3DProjectPaths paths;
	if(!getProjectPathsDns(paths, pDensification))
		return;

	// Make sure directories exist
	if(pDensification->densificationType_ == R3DProject::DTCMVSPMVS)
	{
		wxString pmvsOutPath(paths.relativePMVSOutPath_.c_str(), wxConvLibc);
		if(!wxFileName::DirExists(pmvsOutPath))
#if wxCHECK_VERSION(2, 9, 0)
			wxFileName::Mkdir(pmvsOutPath, wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL);
#else
			wxFileName::Mkdir(pmvsOutPath, 0777, wxPATH_MKDIR_FULL);
#endif
	}
	else if(pDensification->densificationType_ == R3DProject::DTMVE)
	{
		wxString mveOutPath(paths.relativeDensificationPath_.c_str(), wxConvLibc);
		if(!wxFileName::DirExists(mveOutPath))
#if wxCHECK_VERSION(2, 9, 0)
			wxFileName::Mkdir(mveOutPath, wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL);
#else
			wxFileName::Mkdir(mveOutPath, 0777, wxPATH_MKDIR_FULL);
#endif
	}
}

void R3DProject::prepareSurface(R3DProject::Surface *pSurface)
{
	R3DProjectPaths paths;
	if(!getProjectPathsSrf(paths, pSurface))
		return;

	// Make sure directories exist
	wxString surfacePath(paths.absoluteSurfacePath_);
	if(!wxFileName::DirExists(surfacePath))
#if wxCHECK_VERSION(2, 9, 0)
		wxFileName::Mkdir(surfacePath, wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL);
#else
		wxFileName::Mkdir(surfacePath, 0777, wxPATH_MKDIR_FULL);
#endif
}

wxString R3DProject::getImportedFNFromOrigFN(const ImageInfoVector &iiv, const wxString &origFilename)
{
	ImageInfoVector::const_iterator iter = iiv.begin();
	while(iter != iiv.end())
	{
		const ImageInfo &ii = *iter;
		if(ii.filename_.IsSameAs(origFilename))
		{
			if(ii.isImported_ && !ii.importedFilename_.IsEmpty())
			{
				return ii.importedFilename_;
			}
		}
		iter++;
	}
	return wxEmptyString;
}

PreviewInfo R3DProject::prepareKeypointPreview(int computeMatchesId, const wxString &origFilename)
{
	PreviewInfo previewInfo;

	Object *pObject = getObjectByTypeAndID(R3DProject::R3DTreeItem::TypeComputeMatches, computeMatchesId);
	ComputeMatches *pComputeMatches = NULL;
	if(pObject != NULL)
		pComputeMatches = dynamic_cast<ComputeMatches *>(pObject);
	if(pComputeMatches == NULL)
		return previewInfo;
	pObject = getObjectByTypeAndID(R3DProject::R3DTreeItem::TypePictureSet, pComputeMatches->parentId_);
	PictureSet *pPictureSet = NULL;
	if(pObject != NULL)
		pPictureSet = dynamic_cast<PictureSet *>(pObject);
	if(pPictureSet == NULL)
		return previewInfo;

	R3DProjectPaths paths;
	if(!getProjectPathsCM(paths, pComputeMatches))
		return previewInfo;
	const ImageInfoVector &iiv = pPictureSet->imageList_;

	previewInfo.type_ = PreviewInfo::PITKeypoints;
	previewInfo.filename1_ = origFilename;

	wxString importedFilename = getImportedFNFromOrigFN(iiv, origFilename);
	if(!importedFilename.IsEmpty())
	{
		wxFileName fn(importedFilename);
		fn.SetExt(wxT("feat"));
		fn.SetPath(wxString(paths.relativeMatchesPath_.c_str(), wxConvLibc));
		if(fn.FileExists())
		{
			previewInfo.kpFilename1_ = fn.GetFullPath();
		}
	}

	return previewInfo;
}

PreviewInfo R3DProject::prepareMatchesPreview(int computeMatchesId, size_t index1, size_t index2)
{
	PreviewInfo previewInfo;
	Object *pObject = getObjectByTypeAndID(R3DProject::R3DTreeItem::TypeComputeMatches, computeMatchesId);
	ComputeMatches *pComputeMatches = NULL;
	if(pObject != NULL)
		pComputeMatches = dynamic_cast<ComputeMatches *>(pObject);
	if(pComputeMatches == NULL)
		return previewInfo;
	pObject = getObjectByTypeAndID(R3DProject::R3DTreeItem::TypePictureSet, pComputeMatches->parentId_);
	PictureSet *pPictureSet = NULL;
	if(pObject != NULL)
		pPictureSet = dynamic_cast<PictureSet *>(pObject);
	if(pPictureSet == NULL)
		return previewInfo;

	R3DProjectPaths paths;
	if(!getProjectPathsCM(paths, pComputeMatches))
		return previewInfo;
	const ImageInfoVector &iiv = pPictureSet->imageList_;

	previewInfo.type_ = PreviewInfo::PITMatches;
	previewInfo.filename1_ = iiv[index1].filename_;
	previewInfo.filename2_ = iiv[index2].filename_;
	wxString impfilename1 = iiv[index1].importedFilename_;
	wxString impfilename2 = iiv[index2].importedFilename_;
	if(!impfilename1.IsEmpty())
	{
		wxFileName fn(impfilename1);
		fn.SetExt(wxT("feat"));
		fn.SetPath(wxString(paths.relativeMatchesPath_.c_str(), wxConvLibc));
		if(fn.FileExists())
		{
			previewInfo.kpFilename1_ = fn.GetFullPath();
		}
	}
	if(!impfilename2.IsEmpty())
	{
		wxFileName fn(impfilename2);
		fn.SetExt(wxT("feat"));
		fn.SetPath(wxString(paths.relativeMatchesPath_.c_str(), wxConvLibc));
		if(fn.FileExists())
		{
			previewInfo.kpFilename2_ = fn.GetFullPath();
		}
	}
	previewInfo.matchesFilename_ = wxString(paths.matchesFFilename_.c_str(), wxConvLibc);
	previewInfo.index1_ = index1;
	previewInfo.index2_ = index2;

	return previewInfo;
}

void R3DProject::setShownItemId(int id)
{
	shownItemId_ = id;
}

void R3DProject::setInstance(R3DProject *pProject)
{
	pInstance_ = pProject;
}

R3DProject *R3DProject::getInstance()
{
	return pInstance_;
}


/**
 * R3DProject::PictureSet.
 */
R3DProject::PictureSet::PictureSet()
	: Object(), highestLocalImageNr_(0)
{
}

R3DProject::PictureSet::PictureSet(const R3DProject::PictureSet &o)
{
	copy(o);
}

R3DProject::PictureSet::~PictureSet()
{
}

R3DProject::PictureSet &R3DProject::PictureSet::copy(const R3DProject::PictureSet &o)
{
	R3DProject::Object::copy(o);	// Call base class
	imageList_ = o.imageList_;
	highestLocalImageNr_ = o.highestLocalImageNr_;
	computeMatches_ = o.computeMatches_;
	return *this;
}

R3DProject::PictureSet & R3DProject::PictureSet::operator=(const R3DProject::PictureSet &o)
{
	return copy(o);
}

std::string R3DProject::PictureSet::getBasePathname()
{
	return std::string("pictureset");
}

R3DProject::ComputeMatches::ComputeMatches()
	: Object(), threshold_(0), distRatio_(0),
	state_(R3DProject::OSInvalid)
{
}

R3DProject::ComputeMatches::ComputeMatches(const R3DProject::ComputeMatches &o)
{
	copy(o);
}

R3DProject::ComputeMatches::~ComputeMatches()
{
}

R3DProject::ComputeMatches &R3DProject::ComputeMatches::copy(const R3DProject::ComputeMatches &o)
{
	R3DProject::Object::copy(o);	// Call base class
	featureDetector_ = o.featureDetector_;
	descriptorExtractor_ = o.descriptorExtractor_;
	threshold_ = o.threshold_;
	distRatio_ = o.distRatio_;
	state_ = o.state_;
	numberOfKeypoints_ = o.numberOfKeypoints_;
	runningTime_ = o.runningTime_;
	triangulations_ = o.triangulations_;
	return *this;
}

R3DProject::ComputeMatches & R3DProject::ComputeMatches::operator=(const R3DProject::ComputeMatches &o)
{
	return copy(o);
}

std::string R3DProject::ComputeMatches::getBasePathname()
{
	return std::string("matching");
}

R3DProject::Triangulation::Triangulation()
	: Object(), version_(R3DProject::R3DTV_0_7), initialImageIndexA_(0), initialImageIndexB_(0),
	global_(false), globalMSTBasedRot_(false),
	refineIntrinsics_(true), rotAveraging_(2), transAveraging_(1),
	state_(R3DProject::OSInvalid)
{
}

R3DProject::Triangulation::Triangulation(const R3DProject::Triangulation &o)
{
	copy(o);
}

R3DProject::Triangulation::~Triangulation()
{
}

R3DProject::Triangulation &R3DProject::Triangulation::copy(const R3DProject::Triangulation &o)
{
	R3DProject::Object::copy(o);	// Call base class
	version_ = o.version_;
	initialImageIndexA_ = o.initialImageIndexA_;
	initialImageIndexB_ = o.initialImageIndexB_;
	global_ = o.global_;
	state_ = o.state_;
	globalMSTBasedRot_ = o.globalMSTBasedRot_;
	refineIntrinsics_ = o.refineIntrinsics_;
	rotAveraging_ = o.rotAveraging_;
	transAveraging_ = o.transAveraging_;
	resultCameras_ = o.resultCameras_;
	resultNumberOfTracks_ = o.resultNumberOfTracks_;
	resultResidualErrors_ = o.resultResidualErrors_;
	runningTime_ = o.runningTime_;
	densifications_ = o.densifications_;
	return *this;
}

R3DProject::Triangulation & R3DProject::Triangulation::operator=(const R3DProject::Triangulation &o)
{
	return copy(o);
}

std::string R3DProject::Triangulation::getBasePathname()
{
	return std::string("triangulation");
}


R3DProject::Densification::Densification()
	: Object(), densificationType_(R3DProject::DTCMVSPMVS),
	pmvsNumThreads_(1), useCMVS_(true), pmvsMaxClusterSize_(100),
	pmvsLevel_(1), pmvsCSize_(2), pmvsThreshold_(0.7f), pmvsWSize_(7),
	pmvsMinImageNum_(3), mveScale_(0), mveFilterWidth_(5), state_(R3DProject::OSInvalid)
{
}

R3DProject::Densification::Densification(const R3DProject::Densification &o)
{
	copy(o);
}

R3DProject::Densification::~Densification()
{
}

R3DProject::Densification &R3DProject::Densification::copy(const R3DProject::Densification &o)
{
	R3DProject::Object::copy(o);	// Call base class
	densificationType_ = o.densificationType_;
	pmvsNumThreads_ = o.pmvsNumThreads_;
	useCMVS_ = o.useCMVS_;
	pmvsMaxClusterSize_ = o.pmvsMaxClusterSize_;
	pmvsLevel_ = o.pmvsLevel_;
	pmvsCSize_ = o.pmvsCSize_;
	pmvsThreshold_ = o.pmvsThreshold_;
	pmvsWSize_ = o.pmvsWSize_;
	pmvsMinImageNum_ = o.pmvsMinImageNum_;
	mveScale_ = o.mveScale_;
	mveFilterWidth_ = o.mveFilterWidth_;
	finalDenseModelName_ = o.finalDenseModelName_;
	runningTime_ = o.runningTime_;
	state_ = o.state_;
	surfaces_ = o.surfaces_;
	return *this;
}

R3DProject::Densification & R3DProject::Densification::operator=(const R3DProject::Densification &o)
{
	return copy(o);
}

std::string R3DProject::Densification::getBasePathname()
{
	return std::string("densification");
}

R3DProject::Surface::Surface()
	: Object(), surfaceType_(R3DProject::STPoissonRecon),
	poissonDepth_(9), poissonSamplesPerNode_(1.0f), poissonPointWeight_(4.0f), poissonTrimThreshold_(0.0f),
	fssrRefineOctreeLevels_(0), fssrScaleFactorMultiplier_(1.0f), fssrConfidenceThreshold_(1.0f),
	fssrMinComponentSize_(1000),
	colorizationType_(R3DProject::CTTextures), colVertNumNeighbours_(3),
	textOutlierRemovalType_(0), textGeometricVisibilityTest_(true),
	textGlobalSeamLeveling_(true), textLocalSeamLeveling_(true),
	state_(R3DProject::OSInvalid)
{
}

R3DProject::Surface::Surface(const R3DProject::Surface &o)
{
	copy(o);
}

R3DProject::Surface::~Surface()
{
}

R3DProject::Surface &R3DProject::Surface::copy(const R3DProject::Surface &o)
{
	R3DProject::Object::copy(o);	// Call base class
	surfaceType_ = o.surfaceType_;
	poissonDepth_ = o.poissonDepth_;
	poissonSamplesPerNode_ = o.poissonSamplesPerNode_;
	poissonPointWeight_ = o.poissonPointWeight_;
	poissonTrimThreshold_ = o.poissonTrimThreshold_;
	fssrRefineOctreeLevels_ = o.fssrRefineOctreeLevels_;
	fssrScaleFactorMultiplier_ = o.fssrScaleFactorMultiplier_;
	fssrConfidenceThreshold_ = o.fssrConfidenceThreshold_;
	fssrMinComponentSize_ = o.fssrMinComponentSize_;
	colorizationType_ = o.colorizationType_;
	colVertNumNeighbours_ = o.colVertNumNeighbours_;
	textOutlierRemovalType_ = o.textOutlierRemovalType_;
	textGeometricVisibilityTest_ = o.textGeometricVisibilityTest_;
	textGlobalSeamLeveling_ = o.textGlobalSeamLeveling_;
	textLocalSeamLeveling_ = o.textLocalSeamLeveling_;
	finalSurfaceFilename_ = o.finalSurfaceFilename_;
	runningTime_ = o.runningTime_;
	state_ = o.state_;
	return *this;
}

R3DProject::Surface & R3DProject::Surface::operator=(const R3DProject::Surface &o)
{
	return copy(o);
}

std::string R3DProject::Surface::getBasePathname()
{
	return std::string("surface");
}

// Allow wxString to be serialized
namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void save(Archive & ar, const wxString &s, unsigned int version)
		{
			std::string ss(s.mb_str(wxConvUTF8));
			ar & make_nvp("str", ss);
		}

		template<class Archive>
		void load(Archive & ar, wxString &s, unsigned int version)
		{
			std::string ss;
			ar & make_nvp("str", ss);
			s = wxString(ss.c_str(), wxConvUTF8);
		}
	}
}
BOOST_SERIALIZATION_SPLIT_FREE(wxString)

// Serialize ImageInfo class
namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void serialize(Archive & ar, ImageInfo &i, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("filename", i.filename_);
			ar & boost::serialization::make_nvp("importedFilename", i.importedFilename_);
			ar & boost::serialization::make_nvp("isImported", i.isImported_);
			ar & boost::serialization::make_nvp("imageWidth", i.imageWidth_);
			ar & boost::serialization::make_nvp("imageHeight", i.imageHeight_);
			ar & boost::serialization::make_nvp("cameraMaker", i.cameraMaker_);
			ar & boost::serialization::make_nvp("cameraModel", i.cameraModel_);
			ar & boost::serialization::make_nvp("focalLength", i.focalLength_);
			ar & boost::serialization::make_nvp("sensorWidth", i.sensorWidth_);
		}
	}
}

// Serialize Surface class
template<class Archive>
void R3DProject::Surface::serialize(Archive & ar, const unsigned int version)
{
	ar & boost::serialization::make_nvp("id", id_);
	ar & boost::serialization::make_nvp("parentId", parentId_);
	ar & boost::serialization::make_nvp("runningId", runningId_);
	ar & boost::serialization::make_nvp("name", name_);
	ar & boost::serialization::make_nvp("surfaceType", surfaceType_);
	ar & boost::serialization::make_nvp("poissonDepth", poissonDepth_);
	ar & boost::serialization::make_nvp("poissonSamplesPerNode", poissonSamplesPerNode_);
	ar & boost::serialization::make_nvp("poissonPointWeight", poissonPointWeight_);
	ar & boost::serialization::make_nvp("poissonTrimThreshold", poissonTrimThreshold_);
	ar & boost::serialization::make_nvp("fssrRefineOctreeLevels", fssrRefineOctreeLevels_);
	ar & boost::serialization::make_nvp("fssrScaleFactorMultiplier", fssrScaleFactorMultiplier_);
	ar & boost::serialization::make_nvp("fssrConfidenceThreshold", fssrConfidenceThreshold_);
	ar & boost::serialization::make_nvp("fssrMinComponentSize", fssrMinComponentSize_);
	ar & boost::serialization::make_nvp("colorizationType", colorizationType_);
	ar & boost::serialization::make_nvp("colVertNumNeighbours", colVertNumNeighbours_);
	ar & boost::serialization::make_nvp("textOutlierRemovalType", textOutlierRemovalType_);
	ar & boost::serialization::make_nvp("textGeometricVisibilityTest", textGeometricVisibilityTest_);
	ar & boost::serialization::make_nvp("textGlobalSeamLeveling", textGlobalSeamLeveling_);
	ar & boost::serialization::make_nvp("textLocalSeamLeveling", textLocalSeamLeveling_);
	ar & boost::serialization::make_nvp("finalSurfaceFilename", finalSurfaceFilename_);
	ar & boost::serialization::make_nvp("runningTime", runningTime_);
	ar & boost::serialization::make_nvp("state", state_);
	ar & boost::serialization::make_nvp("orientation", orientation_);
}
//BOOST_CLASS_VERSION(R3DProject::Surface, 1)

// Serialize Densification class
template<class Archive>
void R3DProject::Densification::serialize(Archive & ar, const unsigned int version)
{
	ar & boost::serialization::make_nvp("id", id_);
	ar & boost::serialization::make_nvp("parentId", parentId_);
	ar & boost::serialization::make_nvp("runningId", runningId_);
	ar & boost::serialization::make_nvp("name", name_);
	ar & boost::serialization::make_nvp("densificationType", densificationType_);
	ar & boost::serialization::make_nvp("pmvsNumThreads", pmvsNumThreads_);
	ar & boost::serialization::make_nvp("useCMVS", useCMVS_);
	ar & boost::serialization::make_nvp("pmvsMaxClusterSize", pmvsMaxClusterSize_);
	ar & boost::serialization::make_nvp("pmvsLevel", pmvsLevel_);
	ar & boost::serialization::make_nvp("pmvsCSize", pmvsCSize_);
	ar & boost::serialization::make_nvp("pmvsThreshold", pmvsThreshold_);
	ar & boost::serialization::make_nvp("pmvsWSize", pmvsWSize_);
	ar & boost::serialization::make_nvp("pmvsMinImageNum", pmvsMinImageNum_);
	ar & boost::serialization::make_nvp("mveScale", mveScale_);
	ar & boost::serialization::make_nvp("mveFilterWidth", mveFilterWidth_);
	ar & boost::serialization::make_nvp("finalDenseModelName", finalDenseModelName_);
	ar & boost::serialization::make_nvp("runningTime", runningTime_);
	ar & boost::serialization::make_nvp("state", state_);
	ar & boost::serialization::make_nvp("orientation", orientation_);
	ar & boost::serialization::make_nvp("Surfaces", surfaces_);
}
//BOOST_CLASS_VERSION(R3DProject::Densification, 1)

// Serialize Triangulation class
template<class Archive>
void R3DProject::Triangulation::serialize(Archive & ar, const unsigned int version)
{
	ar & boost::serialization::make_nvp("id", id_);
	ar & boost::serialization::make_nvp("parentId", parentId_);
	ar & boost::serialization::make_nvp("runningId", runningId_);
	ar & boost::serialization::make_nvp("name", name_);
	if(version > 0)
		ar & boost::serialization::make_nvp("version", version_);
	else
	{
		if(Archive::is_loading::value)
			version_ = R3DProject::R3DTV_0_7;
	}
	ar & boost::serialization::make_nvp("initialImageIndexA", initialImageIndexA_);
	ar & boost::serialization::make_nvp("initialImageIndexB", initialImageIndexB_);
	ar & boost::serialization::make_nvp("global", global_);
	ar & boost::serialization::make_nvp("state", state_);
	ar & boost::serialization::make_nvp("globalMSTBasedRot", globalMSTBasedRot_);
	if(version > 0)
	{
		ar & boost::serialization::make_nvp("refineIntrinsics", refineIntrinsics_);
		ar & boost::serialization::make_nvp("rotAveraging", rotAveraging_);
		ar & boost::serialization::make_nvp("transAveraging", transAveraging_);
	}
	ar & boost::serialization::make_nvp("resultCameras", resultCameras_);
	ar & boost::serialization::make_nvp("resultNumberOfTracks", resultNumberOfTracks_);
	ar & boost::serialization::make_nvp("resultResidualErrors", resultResidualErrors_);
	ar & boost::serialization::make_nvp("runningTime", runningTime_);
	ar & boost::serialization::make_nvp("orientation", orientation_);
	ar & boost::serialization::make_nvp("Densifications", densifications_);
}
BOOST_CLASS_VERSION(R3DProject::Triangulation, 1)

// Serialize ComputeMatches class
template<class Archive>
void R3DProject::ComputeMatches::serialize(Archive & ar, const unsigned int version)
{
	ar & boost::serialization::make_nvp("id", id_);
	ar & boost::serialization::make_nvp("parentId", parentId_);
	ar & boost::serialization::make_nvp("runningId", runningId_);
	ar & boost::serialization::make_nvp("name", name_);
	ar & boost::serialization::make_nvp("featureDetector", featureDetector_);
	ar & boost::serialization::make_nvp("descriptorExtractor", descriptorExtractor_);
	ar & boost::serialization::make_nvp("threshold", threshold_);
	ar & boost::serialization::make_nvp("distRatio", distRatio_);
	ar & boost::serialization::make_nvp("state", state_);
	ar & boost::serialization::make_nvp("runningTime", runningTime_);
	ar & boost::serialization::make_nvp("numberOfKeypoints", numberOfKeypoints_);
	ar & boost::serialization::make_nvp("Triangulations", triangulations_);
}
//BOOST_CLASS_VERSION(R3DProject::ComputeMatches, 1)

// Serialize PictureSet class
template<class Archive>
void R3DProject::PictureSet::serialize(Archive & ar, const unsigned int version)
{
	ar & boost::serialization::make_nvp("id", id_);
	ar & boost::serialization::make_nvp("parentId", parentId_);
	ar & boost::serialization::make_nvp("runningId", runningId_);
	ar & boost::serialization::make_nvp("name", name_);
	ar & boost::serialization::make_nvp("highestLocalImageNr", highestLocalImageNr_);
	ar & boost::serialization::make_nvp("ImageList", imageList_);
	ar & boost::serialization::make_nvp("ComputeMatches", computeMatches_);
}

template<class Archive>
void R3DProject::serialize(Archive & ar, const unsigned int version)
{
	ar & boost::serialization::make_nvp("projectName", projectName_);
	ar & boost::serialization::make_nvp("imagePath", imagePath_);
	ar & boost::serialization::make_nvp("matchesPath", matchesPath_);
	ar & boost::serialization::make_nvp("outPath", outPath_);
	ar & boost::serialization::make_nvp("pmvsPath", pmvsPath_);
	ar & boost::serialization::make_nvp("highestID", highestID_);
	ar & boost::serialization::make_nvp("PictureSets", pictureSets_);
}
