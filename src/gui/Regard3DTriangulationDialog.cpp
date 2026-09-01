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
#include "Regard3DTriangulationDialog.h"
#include "PreviewGeneratorThread.h"
#include "OpenMVGHelper.h"
#include "ImageInfo.h"
#include "Regard3DSettings.h"
#include "R3DOpenMVGOptions.h"
#include "R3DExternalPrograms.h"

enum
{
   ID_R3D_TD_TIMER = 3800
};

Regard3DTriangulationDialog* Regard3DTriangulationDialog::pDialog_ = NULL;

Regard3DTriangulationDialog::Regard3DTriangulationDialog(wxWindow* pParent)
   : Regard3DTriangulationDialogBase(pParent),
   aTimer_(this, ID_R3D_TD_TIMER),
   pPreviewGeneratorThread_(NULL),
   pProject_(NULL), pComputeMatches_(NULL), pPictureSet_(NULL),
   isGlobalSfmAvailable_(false), initialImagePairListIsEmpty_(false),
   isOpenMVGSfMAvailable_(false),
   ipSortColumn_(0)
{
   for (int i = 0; i < 5; i++)
      ipSortDirections_[i] = 0;
}

Regard3DTriangulationDialog::~Regard3DTriangulationDialog()
{
}

void Regard3DTriangulationDialog::setPreviewGeneratorThread(PreviewGeneratorThread* pPreviewGeneratorThread)
{
   pPreviewGeneratorThread_ = pPreviewGeneratorThread;
}

void Regard3DTriangulationDialog::setComputeMatches(R3DProject* pProject, R3DProject::ComputeMatches* pComputeMatches)
{
   pProject_ = pProject;
   pComputeMatches_ = pComputeMatches;
   R3DProject::Object* pObject = pProject->getObjectByTypeAndID(R3DProject::R3DTreeItem::TypePictureSet, pComputeMatches->parentId_);
   if (pObject != NULL)
   {
      pPictureSet_ = dynamic_cast<R3DProject::PictureSet*>(pObject);
   }
   if (pPictureSet_ == NULL)
      return;
   pProject_->getProjectPathsCM(paths_, pComputeMatches);

   bool enableUseGPSInfo = false;
   int numberOfImagesWithGPSInfo = 0;
   for (const auto& imageInfo : pPictureSet_->imageList_)
   {
      if (imageInfo.hasGPSInfo_)
         numberOfImagesWithGPSInfo++;
   }
   if (numberOfImagesWithGPSInfo > 2)
      enableUseGPSInfo = true;
   pUseGPSCheckBox_->Enable(enableUseGPSInfo);
}

bool Regard3DTriangulationDialog::isTriangulationPossible()
{
   // Check whether preconditions of incremental or global chain are met
   bool isIncrementalTriPossible = false, isGlobalTriPossible = false;

   isIncrementalTriPossible = true;	// We now allow incremental triangulation without known focal length
   /*	// Incremental: At least one pair with known focal length
      OpenMVGHelper::ImagePairList imgPairList = OpenMVGHelper::getBestValidatedPairs(paths_, paths_.matchesFFilename_, 0);
      if(!imgPairList.empty())
      {
         const ImageInfoVector &imageInfoVector = pPictureSet_->imageList_;
         for(size_t i = 0; i < imgPairList.size(); i++)
         {
            const OpenMVGHelper::ImagePair &imgPair = imgPairList[i];

            bool focalLengthKnown1 = false, focalLengthKnown2 = false;
            if(imageInfoVector.size() > imgPair.indexA_)
            {
               const ImageInfo &imageInfo = imageInfoVector[imgPair.indexA_];
               focalLengthKnown1 = ((imageInfo.sensorWidth_ != 0)
                  && (imageInfo.focalLength_ != 0));
            }
            if(imageInfoVector.size() > imgPair.indexB_)
            {
               const ImageInfo &imageInfo = imageInfoVector[imgPair.indexB_];
               focalLengthKnown2 = ((imageInfo.sensorWidth_ != 0)
                  && (imageInfo.focalLength_ != 0));
            }

            if(focalLengthKnown1 && focalLengthKnown2)
            {
               isIncrementalTriPossible = true;
            }
         }
      }
      else
      {
         isIncrementalTriPossible = false;
      }*/
      // Store for later
   initialImagePairListIsEmpty_ = !isIncrementalTriPossible;

   // Global triangulation
   isGlobalTriPossible = OpenMVGHelper::isGlobalSfmAvailable(paths_, pPictureSet_);
   isGlobalSfmAvailable_ = isGlobalTriPossible;	// Store for later

   return (isIncrementalTriPossible || isGlobalTriPossible);
}

void Regard3DTriangulationDialog::getResults(R3DTriangulationDialogResults& results)
{
   R3DProject::R3DTriangulationAlgorithm algorithm;
   size_t initialImageID1 = 0, initialImageID2 = 0;

   if (pTriangulationChoicebook_->GetSelection() == 0)
   {
      algorithm = R3DProject::R3DTriangulationAlgorithm::R3DTA_Incremental2;
   }
   else if (pTriangulationChoicebook_->GetSelection() == 1)
   {
      algorithm = R3DProject::R3DTriangulationAlgorithm::R3DTA_Incremental1;
   }
   else
   {
      algorithm = R3DProject::R3DTriangulationAlgorithm::R3DTA_Global;
   }

   int stateMask = wxLIST_STATE_SELECTED;
   int imagePairCount = pTInitialImagePairListCtrl_->GetItemCount();
   for (int i = 0; i < imagePairCount; i++)
   {
      int itemState = pTInitialImagePairListCtrl_->GetItemState(i, stateMask);
      if (itemState != 0)
      {
         int index = pTInitialImagePairListCtrl_->GetItemData(i);
         // This item is selected, determine image indices
         if (static_cast<size_t>(index) < imageIDList_.size())
         {
            initialImageID1 = imageIDList_[index].first;
            initialImageID2 = imageIDList_[index].second;
         }
      }
   }

   results_.engine_ = pTriEngineRadioBox_->GetSelection();
   results_.algorithm_ = algorithm;
   results_.initialImageID1_ = initialImageID1;
   results_.initialImageID2_ = initialImageID2;
   results_.rotAveraging_ = pTGlobalRotAvgMethodRatioBox_->GetSelection() + 1;
   results_.transAveraging_ = pTGlobalTranslAvgMethodRadioBox_->GetSelection() + 1;
   results_.refineIntrinsics_ = pTRefineCameraIntrinsicsCheckBox_->GetValue();

   results_.useGPSInfo_ = false;
   if (pUseGPSCheckBox_->IsEnabled())
      results_.useGPSInfo_ = pUseGPSCheckBox_->GetValue();

   // The last two initializers only exist in openMVG_main_SfM; the radio box
   // disables them for the built-in engine, so the value is always legal here
   results_.triInitialization_ = static_cast<R3DProject::R3DTriangulationInitialization>(
      pIncrSFMInitRadioBox_->GetSelection());

   readOpenMVGOptions();

   results = results_;
}

/**
 * Whether openMVG_main_SfM could run on this compute matches node.
 *
 * It reads the features through image_describer.json, which only the OpenMVG
 * feature executables write, so a node computed by the built-in engine cannot
 * be reconstructed by the external one.
 */
bool Regard3DTriangulationDialog::isOpenMVGSfMPossible(wxString& reason)
{
   if (R3DExternalPrograms::getInstance().getSfMPath().IsEmpty())
   {
      reason = wxT("openMVG_main_SfM was not found. Please put it into the ")
         wxT("\"openmvg\" directory next to Regard3D.");
      return false;
   }

   wxFileName describerFN(paths_.absoluteMatchesPath_, wxT("image_describer.json"));
   if (!describerFN.FileExists())
   {
      reason = wxT("These matches were computed by the built-in engine, which ")
         wxT("writes no image_describer.json. Compute the matches with the ")
         wxT("OpenMVG engine to be able to use openMVG_main_SfM.");
      return false;
   }

   return true;
}

void Regard3DTriangulationDialog::setOpenMVGToolTips()
{
   pTriEngineRadioBox_->SetToolTip(
      wxT("Who does the reconstruction. The three methods above mean the same\n")
      wxT("either way: they are openMVG_main_SfM's INCREMENTALV2, INCREMENTAL\n")
      wxT("and GLOBAL engines."));
   pOMVGIntrinsicRefinementChoice_->SetToolTip(
      wxT("Which camera intrinsics bundle adjustment may change.\n")
      wxT("Restrict this when the calibration is known and should be kept.\n")
      wxT("Clear \"Refine camera intrinsics\" to hold all of them fixed."));
   pOMVGExtrinsicRefinementChoice_->SetToolTip(
      wxT("Which camera poses bundle adjustment may change.\n")
      wxT("None keeps the poses as the initializer produced them.\n")
      wxT("Only the incremental method above reads this; the old incremental\n")
      wxT("and the global one always adjust the poses."));
   pOMVGTriangulationMethodChoice_->SetToolTip(
      wxT("How a 3D point is computed from its observations.\n")
      wxT("Inverse depth weighted midpoint is OpenMVG's default and the most\n")
      wxT("robust one; direct linear transform is the classic, cheaper method."));
   pOMVGResectionMethodChoice_->SetToolTip(
      wxT("How the pose of a newly added image is estimated.\n")
      wxT("P3P Nordberg is OpenMVG's default. The 6 point transform ignores the\n")
      wxT("known intrinsics, and UP2P assumes an upright camera."));
   pOMVGSfMCameraModelChoice_->SetToolTip(
      wxT("The camera model given to views whose intrinsics are unknown.\n")
      wxT("Regard3D writes intrinsics for every view when it creates the scene,\n")
      wxT("so this rarely applies."));
   pOMVGMatchesFileChoice_->SetToolTip(
      wxT("Which filtered matches the reconstruction is built from.\n")
      wxT("Automatic uses matches.e.txt for the global method and matches.f.txt\n")
      wxT("for the incremental ones, which is what each of them expects."));

   // Say why the two greyed out initializers are greyed out
   pIncrSFMInitRadioBox_->SetItemToolTip(2,
      wxT("Not available: openMVG has not implemented this initializer,\n")
      wxT("openMVG_main_SfM stops with \"Not yet implemented\"."));
   pIncrSFMInitRadioBox_->SetItemToolTip(3,
      wxT("Not available: this starts from camera poses that are already in\n")
      wxT("the scene, and the scene of a compute matches node has none."));
}

void Regard3DTriangulationDialog::initializeOpenMVGOptions()
{
   const R3DOpenMVGTriangulationParams& p = results_.openMVG_;

   pOMVGIntrinsicRefinementChoice_->SetSelection(p.intrinsicRefinement_);
   pOMVGExtrinsicRefinementChoice_->SetSelection(p.extrinsicRefinement_);
   pOMVGTriangulationMethodChoice_->SetSelection(p.triangulationMethod_);
   pOMVGResectionMethodChoice_->SetSelection(p.resectionMethod_);
   pOMVGSfMCameraModelChoice_->SetSelection(p.cameraModel_ - 1);
   pOMVGMatchesFileChoice_->SetSelection(p.matchesFile_);
}

void Regard3DTriangulationDialog::readOpenMVGOptions()
{
   R3DOpenMVGTriangulationParams p;

   p.intrinsicRefinement_ = pOMVGIntrinsicRefinementChoice_->GetCurrentSelection();
   p.extrinsicRefinement_ = pOMVGExtrinsicRefinementChoice_->GetCurrentSelection();
   p.triangulationMethod_ = pOMVGTriangulationMethodChoice_->GetCurrentSelection();
   p.resectionMethod_ = pOMVGResectionMethodChoice_->GetCurrentSelection();
   p.cameraModel_ = pOMVGSfMCameraModelChoice_->GetCurrentSelection() + 1;
   p.matchesFile_ = pOMVGMatchesFileChoice_->GetCurrentSelection();

   results_.openMVG_ = p;
}

/**
 * Enables only what the selected engine and method actually use.
 */
void Regard3DTriangulationDialog::updateEngineDependencies()
{
   const bool openMVG = (pTriEngineRadioBox_->GetSelection() == 1);
   const int page = pTriangulationChoicebook_->GetSelection();
   // Page 0 is INCREMENTALV2, page 1 INCREMENTAL, page 2 GLOBAL
   const bool incremental = (page == 0 || page == 1);

   // Neither of the last two initializers can produce a reconstruction here,
   // whichever engine runs: openMVG_main_SfM answers AUTO_PAIR with "Not yet
   // implemented", and EXISTING_POSE starts from poses that the scene of a
   // compute matches node does not have. They keep their place in the list so
   // that the numbers stored in project files keep their meaning.
   pIncrSFMInitRadioBox_->Enable(2, false);
   pIncrSFMInitRadioBox_->Enable(3, false);
   if (pIncrSFMInitRadioBox_->GetSelection() > 1)
      pIncrSFMInitRadioBox_->SetSelection(0);

   // Both engines refine through the same openMVG bundle adjustment, so
   // these two are not tied to the external one. The extrinsics option is
   // read by the newer incremental engine alone (sequential_SfM2.cpp).
   pOMVGIntrinsicRefinementChoice_->Enable(pTRefineCameraIntrinsicsCheckBox_->GetValue());
   pOMVGExtrinsicRefinementChoice_->Enable(page == 0);
   // -t and -r are read by the incremental engines of openMVG_main_SfM only
   pOMVGTriangulationMethodChoice_->Enable(openMVG && incremental);
   pOMVGResectionMethodChoice_->Enable(openMVG && incremental);
   // The camera model is used by the incremental engines, either engine
   pOMVGSfMCameraModelChoice_->Enable(incremental);
   pOMVGMatchesFileChoice_->Enable(openMVG);
}

void Regard3DTriangulationDialog::OnPreviewFinished()
{
   checkForPreviewImage();
}

void Regard3DTriangulationDialog::OnNewImageInfos()
{
}

void Regard3DTriangulationDialog::EndModal(int retCode)
{
   Regard3DSettings::getInstance().saveTriangulationLayoutToConfig(this);

   wxDialog::EndModal(retCode);
}

void Regard3DTriangulationDialog::OnInitDialog(wxInitDialogEvent& event)
{
   updateInitialImagePairListCtrl();
   updateTriangulationMethodChoice();

   wxString reason;
   isOpenMVGSfMAvailable_ = isOpenMVGSfMPossible(reason);
   if (!isOpenMVGSfMAvailable_)
   {
      // Say why, rather than offering a choice that cannot be taken
      pTriEngineRadioBox_->Enable(1, false);
      pTriEngineRadioBox_->SetItemToolTip(1, reason);
      results_.engine_ = 0;
   }
   pTriEngineRadioBox_->SetSelection(results_.engine_);

   setOpenMVGToolTips();
   initializeOpenMVGOptions();

   pTriEngineRadioBox_->Bind(wxEVT_RADIOBOX,
      [this](wxCommandEvent &) { updateEngineDependencies(); });
   pTriangulationChoicebook_->Bind(wxEVT_CHOICEBOOK_PAGE_CHANGED,
      [this](wxBookCtrlEvent &event) { event.Skip(); updateEngineDependencies(); });
   pTRefineCameraIntrinsicsCheckBox_->Bind(wxEVT_CHECKBOX,
      [this](wxCommandEvent &) { updateEngineDependencies(); });

   updateEngineDependencies();

   Regard3DSettings::getInstance().loadTriangulationLayoutFromConfig(this);
   aTimer_.Start(50);
}

// returns 0 if the items are equal, negative value if the first item is less than the second one and positive value if the first one is greater than the second one 
#if wxCHECK_VERSION(2, 9, 0)
int wxCALLBACK Regard3DTriangulationDialog::TInitialImagePairListCompareFunction(wxIntPtr item1, wxIntPtr item2, wxIntPtr WXUNUSED(sortData))
#else
int wxCALLBACK Regard3DTriangulationDialog::TInitialImagePairListCompareFunction(long item1, long item2, long WXUNUSED(sortData))
#endif
{
   Regard3DTriangulationDialog* pDialog = Regard3DTriangulationDialog::pDialog_;
   wxListCtrl* pListCtrl = pDialog->pTInitialImagePairListCtrl_;
   long index1 = pListCtrl->FindItem(-1, item1);	// Find list entry with data item1
   long index2 = pListCtrl->FindItem(-1, item2);
   int col = pDialog->ipSortColumn_;
   bool reverseOrder = (pDialog->ipSortDirections_[col] == 1);

   size_t item1ImageA = 0, item1ImageB = 0, item2ImageA = 0, item2ImageB = 0;
   if (static_cast<size_t>(item1) < pDialog->imageIDList_.size())
   {
      item1ImageA = pDialog->imageIDList_[item1].first;
      item1ImageB = pDialog->imageIDList_[item1].second;
   }
   if (static_cast<size_t>(item2) < pDialog->imageIDList_.size())
   {
      item2ImageA = pDialog->imageIDList_[item2].first;
      item2ImageB = pDialog->imageIDList_[item2].second;
   }

   if (col == 0)
   {
      int cmp = 0;
      if (item1ImageA < item2ImageA)
         cmp = -1;
      else if (item1ImageA > item2ImageA)
         cmp = 1;
      if (reverseOrder)
         cmp = -cmp;
      return cmp;
   }
   else if (col == 2)
   {
      int cmp = 0;
      if (item1ImageB < item2ImageB)
         cmp = -1;
      else if (item1ImageB > item2ImageB)
         cmp = 1;
      if (reverseOrder)
         cmp = -cmp;
      return cmp;
   }
#if wxCHECK_VERSION(2, 9, 1)
   else if (col == 1)
   {
      wxString text1 = pListCtrl->GetItemText(index1, 1);	// The last parameter only works after wxWidgets 2.9.1
      wxString text2 = pListCtrl->GetItemText(index2, 1);

      int cmp = text1.Cmp(text2);
      if (reverseOrder)
         cmp = -cmp;
      return cmp;
   }
   else if (col == 3)
   {
      wxString text1 = pListCtrl->GetItemText(index1, 3);	// The last parameter only works after wxWidgets 2.9.1
      wxString text2 = pListCtrl->GetItemText(index2, 3);

      int cmp = text1.Cmp(text2);
      if (reverseOrder)
         cmp = -cmp;
      return cmp;
   }
   else if (col == 4)
   {
      wxString text1 = pListCtrl->GetItemText(index1, 4);	// The last parameter only works after wxWidgets 2.9.1
      wxString text2 = pListCtrl->GetItemText(index2, 4);
      long val1, val2;
      text1.ToCLong(&val1);
      text2.ToCLong(&val2);

      int cmp = 0;
      if (val1 < val2)
         cmp = -1;
      else if (val1 > val2)
         cmp = 1;
      if (reverseOrder)
         cmp = -cmp;
      return cmp;
   }
#endif

   return 0;
}

void Regard3DTriangulationDialog::OnTInitialImagePairColClick(wxListEvent& event)
{
   int col = event.GetColumn();
   if (0 <= col
      && col < pTInitialImagePairListCtrl_->GetColumnCount())
   {
      ipSortColumn_ = col;
      pDialog_ = this;
      pTInitialImagePairListCtrl_->SortItems(TInitialImagePairListCompareFunction, 0);

      ipSortDirections_[col] = 1 - ipSortDirections_[col];
   }
}

void Regard3DTriangulationDialog::OnTInitialImagePairItemDeselected(wxListEvent& event)
{
   // Clear preview
   pPreviewCanvas_->clearPreviewImage();
   previewInfoMatches_.clear();
}

void Regard3DTriangulationDialog::OnTInitialImagePairItemSelected(wxListEvent& event)
{
   // Create preview
   long i = event.GetIndex();
   int index = pTInitialImagePairListCtrl_->GetItemData(i);
   int imageID1 = 0, imageID2 = 0;
   if (index < static_cast<long>(imageIDList_.size()))
   {
      imageID1 = static_cast<int>(imageIDList_[index].first);
      imageID2 = static_cast<int>(imageIDList_[index].second);
   }

   PreviewInfo previewInfo = pProject_->prepareMatchesPreview(pComputeMatches_->id_, imageID1, imageID2);

   if (pTPreviewWithMatchesCheckBox_->GetValue())
      previewInfo.keypointType_ = PreviewInfo::PIKPTRichKeypoints;
   else
      previewInfo.keypointType_ = PreviewInfo::PIKPTNoKeypoints;

   pPreviewGeneratorThread_->addPreviewRequest(previewInfo);
}

void Regard3DTriangulationDialog::OnTPreviewWithMatchesCheckBox(wxCommandEvent& event)
{
   pPreviewCanvas_->clearPreviewImage();

   PreviewInfo previewInfo = pProject_->prepareMatchesPreview(pComputeMatches_->id_,
      previewInfoMatches_.index1_, previewInfoMatches_.index2_);
   if (event.IsChecked())
      previewInfo.keypointType_ = PreviewInfo::PIKPTRichKeypoints;
   else
      previewInfo.keypointType_ = PreviewInfo::PIKPTNoKeypoints;

   pPreviewGeneratorThread_->addPreviewRequest(previewInfo);
}

void Regard3DTriangulationDialog::OnTimer(wxTimerEvent& WXUNUSED(event))
{
   checkForPreviewImage();
}

void Regard3DTriangulationDialog::updateInitialImagePairListCtrl()
{
   pTInitialImagePairListCtrl_->ClearAll();
   imageIDList_.clear();

   // This was checked before in isTriangulationPossible()
   if (initialImagePairListIsEmpty_)
      return;

   OpenMVGHelper::ImagePairList imgPairList = OpenMVGHelper::getBestValidatedPairs(paths_, paths_.matchesFFilename_, 0);
   if (!imgPairList.empty())
   {
      const ImageInfoVector& imageInfoVector = pPictureSet_->imageList_;
      pTInitialImagePairListCtrl_->InsertColumn(pTInitialImagePairListCtrl_->GetColumnCount(), wxT("Index 1"));
      pTInitialImagePairListCtrl_->InsertColumn(pTInitialImagePairListCtrl_->GetColumnCount(), wxT("Image 1"));
      pTInitialImagePairListCtrl_->InsertColumn(pTInitialImagePairListCtrl_->GetColumnCount(), wxT("Index 2"));
      pTInitialImagePairListCtrl_->InsertColumn(pTInitialImagePairListCtrl_->GetColumnCount(), wxT("Image 2"));
      pTInitialImagePairListCtrl_->InsertColumn(pTInitialImagePairListCtrl_->GetColumnCount(), wxT("# Matches"));

      size_t newID = 0;

      bool noPairHasFullImageInfo = true;
      for (size_t i = 0; i < imgPairList.size(); i++)
      {
         const OpenMVGHelper::ImagePair& imgPair = imgPairList[i];
         bool focalLengthKnown1 = false, focalLengthKnown2 = false;
         if (imageInfoVector.size() > imgPair.indexA_)
         {
            const ImageInfo& imageInfo = imageInfoVector[imgPair.indexA_];
            focalLengthKnown1 = ((imageInfo.sensorWidth_ != 0)
               && (imageInfo.focalLength_ != 0));
         }
         if (imageInfoVector.size() > imgPair.indexB_)
         {
            const ImageInfo& imageInfo = imageInfoVector[imgPair.indexB_];
            focalLengthKnown2 = ((imageInfo.sensorWidth_ != 0)
               && (imageInfo.focalLength_ != 0));
         }
         if (focalLengthKnown1 && focalLengthKnown2)
            noPairHasFullImageInfo = false;
      }

      for (size_t i = 0; i < imgPairList.size(); i++)
      {
         const OpenMVGHelper::ImagePair& imgPair = imgPairList[i];

         wxString imageFilenameA(imgPair.imageFilenameA_.c_str(), *wxConvCurrent);
         wxString imageFilenameB(imgPair.imageFilenameB_.c_str(), *wxConvCurrent);

         wxFileName imageAFN(imageFilenameA);
         wxFileName imageBFN(imageFilenameB);

         bool focalLengthKnown1 = false, focalLengthKnown2 = false;
         if (imageInfoVector.size() > imgPair.indexA_)
         {
            const ImageInfo& imageInfo = imageInfoVector[imgPair.indexA_];
            imageAFN = wxFileName(imageInfo.filename_);
            focalLengthKnown1 = ((imageInfo.sensorWidth_ != 0)
               && (imageInfo.focalLength_ != 0));
         }
         if (imageInfoVector.size() > imgPair.indexB_)
         {
            const ImageInfo& imageInfo = imageInfoVector[imgPair.indexB_];
            imageBFN = wxFileName(imageInfo.filename_);
            focalLengthKnown2 = ((imageInfo.sensorWidth_ != 0)
               && (imageInfo.focalLength_ != 0));
         }

         if (noPairHasFullImageInfo
            || (focalLengthKnown1 && focalLengthKnown2))
         {
            pTInitialImagePairListCtrl_->InsertItem(static_cast<long>(newID), wxString::Format(wxT("%zu"), imgPair.indexA_));
            pTInitialImagePairListCtrl_->SetItem(static_cast<long>(newID), 1, imageAFN.GetFullName());
            pTInitialImagePairListCtrl_->SetItem(static_cast<long>(newID), 2, wxString::Format(wxT("%zu"), imgPair.indexB_));
            pTInitialImagePairListCtrl_->SetItem(static_cast<long>(newID), 3, imageBFN.GetFullName());
            pTInitialImagePairListCtrl_->SetItem(static_cast<long>(newID), 4, wxString::Format(wxT("%zu"), imgPair.matches_));
            pTInitialImagePairListCtrl_->SetItemData(static_cast<long>(newID), static_cast<long>(newID));	// Used in sorting

            // Store image indexes in imageIDList_
            imageIDList_.push_back(std::make_pair(imgPair.indexA_, imgPair.indexB_));

            newID++;
         }
      }

      if (newID > 0)
      {
         int stateMask = wxLIST_STATE_SELECTED;
         pTInitialImagePairListCtrl_->SetItemState(0, stateMask, stateMask);	// Select first item
         pTInitialImagePairListCtrl_->SetColumnWidth(0, wxLIST_AUTOSIZE);
         pTInitialImagePairListCtrl_->SetColumnWidth(1, wxLIST_AUTOSIZE);
         pTInitialImagePairListCtrl_->SetColumnWidth(2, wxLIST_AUTOSIZE);
         pTInitialImagePairListCtrl_->SetColumnWidth(3, wxLIST_AUTOSIZE);
         pTInitialImagePairListCtrl_->SetColumnWidth(4, wxLIST_AUTOSIZE);
      }
   }
}

void Regard3DTriangulationDialog::updateTriangulationMethodChoice()
{
   if (!isGlobalSfmAvailable_)
      pTriangulationChoicebook_->RemovePage(2);
   pTriangulationChoicebook_->SetSelection(0);

   pTriangulationPanel_->Layout();
}

void Regard3DTriangulationDialog::checkForPreviewImage()
{
   if (pPreviewGeneratorThread_ == NULL)
      return;

   wxImage previewImage;
   PreviewInfo previewInfo;
   if (pPreviewGeneratorThread_->getNewPreviewImage(previewImage, previewInfo))
   {
      pPreviewCanvas_->previewImage(previewImage);
      pPreviewCanvas_->Refresh();
      previewInfoMatches_ = previewInfo;
   }
}

BEGIN_EVENT_TABLE(Regard3DTriangulationDialog, Regard3DTriangulationDialogBase)
EVT_TIMER(ID_R3D_TD_TIMER, Regard3DTriangulationDialog::OnTimer)
END_EVENT_TABLE()

