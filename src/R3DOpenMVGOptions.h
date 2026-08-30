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
#ifndef R3DOPENMVGOPTIONS_H
#define R3DOPENMVGOPTIONS_H

/**
 * The option values of the OpenMVG command line tools.
 *
 * R3DOpenMVGMatchingParams stores indices into these tables, so the dialog that
 * fills them and the process that turns them into a command line must agree on
 * the order. That is why they live here and not in either of the two.
 *
 * The strings are exactly what the executables parse; see main_ComputeFeatures,
 * main_ComputeFeatures_OpenCV, main_PairGenerator, main_ComputeMatches and
 * main_GeometricFilter in openMVG/src/software/SfM.
 */
namespace R3DOpenMVGOptions
{
	// --describerMethod. The OpenCV ones are only understood by
	// openMVG_main_ComputeFeatures_OpenCV, which takes no other options.
	struct Describer
	{
		const wxChar *name_;
		bool binary_;			// Binary descriptors need a Hamming matcher
		bool openCV_;			// Needs openMVG_main_ComputeFeatures_OpenCV
	};

	const Describer kDescribers[] = {
		{ wxT("SIFT"),         false, false },
		{ wxT("SIFT_ANATOMY"), false, false },
		{ wxT("AKAZE_FLOAT"),  false, false },
		{ wxT("AKAZE_MLDB"),   true,  false },
		{ wxT("AKAZE_OPENCV"), false, true  },
		{ wxT("SIFT_OPENCV"),  false, true  }
	};
	const size_t kDescriberCount = WXSIZEOF(kDescribers);

	// --describerPreset
	const wxChar * const kPresets[] = { wxT("NORMAL"), wxT("HIGH"), wxT("ULTRA") };
	const size_t kPresetCount = WXSIZEOF(kPresets);

	// --pair_mode. CONTIGUOUS additionally needs --contiguous_count.
	const wxChar * const kPairModes[] = { wxT("EXHAUSTIVE"), wxT("CONTIGUOUS") };
	const size_t kPairModeCount = WXSIZEOF(kPairModes);

	// --nearest_matching_method, and which descriptor type each one accepts.
	// AUTO works for both: it picks a matcher from the regions type.
	struct Matcher
	{
		const wxChar *name_;
		bool scalar_;
		bool binary_;
	};

	const Matcher kMatchers[] = {
		{ wxT("AUTO"),                 true,  true  },
		{ wxT("BRUTEFORCEL2"),         true,  false },
		{ wxT("HNSWL2"),               true,  false },
		{ wxT("HNSWL1"),               true,  false },
		{ wxT("ANNL2"),                true,  false },
		{ wxT("CASCADEHASHINGL2"),     true,  false },
		{ wxT("FASTCASCADEHASHINGL2"), true,  false },
		{ wxT("BRUTEFORCEHAMMING"),    false, true  },
		{ wxT("HNSWHAMMING"),          false, true  }
	};
	const size_t kMatcherCount = WXSIZEOF(kMatchers);

	/**
	 * Looks up a name by index, falling back to the first entry.
	 *
	 * A project file written by a newer version can hold an index this build
	 * does not know; the defaults are harmless, so use them instead of failing.
	 */
	inline const wxChar *describerName(int index)
	{
		if(index < 0 || index >= static_cast<int>(kDescriberCount))
			index = 0;
		return kDescribers[index].name_;
	}

	inline bool describerIsBinary(int index)
	{
		if(index < 0 || index >= static_cast<int>(kDescriberCount))
			index = 0;
		return kDescribers[index].binary_;
	}

	inline bool describerIsOpenCV(int index)
	{
		if(index < 0 || index >= static_cast<int>(kDescriberCount))
			index = 0;
		return kDescribers[index].openCV_;
	}

	inline const wxChar *presetName(int index)
	{
		if(index < 0 || index >= static_cast<int>(kPresetCount))
			index = 0;
		return kPresets[index];
	}

	inline const wxChar *pairModeName(int index)
	{
		if(index < 0 || index >= static_cast<int>(kPairModeCount))
			index = 0;
		return kPairModes[index];
	}

	inline const wxChar *matcherName(int index)
	{
		if(index < 0 || index >= static_cast<int>(kMatcherCount))
			index = 0;
		return kMatchers[index].name_;
	}

	// ---------------------------------------------------- openMVG_main_SfM ---

	// --sfm_engine, indexed by R3DProject::R3DTriangulationAlgorithm
	// (R3DTA_Incremental1, R3DTA_Incremental2, R3DTA_Global)
	const wxChar * const kSfMEngines[] = {
		wxT("INCREMENTAL"), wxT("INCREMENTALV2"), wxT("GLOBAL") };
	const size_t kSfMEngineCount = WXSIZEOF(kSfMEngines);

	// --sfm_initializer, indexed by R3DProject::R3DTriangulationInitialization.
	// Only INCREMENTALV2 reads it; the built-in engine has the first two only.
	const wxChar * const kSceneInitializers[] = {
		wxT("MAX_PAIR"), wxT("STELLAR"), wxT("AUTO_PAIR"), wxT("EXISTING_POSE") };
	const size_t kSceneInitializerCount = WXSIZEOF(kSceneInitializers);

	// --refine_intrinsic_config. NONE is not in the list: the dialog's
	// "Refine camera intrinsics" checkbox is what turns the refinement off.
	const wxChar * const kIntrinsicRefinements[] = {
		wxT("ADJUST_ALL"),
		wxT("ADJUST_FOCAL_LENGTH"),
		wxT("ADJUST_PRINCIPAL_POINT"),
		wxT("ADJUST_DISTORTION"),
		wxT("ADJUST_FOCAL_LENGTH|ADJUST_PRINCIPAL_POINT"),
		wxT("ADJUST_FOCAL_LENGTH|ADJUST_DISTORTION"),
		wxT("ADJUST_PRINCIPAL_POINT|ADJUST_DISTORTION") };
	const size_t kIntrinsicRefinementCount = WXSIZEOF(kIntrinsicRefinements);

	// --refine_extrinsic_config
	const wxChar * const kExtrinsicRefinements[] = {
		wxT("ADJUST_ALL"), wxT("NONE") };
	const size_t kExtrinsicRefinementCount = WXSIZEOF(kExtrinsicRefinements);

	// --triangulation_method and --resection_method are passed as numbers;
	// these are openMVG's ETriangulationMethod and resection::SolverType, whose
	// values are the indices of the entries the dialog shows.
	const int kTriangulationMethodCount = 4;	// 3 = INVERSE_DEPTH_WEIGHTED_MIDPOINT, the default
	const int kResectionMethodCount = 5;		// 3 = P3P_NORDBERG_ECCV18, the default

	// -M, the filtered matches file the reconstruction is built from
	const wxChar * const kMatchesFiles[] = {
		wxT(""),					// automatic: chosen from the SfM engine
		wxT("matches.f.txt"),
		wxT("matches.e.txt"),
		wxT("matches.h.txt") };
	const size_t kMatchesFileCount = WXSIZEOF(kMatchesFiles);

	inline const wxChar *sfmEngineName(int algorithm)
	{
		if(algorithm < 0 || algorithm >= static_cast<int>(kSfMEngineCount))
			algorithm = 0;
		return kSfMEngines[algorithm];
	}

	inline const wxChar *sceneInitializerName(int index)
	{
		if(index < 0 || index >= static_cast<int>(kSceneInitializerCount))
			index = 0;
		return kSceneInitializers[index];
	}

	inline const wxChar *intrinsicRefinementName(int index)
	{
		if(index < 0 || index >= static_cast<int>(kIntrinsicRefinementCount))
			index = 0;
		return kIntrinsicRefinements[index];
	}

	inline const wxChar *extrinsicRefinementName(int index)
	{
		if(index < 0 || index >= static_cast<int>(kExtrinsicRefinementCount))
			index = 0;
		return kExtrinsicRefinements[index];
	}

	inline const wxChar *matchesFileName(int index)
	{
		if(index < 0 || index >= static_cast<int>(kMatchesFileCount))
			index = 0;
		return kMatchesFiles[index];
	}
}

#endif
