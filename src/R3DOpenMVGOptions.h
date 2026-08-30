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
}

#endif
