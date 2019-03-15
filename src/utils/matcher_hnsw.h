/**
 * Copyright (C) 2018 Roman Hiestand
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

#ifndef MATCHER_HNSW_H
#define MATCHER_HNSW_H

#if defined(R3D_HAVE_OPENMP)
#	include <omp.h>
#endif

#include "openMVG/matching/matching_interface.hpp"
#include "openMVG/matching/metric.hpp"

#include "hnswlib.h"


template < typename Scalar = float, typename Metric = openMVG::matching::L2<Scalar> >
class ArrayMatcher_hnsw : public openMVG::matching::ArrayMatcher<Scalar, Metric>
{
public:
	using DistanceType = typename Metric::ResultType;

	ArrayMatcher_hnsw() = default;

	virtual ~ArrayMatcher_hnsw() = default;

  /**
   * Build the matching structure
   *
   * \param[in] dataset   Input data.
   * \param[in] nbRows    The number of component.
   * \param[in] dimension Length of the data contained in the each
   *  row of the dataset.
   *
   * \return True if success.
   */
  bool Build
  (
	const Scalar * dataset,
	int nbRows,
	int dimension
  ) override
  {
	if (nbRows < 1)
	{
		return false;
	}


	dimension_ = dimension;
	nbRows_ = nbRows;
	dataset_ = dataset;

	l2space_.reset(new hnswlib::L2Space(dimension)); //std::unique_ptr<hnswlib::L2Space>(new hnswlib::L2Space(dimension));

	size_t max_elements = static_cast<size_t>(nbRows);
	apprAlg_.reset(new hnswlib::HierarchicalNSW<Scalar>(l2space_.get(), max_elements, M_, efConstruction_));

	// Add the first point single-threaded
	apprAlg_->addPoint(const_cast<Scalar *>(dataset_), 0);

	// Add the rest multi-threaded
#pragma omp parallel for
	for(int i = 1; i < nbRows; i++)
		apprAlg_->addPoint(const_cast<Scalar *>(dataset_) + dimension_*i, i);

	apprAlg_->setEf(static_cast<size_t>(ef_));
	return true;
}

	/**
	 * Search the nearest Neighbor of the scalar array query.
	 *
	 * \param[in]   query     The query array
	 * \param[out]  indice    The indice of array in the dataset that
	 *  have been computed as the nearest array.
	 * \param[out]  distance  The distance between the two arrays.
	 *
	 * \return True if success.
	 */
	bool SearchNeighbour
	(
		const Scalar * query,
		int * indice,
		DistanceType * distance
	) override
	{
		if(apprAlg_ != nullptr)
		{
			std::priority_queue<std::pair<Scalar, hnswlib::labeltype> > knn = apprAlg_->searchKnn(const_cast<Scalar *>(query), 1);

			auto te = knn.top();

			indice[0] = te.second;
			distance[0] = te.first;
	  }
	  else
	  {
		  return false;
	  }

		return true;
  }


  /**
	 * Search the N nearest Neighbor of the scalar array query.
	 *
	 * \param[in]   query           The query array
	 * \param[in]   nbQuery         The number of query rows
	 * \param[out]  indices   The corresponding (query, neighbor) indices
	 * \param[out]  pvec_distances  The distances between the matched arrays.
	 * \param[in]  NN              The number of maximal neighbor that will be searched.
	 *
	 * \return True if success.
	 */
	bool SearchNeighbours
	(
		const Scalar * query, int nbQuery,
		openMVG::matching::IndMatches * pvec_indices,
		std::vector<DistanceType> * pvec_distances,
		size_t NN
	) override
	{
		if (//!memMapping ||
//			NN > memMapping->rows() ||
			nbQuery < 1)
		{
			return false;
		}

		if(apprAlg_ != nullptr)
		{
			pvec_indices->reserve(nbQuery * NN);
			pvec_distances->reserve(nbQuery * NN);

			std::vector<std::pair<Scalar, hnswlib::labeltype> > knn_copy;

			// Maybe parallel_for ?
			for(int i = 0; i < nbQuery; i++)
			{
				std::priority_queue<std::pair<Scalar, hnswlib::labeltype> > knn = apprAlg_->searchKnn(const_cast<Scalar *>(query) + dimension_*i, NN);

				knn_copy.clear();

				while(!knn.empty())
				{
					auto te = knn.top();
					knn_copy.push_back(te);
					knn.pop();
				}

				std::reverse(std::begin(knn_copy), std::end(knn_copy));

				//Metric metric;
				// Save the resulting found indices
				for(int j = 0; j < NN; j++)
				{
					int index = knn_copy[j].second;
					//Scalar dist = metric(query+i*dimension_, dataset_ + index*dimension_, dimension_);
					Scalar dist = knn_copy[j].first;

					pvec_indices->emplace_back(i, index);
					pvec_distances->emplace_back(dist);
				}

			}
			return true;
		}
		else
		{
			return false;
		}
	}

	// Parameters for HNSW
	// See also
	// https://github.com/nmslib/nmslib/blob/master/python_bindings/parameters.md
	static int efConstruction_;
	static int ef_;
	static int M_;

private:

	// Only works with Scalar==float
	std::unique_ptr<hnswlib::L2Space> l2space_{nullptr};
	std::unique_ptr<hnswlib::HierarchicalNSW<Scalar> > apprAlg_{nullptr};
	int nbRows_{0}, dimension_{0};
	const Scalar *dataset_{nullptr};
};

#endif // !MATCHER_HNSW_H
