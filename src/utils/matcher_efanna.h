/**
 * Copyright (C) 2017 Roman Hiestand
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

#ifndef MATCHER_EFANNA_H
#define MATCHER_EFANNA_H

#if defined(R3D_HAVE_OPENMP)
#	include <omp.h>
#endif

#include "openMVG/matching/matching_interface.hpp"
#include "openMVG/matching/metric.hpp"

#include <efanna.hpp>


template < typename Scalar = float, typename Metric = openMVG::matching::L2<Scalar> >
class ArrayMatcher_EFANNA : public openMVG::matching::ArrayMatcher<Scalar, Metric>
{
public:
	using DistanceType = typename Metric::ResultType;

	ArrayMatcher_EFANNA() = default;

	virtual ~ArrayMatcher_EFANNA() = default;

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
		memMapping.reset(nullptr);
		return false;
	}
	memMapping.reset(new Eigen::Map<BaseMat>( (Scalar*)dataset, nbRows, dimension));
	dimension_ = dimension;
	nbRows_ = nbRows;
	dataset_ = dataset;

	efanna::Matrix<Scalar> features(nbRows_, dimension_, dataset);
	efanna::IndexParams params;

	// 8 8 3 20 10 10 10
	/**
	8 -- number of trees used to build the graph (larger is more accurate but slower)   
8 -- conquer-to-depeth(smaller is more accurate but slower)   
8 -- number of iterations to build the graph 
 
30 -- L (larger is more accurate but slower, no smaller than K)  
25 -- check (larger is more accurate but slower, no smaller than K)  
10 -- K, for KNN graph
*/

	int trees = 4, mlevel = 4, epochs = 3, checkK = 8, L = 8, kNN = 2, tree_num_build = 0, S = 6;
	index_ = std::make_unique<efanna::FIndex<Scalar> >(features, new efanna::L2DistanceSSE<Scalar>(), 
		efanna::KDTreeUbIndexParams(true, trees ,mlevel ,epochs,checkK,L, 10, tree_num_build, S));

	index_->buildIndex();
	index_->buildTrees();

	int search_epoc = 4, poolsz = 1200, search_extend = 200, search_trees = trees, search_lv = -1, search_method = 1;
	index_->setSearchParams(search_epoc, poolsz, search_extend, search_trees, search_lv, search_method);
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
		if(index_.get() != nullptr)
		{
			// TODO
			//index_->search(soracle, sparams, &knn[0], &dists[0], &info);

			//indice[0] = knn[0];
			//distance[0] = dists[0];
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
		if (!memMapping ||
			NN > memMapping->rows() ||
			nbQuery < 1)
		{
			return false;
		}

		if(index_.get() != nullptr)
		{
			pvec_indices->reserve(nbQuery * NN);
			pvec_distances->reserve(nbQuery * NN);

			// Maybe parallel_for ?
			for(int i = 0; i < nbQuery; i++)
			{
				std::vector<unsigned int> knn(NN);
				std::vector<float> dists(NN);

				efanna::Matrix<Scalar> queryE(1, dimension_, query + i*dimension_);

				const std::vector<std::vector<int> > results = index_->knnSearch(NN, queryE);

				// Calculate the distances
				Metric metric;
				for(int j = 0; j < NN; j++)
				{
					knn[j] = results[0][j];
					dists[j] = metric(query+i*dimension_, dataset_ + knn[j]*dimension_, dimension_);
				}

				// Save the resulting found indices
				for(int j = 0; j < NN; j++)
				{
					pvec_indices->emplace_back(i, knn[j]);
					pvec_distances->emplace_back(dists[j]);
				}

			}
			return true;
		}
		else
		{
			return false;
		}
	}

private:

	std::unique_ptr<efanna::FIndex<Scalar> > index_;
	int nbRows_, dimension_;
	const Scalar *dataset_;

	using BaseMat = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
	/// Use a memory mapping in order to avoid memory re-allocation
	std::unique_ptr< Eigen::Map<BaseMat> > memMapping;
};

#endif // !MATCHER_EFANNA_H
