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

#ifndef MATCHER_MRPT_H
#define MATCHER_MRPT_H

#if defined(R3D_HAVE_OPENMP)
#	include <omp.h>
#endif

#include "openMVG/matching/matching_interface.hpp"
#include "openMVG/matching/metric.hpp"

#include <mrpt.h>

#include <random>

#include <mutex>

// Parameters from https://github.com/ejaasaari/mrpt-comparison/blob/master/parameters/sift.sh
// MRPT_N_TREES="1 5 10 25 50 75 100 125 150 200 250 300 350 400 500 600"
// MRPT_DEPTH="9 10 11 12 13 14 15 16 17"
// MRPT_SPARSITY=0.088
// MRPT_VOTING_N_TREES="10 25 50 75 100 125 150 200 250 300 350 400 500 600 700 800 900 1000"
// MRPT_VOTES="1 2 3 4 5 8 10 12 15 20 25 30 40 50 60 70 80 90 100"


template < typename Scalar = float, typename Metric = openMVG::matching::L2<Scalar> >
class ArrayMatcher_mrpt : public openMVG::matching::ArrayMatcher<Scalar, Metric>
{
public:
	using DistanceType = typename Metric::ResultType;

	static int n_trees, depth, votes;
	static float sparsity;
	static float targetRecall_;
	static bool autotune_;
	static int treesMax_;
	std::mutex mutex_;

	ArrayMatcher_mrpt()
		: datasetMap_(nullptr)
	{
	}

	virtual ~ArrayMatcher_mrpt()
	{
		delete datasetMap_;
	}

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

	datasetMap_ = new Eigen::Map<const Eigen::MatrixXf>(const_cast<Scalar *>(dataset_), dimension_, nbRows_);

	// Make sure depth is not too big such that 2^depth < nbRows
	int depth_l = 0;
	depth_l = std::max(2, std::min(depth, static_cast<int>(std::floor(std::log2(nbRows))) - 1));
	//index_ = std::unique_ptr<Mrpt>(new Mrpt(datasetMap_));
	index_ = std::unique_ptr<Mrpt>(new Mrpt(dataset_, dimension_, nbRows_));

	if(autotune_)
	{
		// Autotuned index generation
		index_->grow_autotune(targetRecall_, 2, treesMax_);
		/*
		const int numberOfTestQueries = 100;

		Eigen::MatrixXf testQueries;
		testQueries.setZero(dimension_, numberOfTestQueries);
		std::default_random_engine re;
		std::uniform_int_distribution<> dis(0, nbRows_ - 1);
		for(int i = 0; i < numberOfTestQueries; i++)
		{
			int index = dis(re);
			testQueries.col(i) = datasetMap_->col(index);
		}
		index_->grow(targetRecall_, testQueries, 2);

		//std::vector<Mrpt_Parameters> optimalParams = index_->optimal_parameters();*/
		Mrpt_Parameters params = index_->parameters();

		std::ostringstream ostr;
		ostr << "MRPT params n_trees: " << params.n_trees
			<< " depth: " << params.depth
			<< " votes: " << params.votes
			<< " estimated_qtime: " << params.estimated_qtime << std::endl;

#if defined(_MSC_VER)
		OutputDebugStringA(ostr.str().c_str());
#endif
	}
	else
	{
		// Normal index generation
		index_->grow(n_trees, depth_l);
	}

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
			int k = 1;
			std::vector<int> result(k);
			const Eigen::Map<Eigen::VectorXf> queryMap(const_cast<Scalar *>(query), dimension_);
			index_->query(queryMap, k, votes, &result[0], distance);

			indice[0] = result[0];
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
		if (!datasetMap_ ||
			NN > datasetMap_->rows() ||
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
				std::vector<int> knn(NN);
				std::vector<float> dists(NN);

				const Eigen::Map<Eigen::VectorXf> queryMap(const_cast<Scalar *>(query)+i*dimension_, dimension_);

				if(autotune_)
					index_->query(queryMap, &knn[0], &(dists[0]));
				else
					index_->query(queryMap, NN, votes, &knn[0], &(dists[0]));

				bool isValid = true;
				for(int j = 0; j < NN; j++)
					if(knn[j] < 0)
						isValid = false;
						//throw std::runtime_error("Query returned -1");

				if(!isValid && !autotune_)
				{
					// Try again
					index_->query(queryMap, NN, votes-1, &knn[0], &(dists[0]));
					isValid = true;
					for(int j = 0; j < NN; j++)
						if(knn[j] < 0)
							isValid = false;
				}

				if(isValid)
				{
					// Save the resulting found indices
					for(int j = 0; j < NN; j++)
					{
						pvec_indices->emplace_back(i, knn[j]);
						pvec_distances->emplace_back(dists[j]);
					}
				}
			}
			return true;
		}
		else
		{
			return false;
		} return true;
	}

private:

	std::unique_ptr<Mrpt> index_;
	int nbRows_, dimension_;
	const Scalar *dataset_;
	const Eigen::Map<const Eigen::MatrixXf> *datasetMap_;
};

#endif // !MATCHER_KGRAPH_H
