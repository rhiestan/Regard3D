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

#ifndef MATCHER_KGRAPH_H
#define MATCHER_KGRAPH_H

#if defined(R3D_HAVE_OPENMP)
#	include <omp.h>
#endif

#include "openMVG/matching/matching_interface.hpp"
#include "openMVG/matching/metric.hpp"

#include <kgraph.h>


template < typename Scalar = float, typename Metric = openMVG::matching::L2<Scalar> >
class ArrayMatcher_kgraph : public openMVG::matching::ArrayMatcher<Scalar, Metric>
{
public:
	using DistanceType = typename Metric::ResultType;

	static kgraph::KGraph::IndexParams iparams;
	static kgraph::KGraph::SearchParams sparams;

	class R3D_IndexOracle: public kgraph::IndexOracle
	{
	public:
		R3D_IndexOracle(const Scalar *dataset, int nbRows, int dimension)
			: kgraph::IndexOracle(),
			dataset_(dataset), nbRows_(nbRows), dimension_(dimension)
		{
		}

		// returns size N of dataset
		virtual unsigned size() const
		{
			return static_cast<unsigned>(nbRows_);
		}

		// computes similarity of object 0 <= i and j < N
		virtual float operator () (unsigned i, unsigned j) const
		{
			Metric metric;

			const Scalar *pDataI = dataset_ + dimension_*i;
			const Scalar *pDataJ = dataset_ + dimension_*j;

			return metric(pDataI, pDataJ, dimension_);
		}

		const Scalar *dataset_;
		int nbRows_, dimension_;
	};

	class R3D_SearchOracle: public kgraph::SearchOracle
	{
	public:
		R3D_SearchOracle(const Scalar *dataset, const Scalar *query, int nbRows, int dimension, int index)
			: kgraph::SearchOracle(),
			dataset_(dataset), query_(query), nbRows_(nbRows), dimension_(dimension), index_(index)
		{
		}

		// returns size N of dataset
		virtual unsigned size() const
		{
			return static_cast<unsigned>(nbRows_);
		}

		/// Computes similarity of query and object 0 <= i < N.
		virtual float operator () (unsigned i) const
		{
			Metric metric;

			const Scalar *pDataI = dataset_ + dimension_*i;
			const Scalar *pDataJ = query_ + dimension_*index_;

			return metric(pDataI, pDataJ, dimension_);
		}

		const Scalar *dataset_, *query_;
		int nbRows_, dimension_, index_;
	};

	ArrayMatcher_kgraph() = default;

	virtual ~ArrayMatcher_kgraph() = default;

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
		//memMapping.reset(nullptr);
		return false;
	}
	//memMapping.reset(new Eigen::Map<BaseMat>( (Scalar*)dataset, nbRows, dimension));
	dimension_ = dimension;
	nbRows_ = nbRows;
	dataset_ = dataset;

	index_ = std::unique_ptr<kgraph::KGraph>(kgraph::KGraph::create());

	R3D_IndexOracle oracle(dataset, nbRows, dimension);

/*	kgraph::KGraph::IndexParams params;
	params.reverse = -1;
	params.recall = 0.99f;
	params.K = 25;
	params.L = 100;
	params.prune = 1;*/

	kgraph::KGraph::IndexInfo info;

	index_->build(oracle, iparams, &info);

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
			R3D_SearchOracle soracle(dataset_, query, nbRows_, dimension_, 0);
			std::array<unsigned int, 1> knn;
			std::array<float, 1> dists;
			kgraph::KGraph::SearchParams sparams = this->sparams;
			sparams.K = 1;
//			sparams.P = 1;

			kgraph::KGraph::SearchInfo info;
			index_->search(soracle, sparams, &knn[0], &dists[0], &info);

			indice[0] = knn[0];
			distance[0] = dists[0];
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

		if(index_.get() != nullptr)
		{
			pvec_indices->reserve(nbQuery * NN);
			pvec_distances->reserve(nbQuery * NN);

			// Maybe parallel_for ?
			for(int i = 0; i < nbQuery; i++)
			{
				R3D_SearchOracle soracle(dataset_, query, nbRows_, dimension_, i);
				std::vector<unsigned int> knn(NN);
				std::vector<float> dists(NN);
				kgraph::KGraph::SearchParams sparams = this->sparams;
				sparams.K = NN;
				//sparams.P = 100;
				kgraph::KGraph::SearchInfo info;

				index_->search(soracle, sparams, &knn[0], &dists[0], &info);

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

	std::unique_ptr<kgraph::KGraph> index_;
	int nbRows_, dimension_;
	const Scalar *dataset_;

	//using BaseMat = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
	/// Use a memory mapping in order to avoid memory re-allocation
	//std::unique_ptr< Eigen::Map<BaseMat> > memMapping;
};

#endif // !MATCHER_KGRAPH_H
