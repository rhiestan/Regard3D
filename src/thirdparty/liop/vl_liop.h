/** @file liop.h
 ** @brief Local Intensity Order Pattern (LIOP) descriptor (@ref liop)
 ** @author Hana Sarbortova
 ** @author Andrea Vedaldi
 ** @see @ref liop
 **/

/*
Copyright (C) 2013 Hana Sarbortova and Andrea Vedaldi.
All rights reserved.

This file is part of the VLFeat library and is made available under
the terms of the BSD license (see the COPYING file).
*/

#ifndef R3D_VL_LIOP_H
#define R3D_VL_LIOP_H

/* #include "generic.h" */
#include "vl_host.h"


/** @brief LIOP descriptor extractor object */
typedef struct _VlLiopDesc
{
  vl_int numNeighbours; /**< Number of neighbours. */
  vl_int numSpatialBins; /**< Number of bins. */
  float intensityThreshold; /**< Weight threshold. */
  vl_size dimension; /**< LIOP descriptor size. */

  /* Pixels in the circular patch */
  vl_size patchSideLength ;
  vl_size patchSize ; /* only circular neighbourhood */
  vl_uindex * patchPixels ;
  float * patchIntensities ;
  vl_uindex * patchPermutation ;

  /* Neighbourhoods of each pixel (samples in a circle) */
  float neighRadius; /**< Point to neighbour radius (distance). */

  float * neighIntensities ;
  vl_uindex * neighPermutation ;
  double * neighSamplesX ;
  double * neighSamplesY ;

} VlLiopDesc ;

/** @name Construct and destroy
 ** @{ */
VL_EXPORT
VlLiopDesc * r3d_vl_liopdesc_new (vl_int numNeighbours,
                              vl_int numSpatialBins,
                              float radius,
                              vl_size sideLength) ;

VL_EXPORT
VlLiopDesc * r3d_vl_liopdesc_new_basic (vl_size sideLength) ;

VL_EXPORT
void r3d_vl_liopdesc_delete (VlLiopDesc * self) ;
/** @} */

/**  @name Get data and parameters
 **  @{ */
VL_EXPORT vl_size r3d_vl_liopdesc_get_dimension (VlLiopDesc const * self) ;
VL_EXPORT vl_size r3d_vl_liopdesc_get_num_neighbours (VlLiopDesc const * self) ;
VL_EXPORT float r3d_vl_liopdesc_get_intensity_threshold (VlLiopDesc const * self) ;
VL_EXPORT vl_size r3d_vl_liopdesc_get_num_spatial_bins (VlLiopDesc const * self) ;
VL_EXPORT double r3d_vl_liopdesc_get_neighbourhood_radius (VlLiopDesc const * self) ;
VL_EXPORT void r3d_vl_liopdesc_set_intensity_threshold (VlLiopDesc * self, float x) ;
/** @} */

/**  @name Compute LIOP descriptor
 **  @{ */
VL_EXPORT
void r3d_vl_liopdesc_process (VlLiopDesc * liop, float * desc, float const * patch) ;
/** @} */

/* VL_LIOP_H */
#endif
