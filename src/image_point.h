/**
 * \file image_point.h
 *
 *  Created on: 15/08/2014
 *     \author: jsola
 */

#ifndef IMAGE_POINT_H_
#define IMAGE_POINT_H_

#include "feature_base.h"

/**
 * \brief An image point, a 2D point in pixel units.
 *
 * An image point, a 2D point in pixel units.
 */
class ImagePoint : public FeatureBase
{
    public:
        /// Dimension of the measurement space
        static const unsigned int DIM_MEASUREMENT_ = 2;

    public:

        ImagePoint(const CaptureShPtr& _sc_ptr, const Eigen::Vector2s& _meas, const NodeLocation _loc = MID);

        virtual ~ImagePoint();
};

////////////////////////////////
// IMPLEMENTATION
////////////////////////////////



inline ImagePoint::ImagePoint(const CaptureShPtr& _sc_ptr, const Eigen::Vector2s& _meas, const NodeLocation _loc) :
        FeatureBase(_sc_ptr, DIM_MEASUREMENT_, _loc) //
{
    measurement_ = _meas;
}

inline ImagePoint::~ImagePoint()
{
    //
}

#endif /* IMAGE_POINT_H_ */
