/**
 * \file landmark_point_2D.h
 *
 *  Created on: Dec 4, 2017
 *      \author: jsola
 */

#ifndef HELLO_WOLF_LANDMARK_POINT_2D_H_
#define HELLO_WOLF_LANDMARK_POINT_2D_H_

#include "landmark_base.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(LandmarkPoint2D);

class LandmarkPoint2D : public LandmarkBase
{
    public:
        LandmarkPoint2D(int _id, const Eigen::Vector2s& _xy);
        virtual ~LandmarkPoint2D();
};

} /* namespace wolf */

#endif /* HELLO_WOLF_LANDMARK_POINT_2D_H_ */
