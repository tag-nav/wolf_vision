/**
 * \file landmark_point_2D.cpp
 *
 *  Created on: Dec 4, 2017
 *      \author: jsola
 */

#include "landmark_point_2D.h"

namespace wolf
{

LandmarkPoint2D::LandmarkPoint2D(int _id, const Eigen::Vector2s& _xy) :
        LandmarkBase("POINT 2D", std::make_shared<StateBlock>(_xy))
{
    setId(_id);
}

LandmarkPoint2D::~LandmarkPoint2D()
{
    //
}

} /* namespace wolf */
