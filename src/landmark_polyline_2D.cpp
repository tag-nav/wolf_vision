/**
 * \file landmark_polyline_2D.cpp
 *
 *  Created on: May 26, 2016
 *      \author: jvallve
 */

#include "feature_polyline_2D.h"
#include "landmark_polyline_2D.h"
#include "state_block.h"

namespace wolf
{

LandmarkPolyline2D::LandmarkPolyline2D(FeaturePolyline2D* _polyline_ptr) :
        LandmarkBase(LANDMARK_POLYLINE_2D, nullptr, nullptr), first_extreme_(_polyline_ptr->isFirstExtreme()), last_extreme_(
                _polyline_ptr->isLastExtreme())
{
    for (auto i = 0; i < _polyline_ptr->getPoints().size(); i++)
        point_state_ptr_vector_.push_back(new StateBlock(_polyline_ptr->getPoints().col(i)));
}

LandmarkPolyline2D::~LandmarkPolyline2D()
{
    while (!point_state_ptr_vector_.empty())
    {
        if (getProblem() != nullptr)
            getProblem()->removeStateBlockPtr(point_state_ptr_vector_.front());

        delete point_state_ptr_vector_.front();
        point_state_ptr_vector_.pop_front();
    }
}

void LandmarkPolyline2D::addPoint(const Eigen::VectorXs& _point, const bool& _extreme, const bool& _back)
{

    if (_back)
    {
        point_state_ptr_vector_.push_back(new StateBlock(_point.head<2>()));
        last_extreme_ = _extreme;
    }
    else
    {
        point_state_ptr_vector_.push_front(new StateBlock(_point.head<2>()));
        first_extreme_ = _extreme;
    }
}

void LandmarkPolyline2D::addPoints(const Eigen::MatrixXs& _points, const int& _idx, const bool& _extreme,
                                   const bool& _back)
{

    if (_back)
    {
        for (auto i = _idx; _idx < _points.cols(); i++)
            point_state_ptr_vector_.push_back(new StateBlock(_points.col(i).head<2>()));
        last_extreme_ = _extreme;
    }
    else
    {
        for (auto i = _idx; _idx > 0; i--)
            point_state_ptr_vector_.push_front(new StateBlock(_points.col(i).head<2>()));
        first_extreme_ = _extreme;
    }
}

} /* namespace wolf */
