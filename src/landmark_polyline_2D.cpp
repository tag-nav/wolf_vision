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
        LandmarkBase(LANDMARK_POLYLINE_2D, nullptr, nullptr), first_defined_(_polyline_ptr->isFirstDefined()), last_defined_(
                _polyline_ptr->isLastDefined())
{
    for (auto i = 0; i < _polyline_ptr->getPoints().size(); i++)
        point_state_ptr_vector_.push_back(new StateBlock(_polyline_ptr->getPoints().col(i).head<2>()));
}

LandmarkPolyline2D::LandmarkPolyline2D(const Eigen::MatrixXs& _points, const bool _first_extreme, const bool _last_extreme) :
        LandmarkBase(LANDMARK_POLYLINE_2D, nullptr, nullptr), first_defined_(_first_extreme), last_defined_(_last_extreme)
{
    for (auto i = 0; i < _points.cols(); i++)
        point_state_ptr_vector_.push_back(new StateBlock(_points.col(i).head<2>()));
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

void LandmarkPolyline2D::setFirst(const Eigen::VectorXs& _point, bool _defined)
{
    std::cout << "LandmarkPolyline2D::setFirst" << std::endl;
    assert(_point.size() >= 2 && "LandmarkPolyline2D::setFirstExtreme: bad point size");
    point_state_ptr_vector_.front()->setVector(_point.head(2));
    first_defined_ = _defined;
}

void LandmarkPolyline2D::setLast(const Eigen::VectorXs& _point, bool _defined)
{
    std::cout << "LandmarkPolyline2D::setLast" << std::endl;
    assert(_point.size() >= 2 && "LandmarkPolyline2D::setLastExtreme: bad point size");
    point_state_ptr_vector_.back()->setVector(_point.head(2));
    last_defined_ = _defined;
}

void LandmarkPolyline2D::addPoint(const Eigen::VectorXs& _point, const bool& _defined, const bool& _back)
{
    assert(_point.size() >= 2 && "bad point size");
    if (_back)
    {
        point_state_ptr_vector_.push_back(new StateBlock(_point.head<2>()));
        last_defined_ = _defined;
    }
    else
    {
        point_state_ptr_vector_.push_front(new StateBlock(_point.head<2>()));
        first_defined_ = _defined;
    }
}

void LandmarkPolyline2D::addPoints(const Eigen::MatrixXs& _points, const int& _idx, const bool& _defined,
                                   const bool& _back)
{
    assert(_points.cols() >= 2 && "bad points size");

    if (_back)
    {
        for (auto i = _idx; _idx < _points.cols(); i++)
            point_state_ptr_vector_.push_back(new StateBlock(_points.col(i).head<2>()));
        last_defined_ = _defined;
    }
    else
    {
        for (auto i = _idx; _idx > 0; i--)
            point_state_ptr_vector_.push_front(new StateBlock(_points.col(i).head<2>()));
        first_defined_ = _defined;
    }
}

} /* namespace wolf */
