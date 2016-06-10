/**
 * \file landmark_polyline_2D.cpp
 *
 *  Created on: May 26, 2016
 *      \author: jvallve
 */

#include "feature_polyline_2D.h"
#include "landmark_polyline_2D.h"
#include "local_parametrization_polyline_extreme.h"
#include "state_block.h"

namespace wolf
{

LandmarkPolyline2D::LandmarkPolyline2D(FeaturePolyline2D* _polyline_ptr) :
        LandmarkPolyline2D(_polyline_ptr->getPoints(), _polyline_ptr->isFirstDefined(), _polyline_ptr->isLastDefined())
{
}

LandmarkPolyline2D::LandmarkPolyline2D(const Eigen::MatrixXs& _points, const bool _first_extreme, const bool _last_extreme) :
        LandmarkBase(LANDMARK_POLYLINE_2D, nullptr, nullptr), first_defined_(_first_extreme), last_defined_(_last_extreme)
{
    for (auto i = 0; i < _points.cols(); i++)
        point_state_ptr_vector_.push_back(new StateBlock(_points.col(i).head<2>()));

    if (!first_defined_)
        point_state_ptr_vector_.front()->setLocalParametrizationPtr(new LocalParametrizationPolylineExtreme(point_state_ptr_vector_[1]));
    if (!last_defined_)
        point_state_ptr_vector_.back()->setLocalParametrizationPtr(new LocalParametrizationPolylineExtreme(point_state_ptr_vector_[point_state_ptr_vector_.size() - 2]));
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
    //std::cout << "LandmarkPolyline2D::setFirst" << std::endl;
    assert(_point.size() >= 2 && "LandmarkPolyline2D::setFirstExtreme: bad point size");
    point_state_ptr_vector_.front()->setVector(_point.head(2));
    first_defined_ = _defined;
}

void LandmarkPolyline2D::setLast(const Eigen::VectorXs& _point, bool _defined)
{
    //std::cout << "LandmarkPolyline2D::setLast" << std::endl;
    assert(_point.size() >= 2 && "LandmarkPolyline2D::setLastExtreme: bad point size");
    point_state_ptr_vector_.back()->setVector(_point.head(2));
    last_defined_ = _defined;
}

const Eigen::VectorXs& LandmarkPolyline2D::getPointVector(unsigned int _i) const
{
    return point_state_ptr_vector_[_i]->getVector();
}

void LandmarkPolyline2D::addPoint(const Eigen::VectorXs& _point, const bool& _defined, const bool& _back)
{
    assert(_point.size() >= 2 && "bad point size");

    // define previous extreme
    defineExtreme(_back);

    // add new extreme
    if (_back)
    {
        point_state_ptr_vector_.push_back(new StateBlock(_point.head<2>(),
                                                         (!_defined ?
                                                                 new LocalParametrizationPolylineExtreme(point_state_ptr_vector_.back()) :
                                                                 nullptr)));
        last_defined_ = _defined;
    }
    else
    {
        point_state_ptr_vector_.push_front(new StateBlock(_point.head<2>(),
                                                          (!_defined ?
                                                                  new LocalParametrizationPolylineExtreme(point_state_ptr_vector_.front()) :
                                                                  nullptr)));
        first_defined_ = _defined;
    }
}

void LandmarkPolyline2D::addPoints(const Eigen::MatrixXs& _points, const unsigned int& _idx, const bool& _defined,
                                   const bool& _back)
{
    //std::cout << "LandmarkPolyline2D::addPoints from/to: " << _idx << std::endl << _points << std::endl;
    assert(_points.rows() >= 2 && "bad points size");
    assert(_idx < _points.cols() && "bad index!");

    // define previous extreme
    defineExtreme(_back);

    // add new extreme points
    if (_back)
    {
        for (int i = _idx; i < _points.cols(); i++)
            point_state_ptr_vector_.push_back(new StateBlock(_points.block(0,i,2,1),
                                                             (i == _points.cols()-1 && !_defined ?
                                                                     new LocalParametrizationPolylineExtreme(point_state_ptr_vector_.back()) :
                                                                     nullptr)));
        last_defined_ = _defined;
    }
    else
    {
        for (int i = _idx; i >= 0; i--)
            point_state_ptr_vector_.push_front(new StateBlock(_points.block(0,i,2,1),
                                               (i == 0 && !_defined ?
                                                       new LocalParametrizationPolylineExtreme(point_state_ptr_vector_.front()) :
                                                       nullptr)));
        first_defined_ = _defined;
    }
}

void LandmarkPolyline2D::defineExtreme(const bool _back)
{
    StateBlock* state = (_back ? point_state_ptr_vector_.back() : point_state_ptr_vector_.front());
    // remove and add state block without local parameterization
    getProblem()->removeStateBlockPtr(state);
    point_state_ptr_vector_.front()->removeLocalParametrization();
    getProblem()->addStateBlockPtr(state);
    // remove and add all constraints to the point
    for (auto ctr_ptr : constrained_by_list_)
        for (auto st_ptr : ctr_ptr->getStatePtrVector())
            if (st_ptr == state)
            {
                getProblem()->removeConstraintPtr(ctr_ptr);
                getProblem()->addConstraintPtr(ctr_ptr);
            }
    // update boolean
    if (_back)
        last_defined_ = true;
    else
        first_defined_ = true;
}

} /* namespace wolf */
