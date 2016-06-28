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

LandmarkPolyline2D::LandmarkPolyline2D(const Eigen::MatrixXs& _points, const bool _first_extreme, const bool _last_extreme) :
        LandmarkBase(LANDMARK_POLYLINE_2D, nullptr, nullptr), first_id_(0), first_defined_(_first_extreme), last_defined_(_last_extreme)
{
    //std::cout << "LandmarkPolyline2D::LandmarkPolyline2D" << std::endl;
	assert(_points.cols() >= 2 && "LandmarkPolyline2D::LandmarkPolyline2D: 2 points at least needed.");
    for (auto i = 0; i < _points.cols(); i++)
    	point_state_ptr_vector_.push_back(new StateBlock(_points.col(i).head<2>()));

    if (!first_defined_)
        point_state_ptr_vector_.front()->setLocalParametrizationPtr(new LocalParametrizationPolylineExtreme(point_state_ptr_vector_[1]));
    if (!last_defined_)
        point_state_ptr_vector_.back()->setLocalParametrizationPtr(new LocalParametrizationPolylineExtreme(point_state_ptr_vector_[point_state_ptr_vector_.size() - 2]));

    assert(point_state_ptr_vector_.front()->hasLocalParametrization() ? !first_defined_ : first_defined_);
    assert(point_state_ptr_vector_.back()->hasLocalParametrization() ? !last_defined_ : last_defined_);
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
    //std::cout << "LandmarkPolyline2D::setFirst. Defined " << _defined << std::endl;
    assert(_point.size() >= 2 && "LandmarkPolyline2D::setFirstExtreme: bad point size");
    assert(!(!_defined && first_defined_) && "setting a defined extreme with a not defined point");
    point_state_ptr_vector_.front()->setVector(_point.head(2));
    if (!first_defined_ && _defined)
    	defineExtreme(false);
}

void LandmarkPolyline2D::setLast(const Eigen::VectorXs& _point, bool _defined)
{
    //std::cout << "LandmarkPolyline2D::setLast. Defined " << _defined << std::endl;
    assert(_point.size() >= 2 && "LandmarkPolyline2D::setLastExtreme: bad point size");
    assert(!(!_defined && last_defined_) && "setting a defined extreme with a not defined point");
    point_state_ptr_vector_.back()->setVector(_point.head(2));
    if (!last_defined_ && _defined)
    	defineExtreme(true);
}

const Eigen::VectorXs& LandmarkPolyline2D::getPointVector(int _i) const
{
	//std::cout << "LandmarkPolyline2D::getPointVector: " << _i << std::endl;
	//std::cout << "First: " << first_id_ << " - size: " << point_state_ptr_vector_.size() << std::endl;
	assert(_i >= first_id_ && _i < first_id_+(int)(point_state_ptr_vector_.size()));
    return point_state_ptr_vector_[_i-first_id_]->getVector();
}

StateBlock* LandmarkPolyline2D::getPointStateBlockPtr(int _i)
{
	assert(_i >= first_id_ && _i < first_id_+(int)(point_state_ptr_vector_.size()));
	return point_state_ptr_vector_[_i-first_id_];
}

void LandmarkPolyline2D::addPoint(const Eigen::VectorXs& _point, const bool& _defined, const bool& _back)
{
	//std::cout << "LandmarkPolyline2D::addPoint. Defined " << _defined << std::endl;
    assert(_point.size() >= 2 && "bad point size");

    // define previous extreme if not defined
    if (_back ? !last_defined_ : !first_defined_)
    	defineExtreme(_back);

    // add new extreme
    if (_back)
    {
        point_state_ptr_vector_.push_back(new StateBlock(_point.head<2>(), false,
                                                         (!_defined ?
                                                                 new LocalParametrizationPolylineExtreme(point_state_ptr_vector_.back()) :
                                                                 nullptr)));
        if (getProblem() != nullptr)
        	getProblem()->addStateBlockPtr(point_state_ptr_vector_.back());
        last_defined_ = _defined;
		assert(point_state_ptr_vector_.back()->hasLocalParametrization() ? !last_defined_ : last_defined_);
    }
    else
    {
        point_state_ptr_vector_.push_front(new StateBlock(_point.head<2>(), false,
                                                          (!_defined ?
                                                                  new LocalParametrizationPolylineExtreme(point_state_ptr_vector_.front()) :
                                                                  nullptr)));
        if (getProblem() != nullptr)
        	getProblem()->addStateBlockPtr(point_state_ptr_vector_.front());
        first_defined_ = _defined;
        first_id_--;
		assert(point_state_ptr_vector_.front()->hasLocalParametrization() ? !first_defined_ : first_defined_);
    }
}

void LandmarkPolyline2D::addPoints(const Eigen::MatrixXs& _points, const unsigned int& _idx, const bool& _defined,
                                   const bool& _back)
{
    //std::cout << "LandmarkPolyline2D::addPoints from/to: " << _idx << " Defined " << _defined << std::endl;
    assert(_points.rows() >= 2 && "bad points size");
    assert(_idx < _points.cols() && "bad index!");

    // define previous extreme if not defined
    if (_back ? !last_defined_ : !first_defined_)
    	defineExtreme(_back);

    // add new extreme points
    if (_back)
    {
        for (int i = _idx; i < _points.cols(); i++)
        {
        	point_state_ptr_vector_.push_back(new StateBlock(_points.block(0,i,2,1),
        													 false,
        													 (i == _points.cols()-1 && !_defined ?
        															 new LocalParametrizationPolylineExtreme(point_state_ptr_vector_.back()) :
        															 nullptr)));
        	if (getProblem() != nullptr)
        		getProblem()->addStateBlockPtr(point_state_ptr_vector_.back());
        }
        last_defined_ = _defined;
		assert(point_state_ptr_vector_.back()->hasLocalParametrization() ? !last_defined_ : last_defined_);
    }
    else
    {
        for (int i = _idx; i >= 0; i--)
        {
        	point_state_ptr_vector_.push_front(new StateBlock(_points.block(0,i,2,1),
        													  false,
        													  (i == 0 && !_defined ?
        															  new LocalParametrizationPolylineExtreme(point_state_ptr_vector_.front()) :
        															  nullptr)));
        	if (getProblem() != nullptr)
        		getProblem()->addStateBlockPtr(point_state_ptr_vector_.front());
            first_id_--;
        }
		first_defined_ = _defined;
		assert(point_state_ptr_vector_.front()->hasLocalParametrization() ? !first_defined_ : first_defined_);
    }

    //std::cout << "final number of points: " << point_state_ptr_vector_.size() << std::endl;
}

void LandmarkPolyline2D::defineExtreme(const bool _back)
{
    StateBlock* state = (_back ? point_state_ptr_vector_.back() : point_state_ptr_vector_.front());
    assert((_back ? !last_defined_: !first_defined_) && "defining an already defined extreme");
    assert(state->hasLocalParametrization() && "not defined extreme without local parameterization");

    // remove and add state block without local parameterization
    if (getProblem() != nullptr)
    	getProblem()->removeStateBlockPtr(state);
    state->removeLocalParametrization();

    if (getProblem() != nullptr)
    	getProblem()->addStateBlockPtr(state);

    // remove and add all constraints to the point
    for (auto ctr_ptr : constrained_by_list_)
        for (auto st_ptr : ctr_ptr->getStatePtrVector())
            if (st_ptr == state && getProblem() != nullptr)
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

void LandmarkPolyline2D::registerNewStateBlocks()
{
	if (getProblem() != nullptr)
		for (auto state : point_state_ptr_vector_)
			getProblem()->addStateBlockPtr(state);
}

} /* namespace wolf */
