/**
 * \file landmark_polyline_2D.cpp
 *
 *  Created on: May 26, 2016
 *      \author: jvallve
 */

#include "feature_polyline_2D.h"
#include "landmark_polyline_2D.h"
#include "local_parametrization_polyline_extreme.h"
#include "constraint_point_2D.h"
#include "constraint_point_to_line_2D.h"
#include "state_block.h"
#include "factory.h"
#include "yaml/yaml_conversion.h"

namespace wolf
{

LandmarkPolyline2D::LandmarkPolyline2D(const Eigen::MatrixXs& _points, const bool _first_extreme, const bool _last_extreme, unsigned int _first_id) :
        LandmarkBase(LANDMARK_POLYLINE_2D, new StateBlock(Eigen::Vector2s::Zero(), true), new StateBlock(Eigen::Vector1s::Zero(), true)), first_id_(0), first_defined_(_first_extreme), last_defined_(_last_extreme), closed_(false)
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
    assert(!closed_ && "adding point to a closed polyline!");

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
    assert(!closed_ && "adding points to a closed polyline!");

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

    //std::cout << "Defining extreme --> Removing and adding state blocks and constraints" << std::endl;

    // remove and add state block without local parameterization
    if (getProblem() != nullptr)
    	getProblem()->removeStateBlockPtr(state);

    state->removeLocalParametrization();

    if (getProblem() != nullptr)
    	getProblem()->addStateBlockPtr(state);

    // remove and add all constraints to the point
    for (auto ctr_ptr : *getConstrainedByListPtr())
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

void LandmarkPolyline2D::setClosed(bool _merge_extremes)
{
    assert(((_merge_extremes && (first_defined_ || last_defined_)) || (first_defined_ && last_defined_)) && "closing a polyline without merging extremes with a non-defined extreme or merging without any defined extreme");

    if (_merge_extremes)
    {
        // take a defined extreme as remaining
        StateBlock* remove_state = (first_defined_ ? point_state_ptr_vector_.back() : point_state_ptr_vector_.front());
        StateBlock* remain_state = (first_defined_ ? point_state_ptr_vector_.front() : point_state_ptr_vector_.back());

        // new point id for new constraints
        int remove_id = (first_defined_ ? getLastId() : getFirstId());
        int remain_id = (first_defined_ ? getFirstId() : getLastId());

        // Change constraints from remove_state to remain_state
        ConstraintBaseList old_constraints_list = *getConstrainedByListPtr();
        ConstraintBase* new_ctr_ptr = nullptr;
        for (auto ctr_ptr : old_constraints_list)
        {
            if (ctr_ptr->getType() == CTR_POINT_2D)
            {
                ConstraintPoint2D* ctr_point_ptr = (ConstraintPoint2D*)ctr_ptr;

                // If landmark point constrained -> new constraint
                if (ctr_point_ptr->getLandmarkPointId() == remove_id)
                    new_ctr_ptr = new ConstraintPoint2D((FeaturePolyline2D*)(ctr_ptr->getFeaturePtr()),
                                                        this,
                                                        ctr_point_ptr->getFeaturePointId(),
                                                        remain_id,
                                                        ctr_point_ptr->getApplyLossFunction(),
                                                        ctr_point_ptr->getStatus());
            }
            else if  (ctr_ptr->getType() == CTR_POINT_TO_LINE_2D)
            {
                ConstraintPointToLine2D* ctr_point_ptr = (ConstraintPointToLine2D*)ctr_ptr;

                // If landmark point constrained -> new constraint
                if (ctr_point_ptr->getLandmarkPointId() == remove_id)
                    new_ctr_ptr = new ConstraintPointToLine2D((FeaturePolyline2D*)(ctr_ptr->getFeaturePtr()),
                                                                                      this,
                                                                                      ctr_point_ptr->getFeaturePointId(),
                                                                                      remain_id,
                                                                                      ctr_point_ptr->getLandmarkPointAuxId(),
                                                                                      ctr_point_ptr->getApplyLossFunction(),
                                                                                      ctr_point_ptr->getStatus());
                // If landmark point is aux point -> new constraint
                else if (ctr_point_ptr->getLandmarkPointAuxId() == remove_id)
                    new_ctr_ptr = new ConstraintPointToLine2D((FeaturePolyline2D*)(ctr_ptr->getFeaturePtr()),
                                                                                      this,
                                                                                      ctr_point_ptr->getFeaturePointId(),
                                                                                      ctr_point_ptr->getLandmarkPointId(),
                                                                                      remain_id,
                                                                                      ctr_point_ptr->getApplyLossFunction(),
                                                                                      ctr_point_ptr->getStatus());
            }
            else
                throw std::runtime_error ("polyline constraint of unknown type");

            // If new constraint
            if (new_ctr_ptr != nullptr)
            {
                // add new constraint
                ctr_ptr->getFeaturePtr()->addConstraint(new_ctr_ptr);

                // delete constraint
                ctr_ptr->destruct();

                new_ctr_ptr = nullptr;
            }
        }

        // Remove remove_state
        if (getProblem() != nullptr)
            getProblem()->removeStateBlockPtr(remove_state);
        delete remove_state;

        if (first_defined_)
            point_state_ptr_vector_.pop_back();
        else
        {
            point_state_ptr_vector_.pop_front();
            first_id_++;
        }

        // all defined
        last_defined_ = true;
        first_defined_ = true;
    }

    closed_ = true;
}

void LandmarkPolyline2D::registerNewStateBlocks()
{
    LandmarkBase::registerNewStateBlocks();
	if (getProblem() != nullptr)
		for (auto state : point_state_ptr_vector_)
			getProblem()->addStateBlockPtr(state);
}

LandmarkBase* LandmarkPolyline2D::create(const YAML::Node& _lmk_node)
{
    // Parse YAML node with lmk info and data
    unsigned int id         = _lmk_node["id"].as<unsigned int>();
    int first_id            = _lmk_node["first_id"].as<int>();
    bool first_defined      = _lmk_node["first_defined"].as<bool>();
    bool last_defined       = _lmk_node["last_defined"].as<bool>();
    unsigned int npoints    = _lmk_node["points"].size();
    Eigen::MatrixXs points(2,npoints);
    for (unsigned int i = 0; i < npoints; i++)
    {
        points.col(i) = _lmk_node["points"][i].as<Eigen::Vector2s>();
    }

    //std::cout << "Points in lmk: " << id << ":\n" << points << std::endl;

    // Create a new landmark
    LandmarkBase* lmk_ptr = new LandmarkPolyline2D(points, first_defined, last_defined, first_id);

    lmk_ptr->setId(id);

    return lmk_ptr;

}

YAML::Node LandmarkPolyline2D::saveToYaml() const
{
    YAML::Node n;
    n["id"]             = landmark_id_;
    n["type"]           = "POLYLINE 2D";
    n["first_id"]       = first_id_;
    n["first_defined"]  = first_defined_;
    n["last_defined"]   = last_defined_;

    int npoints = point_state_ptr_vector_.size();

    for (int i = 0; i < npoints; i++)
    {
        n["points"].push_back(point_state_ptr_vector_[i]->getVector());
    }

    return n;
}

// Register landmark creator
namespace
{
const bool registered_lmk_polyline_2D = LandmarkFactory::get().registerCreator("POLYLINE 2D", LandmarkPolyline2D::create);
}

} /* namespace wolf */
