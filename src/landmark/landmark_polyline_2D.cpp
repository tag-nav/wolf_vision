/**
 * \file landmark_polyline_2D.cpp
 *
 *  Created on: May 26, 2016
 *      \author: jvallve
 */

#include "base/feature/feature_polyline_2D.h"
#include "base/landmark/landmark_polyline_2D.h"
#include "base/state_block/local_parametrization_polyline_extreme.h"
#include "base/factor/factor_point_2D.h"
#include "base/factor/factor_point_to_line_2D.h"
#include "base/state_block/state_block.h"
#include "base/common/factory.h"
#include "base/yaml/yaml_conversion.h"

namespace wolf
{

LandmarkPolyline2D::LandmarkPolyline2D(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, const Eigen::MatrixXs& _points, const bool _first_extreme, const bool _last_extreme, unsigned int _first_id, LandmarkClassification _class) :
        LandmarkBase("POLYLINE 2D", _p_ptr, _o_ptr), first_id_(_first_id), first_defined_(_first_extreme), last_defined_(_last_extreme), closed_(false), classification_(_class)
{
    //std::cout << "LandmarkPolyline2D::LandmarkPolyline2D" << std::endl;
    assert(_points.cols() >= 2 && "LandmarkPolyline2D::LandmarkPolyline2D: 2 points at least needed.");
    for (auto i = 0; i < _points.cols(); i++)
        point_state_ptr_vector_.push_back(std::make_shared<StateBlock>(_points.col(i).head<2>()));

    if (!first_defined_)
        point_state_ptr_vector_.front()->setLocalParametrization(std::make_shared<LocalParametrizationPolylineExtreme>(point_state_ptr_vector_[1]));
    if (!last_defined_)
        point_state_ptr_vector_.back()->setLocalParametrization(std::make_shared<LocalParametrizationPolylineExtreme>(point_state_ptr_vector_[point_state_ptr_vector_.size() - 2]));

    assert(point_state_ptr_vector_.front()->hasLocalParametrization() ? !first_defined_ : first_defined_);
    assert(point_state_ptr_vector_.back()->hasLocalParametrization() ? !last_defined_ : last_defined_);
}

LandmarkPolyline2D::~LandmarkPolyline2D()
{
    removeStateBlocks();
}

void LandmarkPolyline2D::setFirst(const Eigen::VectorXs& _point, bool _defined)
{
    //std::cout << "LandmarkPolyline2D::setFirst. Defined " << _defined << std::endl;
    assert(_point.size() >= 2 && "LandmarkPolyline2D::setFirstExtreme: bad point size");
    assert(!(!_defined && first_defined_) && "setting a defined extreme with a not defined point");
    point_state_ptr_vector_.front()->setState(_point.head(2));
    if (!first_defined_ && _defined)
    	defineExtreme(false);
}

void LandmarkPolyline2D::setLast(const Eigen::VectorXs& _point, bool _defined)
{
    //std::cout << "LandmarkPolyline2D::setLast. Defined " << _defined << std::endl;
    assert(_point.size() >= 2 && "LandmarkPolyline2D::setLastExtreme: bad point size");
    assert(!(!_defined && last_defined_) && "setting a defined extreme with a not defined point");
    point_state_ptr_vector_.back()->setState(_point.head(2));
    if (!last_defined_ && _defined)
    	defineExtreme(true);
}

const Eigen::VectorXs LandmarkPolyline2D::getPointVector(int _i) const
{
	//std::cout << "LandmarkPolyline2D::getPointVector: " << _i << std::endl;
	//std::cout << "First: " << first_id_ << " - size: " << point_state_ptr_vector_.size() << std::endl;
	assert(_i >= first_id_ && _i < first_id_+(int)(point_state_ptr_vector_.size()));
    return point_state_ptr_vector_[_i-first_id_]->getState();
}

StateBlockPtr LandmarkPolyline2D::getPointStateBlock(int _i)
{
	assert(_i-first_id_ >= 0 && _i-first_id_ <= (int)(point_state_ptr_vector_.size()) && "out of range!");
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
        point_state_ptr_vector_.push_back(std::make_shared<StateBlock>(_point.head<2>(), false,
                                                         (!_defined ?
                                                                 std::make_shared<LocalParametrizationPolylineExtreme>(point_state_ptr_vector_.back()) :
                                                                 nullptr)));
        if (getProblem() != nullptr)
        	getProblem()->addStateBlock(point_state_ptr_vector_.back());
        last_defined_ = _defined;
		assert(point_state_ptr_vector_.back()->hasLocalParametrization() ? !last_defined_ : last_defined_);
    }
    else
    {
        point_state_ptr_vector_.push_front(std::make_shared<StateBlock>(_point.head<2>(), false,
                                                          (!_defined ?
                                                                  std::make_shared<LocalParametrizationPolylineExtreme>(point_state_ptr_vector_.front()) :
                                                                  nullptr)));
        if (getProblem() != nullptr)
        	getProblem()->addStateBlock(point_state_ptr_vector_.front());
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
        	point_state_ptr_vector_.push_back(std::make_shared<StateBlock>(_points.block(0,i,2,1),
        													 false,
        													 (i == _points.cols()-1 && !_defined ?
        															 std::make_shared<LocalParametrizationPolylineExtreme>(point_state_ptr_vector_.back()) :
        															 nullptr)));
        	if (getProblem() != nullptr)
        		getProblem()->addStateBlock(point_state_ptr_vector_.back());
        }
        last_defined_ = _defined;
		assert(point_state_ptr_vector_.back()->hasLocalParametrization() ? !last_defined_ : last_defined_);
    }
    else
    {
        for (int i = _idx; i >= 0; i--)
        {
        	point_state_ptr_vector_.push_front(std::make_shared<StateBlock>(_points.block(0,i,2,1),
        													  false,
        													  (i == 0 && !_defined ?
        															  std::make_shared<LocalParametrizationPolylineExtreme>(point_state_ptr_vector_.front()) :
        															  nullptr)));
        	if (getProblem() != nullptr)
        		getProblem()->addStateBlock(point_state_ptr_vector_.front());
            first_id_--;
        }
		first_defined_ = _defined;
		assert(point_state_ptr_vector_.front()->hasLocalParametrization() ? !first_defined_ : first_defined_);
    }

    //std::cout << "final number of points: " << point_state_ptr_vector_.size() << std::endl;
}

void LandmarkPolyline2D::defineExtreme(const bool _back)
{
    StateBlockPtr state = (_back ? point_state_ptr_vector_.back() : point_state_ptr_vector_.front());
    assert((_back ? !last_defined_: !first_defined_) && "defining an already defined extreme");
    assert(state->hasLocalParametrization() && "not defined extreme without local parameterization");

    //std::cout << "Defining extreme --> Removing and adding state blocks and factors" << std::endl;

    // remove and add state block without local parameterization
    if (getProblem() != nullptr)
    	getProblem()->removeStateBlock(state);

    state->removeLocalParametrization();

    if (getProblem() != nullptr)
    	getProblem()->addStateBlock(state);

    // remove and add all factors to the point
    for (auto fac_ptr : getConstrainedByList())
        for (auto st_ptr : fac_ptr->getStateBlockPtrVector())
            if (st_ptr == state && getProblem() != nullptr)
            {
                getProblem()->removeFactor(fac_ptr);
                getProblem()->addFactor(fac_ptr);
            }

    // update boolean
    if (_back)
        last_defined_ = true;
    else
        first_defined_ = true;
}

void LandmarkPolyline2D::setClosed()
{
    std::cout << "setting polyline landmark closed" << std::endl;

    assert(getNPoints() - (first_defined_ ? 0 : 1) - (last_defined_ ? 0 : 1)  >= 2 && "closing a polyline with less than 2 defined points");

    // merge first not defined with last defined
    if (!first_defined_)
    {
        std::cout << "not defined first point: merging with last definite point" << std::endl;

        mergePoints(first_id_, getLastId() - (last_defined_ ? 0 : 1));
        first_id_++;
        first_defined_ = true;
    }
    // merge last not defined with first (defined for sure)
    if (!last_defined_)
    {
        std::cout << "not defined last point: merging with first point" << std::endl;

        mergePoints(getLastId(), first_id_);
        last_defined_ = true;
    }

    // set closed
    closed_ = true;
}

void LandmarkPolyline2D::mergePoints(int _remove_id, int _remain_id)
{
    std::cout << "merge points: remove " << _remove_id << " and keep " << _remain_id << " (ids: " << first_id_ << " to " << getLastId() << ")" << std::endl;

    assert(_remove_id >= first_id_ && _remove_id <= getLastId());
    assert(_remain_id >= first_id_ && _remain_id <= getLastId());
    assert(_remain_id > first_id_ || first_defined_);
    assert(_remain_id < getLastId() || last_defined_);

    // take a defined extreme as remaining
    StateBlockPtr remove_state = getPointStateBlock(_remove_id);
    std::cout << "state block to remove " << remove_state->getState().transpose() << std::endl;

    // Change factors from remove_state to remain_state
    FactorBasePtrList old_factors_list = getConstrainedByList();
    std::cout << "changing factors: " << old_factors_list.size() << std::endl;
    FactorBasePtr new_fac_ptr = nullptr;
    for (auto fac_ptr : old_factors_list)
    {
        FactorPoint2DPtr fac_point_ptr;
        FactorPointToLine2DPtr fac_point_line_ptr;
        if ( (fac_point_ptr = std::dynamic_pointer_cast<FactorPoint2D>(fac_ptr)))
//        if (fac_ptr->getTypeId() == FAC_POINT_2D)
        {
//            FactorPoint2DPtr fac_point_ptr = std::static_pointer_cast<FactorPoint2D>(fac_ptr);

            // If landmark point constrained -> new factor
            if (fac_point_ptr->getLandmarkPointId() == _remove_id)
                new_fac_ptr = std::make_shared<FactorPoint2D>(std::static_pointer_cast<FeaturePolyline2D>(fac_ptr->getFeature()),
                                                                  std::static_pointer_cast<LandmarkPolyline2D>(shared_from_this()),
                                                                  fac_point_ptr->getProcessor(),
                                                                  fac_point_ptr->getFeaturePointId(),
                                                                  _remain_id,
                                                                  fac_point_ptr->getApplyLossFunction(),
                                                                  fac_point_ptr->getStatus());
        }
        else if ((fac_point_line_ptr = std::dynamic_pointer_cast<FactorPointToLine2D>(fac_ptr)))
//        else if  (fac_ptr->getTypeId() == FAC_POINT_TO_LINE_2D)
        {
//            FactorPointToLine2DPtr fac_point_line_ptr = std::static_pointer_cast<FactorPointToLine2D>(fac_ptr);

            // If landmark point constrained -> new factor
            if (fac_point_line_ptr->getLandmarkPointId() == _remove_id)
                new_fac_ptr = std::make_shared<FactorPointToLine2D>(std::static_pointer_cast<FeaturePolyline2D>(fac_ptr->getFeature()),
                                                                        std::static_pointer_cast<LandmarkPolyline2D>(shared_from_this()),
                                                                        fac_point_line_ptr->getProcessor(),
                                                                        fac_point_line_ptr->getFeaturePointId(),
                                                                        _remain_id,
                                                                        fac_point_line_ptr->getLandmarkPointAuxId(),
                                                                        fac_point_ptr->getApplyLossFunction(),
                                                                        fac_point_line_ptr->getStatus());
            // If landmark point is aux point -> new factor
            else if (fac_point_line_ptr->getLandmarkPointAuxId() == _remove_id)
                new_fac_ptr = std::make_shared<FactorPointToLine2D>(std::static_pointer_cast<FeaturePolyline2D>(fac_ptr->getFeature()),
                                                                        std::static_pointer_cast<LandmarkPolyline2D>(shared_from_this()),
                                                                        fac_point_line_ptr->getProcessor(),
                                                                        fac_point_line_ptr->getFeaturePointId(),
                                                                        fac_point_line_ptr->getLandmarkPointId(),
                                                                        _remain_id,
                                                                        fac_point_line_ptr->getApplyLossFunction(),
                                                                        fac_point_line_ptr->getStatus());
        }
        else
            throw std::runtime_error ("polyline factor of unknown type");

        // If new factor
        if (new_fac_ptr != nullptr)
        {
            std::cout << "created new factor: " << new_fac_ptr->id() << std::endl;
            std::cout << "deleting factor: " << fac_ptr->id() << std::endl;

            // add new factor
            fac_ptr->getFeature()->addFactor(new_fac_ptr);

            // remove factor
            fac_ptr->remove();

            new_fac_ptr = nullptr;
        }
    }

    // Remove remove_state
    if (getProblem() != nullptr)
        getProblem()->removeStateBlock(remove_state);
    std::cout << "state removed " << std::endl;

    // remove element from deque
    point_state_ptr_vector_.erase(point_state_ptr_vector_.begin() + _remove_id - first_id_);
    std::cout << "state removed from point vector " << std::endl;
}

void LandmarkPolyline2D::registerNewStateBlocks()
{
    LandmarkBase::registerNewStateBlocks();
	if (getProblem() != nullptr)
		for (auto state : point_state_ptr_vector_)
			getProblem()->addStateBlock(state);
}

void LandmarkPolyline2D::removeStateBlocks()
{
    for (unsigned int i = 0; i < point_state_ptr_vector_.size(); i++)
    {
        auto sbp = point_state_ptr_vector_[i];
        if (sbp != nullptr)
        {
            if (getProblem() != nullptr)
            {
                getProblem()->removeStateBlock(sbp);
            }
            point_state_ptr_vector_[i] = nullptr;
        }
    }
    LandmarkBase::removeStateBlocks();
}

// static
LandmarkBasePtr LandmarkPolyline2D::create(const YAML::Node& _lmk_node)
{
    // Parse YAML node with lmk info and data
    unsigned int    id              = _lmk_node["id"].as<unsigned int>();
    Eigen::VectorXs pos             = _lmk_node["position"].as<Eigen::VectorXs>();
    bool            pos_fixed       = true;//_lmk_node["position fixed"].as<bool>();
    Eigen::VectorXs ori             = _lmk_node["orientation"].as<Eigen::VectorXs>();
    bool            ori_fixed       = true;//_lmk_node["orientation fixed"].as<bool>();
    int             first_id        = _lmk_node["first_id"].as<int>();
    bool            first_defined   = _lmk_node["first_defined"].as<bool>();
    bool            last_defined    = _lmk_node["last_defined"].as<bool>();
    unsigned int    npoints         = _lmk_node["points"].size();
    LandmarkClassification classification = (LandmarkClassification)(_lmk_node["classification"].as<int>());
    Eigen::MatrixXs points(2,npoints);
    for (unsigned int i = 0; i < npoints; i++)
    {
        points.col(i)               = _lmk_node["points"][i].as<Eigen::Vector2s>();
    }

    // Create a new landmark
    LandmarkPolyline2DPtr lmk_ptr = std::make_shared<LandmarkPolyline2D>(std::make_shared<StateBlock>(pos, pos_fixed), std::make_shared<StateBlock>(ori, ori_fixed), points, first_defined, last_defined, first_id, classification);
    lmk_ptr->setId(id);

    // fix all points
    for (auto st_ptr : lmk_ptr->getPointsStateBlockVector())
        st_ptr->fix();

    return lmk_ptr;
}

YAML::Node LandmarkPolyline2D::saveToYaml() const
{
    // First base things
    YAML::Node node = LandmarkBase::saveToYaml();

    // Then add specific things
    node["first_id"]       = first_id_;
    node["first_defined"]  = first_defined_;
    node["last_defined"]   = last_defined_;
    node["classification"] = (int)classification_;

    int npoints = point_state_ptr_vector_.size();

    for (int i = 0; i < npoints; i++)
    {
        node["points"].push_back(point_state_ptr_vector_[i]->getState());
    }

    return node;
}

// Register landmark creator
namespace
{
const bool WOLF_UNUSED registered_lmk_polyline_2D = LandmarkFactory::get().registerCreator("POLYLINE 2D", LandmarkPolyline2D::create);
}

} /* namespace wolf */
