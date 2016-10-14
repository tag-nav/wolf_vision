
#include "landmark_base.h"
#include "constraint_base.h"
#include "map_base.h"
#include "state_block.h"
#include "yaml/yaml_conversion.h"

namespace wolf {

unsigned int LandmarkBase::landmark_id_count_ = 0;

LandmarkBase::LandmarkBase(const LandmarkType & _tp, const std::string& _type, StateBlock* _p_ptr, StateBlock* _o_ptr) :
            NodeBase("LANDMARK", _type),
            problem_ptr_(nullptr),
            map_ptr_(nullptr),
            landmark_id_(++landmark_id_count_),
            type_id_(_tp),
            status_(LANDMARK_CANDIDATE),
			p_ptr_(_p_ptr),
			o_ptr_(_o_ptr)
{
    //
}
                
LandmarkBase::~LandmarkBase()
{
	//std::cout << "deleting LandmarkBase " << nodeId() << std::endl;
    is_deleting_ = true;

    // Remove Frame State Blocks
    if (p_ptr_ != nullptr)
    {
        if (getProblem() != nullptr)
            getProblem()->removeStateBlockPtr(p_ptr_);
        delete p_ptr_;
    }
    if (o_ptr_ != nullptr)
    {
        if (getProblem() != nullptr)
            getProblem()->removeStateBlockPtr(o_ptr_);
        delete o_ptr_;
    }
	//std::cout << "states deleted" << std::endl;

	while (!constrained_by_list_.empty())
	{
	    //std::cout << "destruct() constraint " << (*constrained_by_list_.begin())->nodeId() << std::endl;
	    constrained_by_list_.front()->destruct();
	    constrained_by_list_.pop_front();
        //std::cout << "deleted " << std::endl;
	}
	//std::cout << "constraints deleted" << std::endl;
}

void LandmarkBase::setStatus(LandmarkStatus _st)
{
    status_ = _st;

    // State Blocks
    if (status_ == LANDMARK_FIXED)
    {
        if (p_ptr_!=nullptr)
        {
            p_ptr_->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(p_ptr_);
        }
        if (o_ptr_!=nullptr)
        {
            o_ptr_->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(o_ptr_);
        }
    }
    else if(status_ == LANDMARK_ESTIMATED)
    {
        if (p_ptr_!=nullptr)
        {
            p_ptr_->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(p_ptr_);
        }
        if (o_ptr_!=nullptr)
        {
            o_ptr_->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(o_ptr_);
        }
    }
}

void LandmarkBase::registerNewStateBlocks()
{
    if (getProblem() != nullptr)
    {
        if (p_ptr_ != nullptr)
            getProblem()->addStateBlockPtr(p_ptr_);
        if (o_ptr_ != nullptr)
            getProblem()->addStateBlockPtr(o_ptr_);
    }
}

YAML::Node LandmarkBase::saveToYaml() const
{
    YAML::Node node;
    node["id"] = landmark_id_;
    node["type"] = node_type_;
    if (p_ptr_ != nullptr)
    {
        node["position"] = p_ptr_->getVector();
        node["position fixed"] = p_ptr_->isFixed();
    }
    if (o_ptr_ != nullptr)
    {
        node["orientation"] = o_ptr_->getVector();
        node["orientation fixed"] = p_ptr_->isFixed();
    }
    return node;
}

} // namespace wolf
