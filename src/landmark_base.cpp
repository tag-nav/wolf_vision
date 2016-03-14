
#include "landmark_base.h"
#include "constraint_base.h"
#include "map_base.h"
#include "node_terminus.h"
#include "state_block.h"

LandmarkBase::LandmarkBase(const LandmarkType & _tp, StateBlock* _p_ptr, StateBlock* _o_ptr) :
            NodeLinked(MID, "LANDMARK"),
            type_(_tp),
            status_(LANDMARK_CANDIDATE),
			p_ptr_(_p_ptr),
			o_ptr_(_o_ptr),
			constrained_by_list_({})
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
        if (getTop() != nullptr)
            getTop()->removeStateBlockPtr(p_ptr_);
        delete p_ptr_;
    }
    if (o_ptr_ != nullptr)
    {
        if (getTop() != nullptr)
            getTop()->removeStateBlockPtr(o_ptr_);
        delete o_ptr_;
    }
	//std::cout << "states deleted" << std::endl;

	while (!constrained_by_list_.empty())
	{
	    //std::cout << "destruct() constraint " << (*constrained_by_list_.begin())->nodeId() << std::endl;
	    constrained_by_list_.front()->destruct();
        //std::cout << "deleted " << std::endl;
	}
	//std::cout << "constraints deleted" << std::endl;
}

void LandmarkBase::removeConstrainedBy(ConstraintBase* _ctr_ptr)
{
    constrained_by_list_.remove(_ctr_ptr);

    if (constrained_by_list_.empty())
        this->destruct();
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
            if (getTop() != nullptr)
                getTop()->updateStateBlockPtr(p_ptr_);
        }
        if (o_ptr_!=nullptr)
        {
            o_ptr_->fix();
            if (getTop() != nullptr)
                getTop()->updateStateBlockPtr(o_ptr_);
        }
    }
    else if(status_ == LANDMARK_ESTIMATED)
    {
        if (p_ptr_!=nullptr)
        {
            p_ptr_->unfix();
            if (getTop() != nullptr)
                getTop()->updateStateBlockPtr(p_ptr_);
        }
        if (o_ptr_!=nullptr)
        {
            o_ptr_->unfix();
            if (getTop() != nullptr)
                getTop()->updateStateBlockPtr(o_ptr_);
        }
    }
}

void LandmarkBase::registerNewStateBlocks()
{
    if (getTop() != nullptr)
    {
        if (p_ptr_ != nullptr)
            getTop()->addStateBlockPtr(p_ptr_);

        if (o_ptr_ != nullptr)
            getTop()->addStateBlockPtr(o_ptr_);
    }
}
