
#include "landmark_base.h"
#include "constraint_base.h"
#include "map_base.h"
#include "node_terminus.h"
#include "state_block.h"

namespace wolf {

unsigned int LandmarkBase::landmark_id_count_ = 0;

LandmarkBase::LandmarkBase(const LandmarkType & _tp, StateBlock* _p_ptr, StateBlock* _o_ptr) :
            NodeConstrained(MID, "LANDMARK"),
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

	while (!getConstrainedByListPtr()->empty())
	{
	    //std::cout << "destruct() constraint " << (*constrained_by_list_.begin())->nodeId() << std::endl;
	    getConstrainedByListPtr()->front()->destruct();
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

} // namespace wolf
