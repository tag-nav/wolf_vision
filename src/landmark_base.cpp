
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
			constraint_to_list_({})
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

	while (!constraint_to_list_.empty())
	{
	    //std::cout << "destruct() constraint " << (*constraint_to_list_.begin())->nodeId() << std::endl;
	    constraint_to_list_.front()->destruct();
        //std::cout << "deleted " << std::endl;
	}
	//std::cout << "constraints deleted" << std::endl;
}

void LandmarkBase::addConstraintTo(ConstraintBase* _ctr_ptr)
{
    constraint_to_list_.push_back(_ctr_ptr);
}

void LandmarkBase::removeConstraintTo(ConstraintBase* _ctr_ptr)
{
    constraint_to_list_.remove(_ctr_ptr);

    if (constraint_to_list_.empty())
        this->destruct();
}

unsigned int LandmarkBase::getHits() const
{
    return constraint_to_list_.size();
}

std::list<ConstraintBase*>* LandmarkBase::getConstraintToListPtr()
{
    return &constraint_to_list_;
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

void LandmarkBase::fix()
{
	//std::cout << "Fixing frame " << nodeId() << std::endl;
    this->setStatus(LANDMARK_FIXED);
}

void LandmarkBase::unfix()
{
	//std::cout << "Unfixing frame " << nodeId() << std::endl;
    this->setStatus(LANDMARK_ESTIMATED);
}

StateBlock* LandmarkBase::getPPtr() const
{
	return p_ptr_;
}

StateBlock* LandmarkBase::getOPtr() const
{
	return o_ptr_;
}

void LandmarkBase::setPPtr(StateBlock* _st_ptr)
{
    p_ptr_ = _st_ptr;
}

void LandmarkBase::setOPtr(StateBlock* _st_ptr)
{
    o_ptr_ = _st_ptr;
}

void LandmarkBase::setDescriptor(const Eigen::VectorXs& _descriptor)
{
	descriptor_ = _descriptor;
}

const Eigen::VectorXs& LandmarkBase::getDescriptor() const
{
	return descriptor_;
}

WolfScalar LandmarkBase::getDescriptor(unsigned int _ii) const
{
    return descriptor_(_ii);
}

const LandmarkType LandmarkBase::getType() const
{
    return type_;
}
