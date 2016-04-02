
#include "frame_base.h"
#include "constraint_base.h"
#include "trajectory_base.h"
#include "capture_base.h"
#include "state_block.h"

FrameBase::FrameBase(const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _v_ptr) :
            NodeLinked(MID, "FRAME"),
            type_(NON_KEY_FRAME),
            time_stamp_(_ts),
			status_(ST_ESTIMATED),
			p_ptr_(_p_ptr),
            o_ptr_(_o_ptr),
            v_ptr_(_v_ptr)
{
    //
}

FrameBase::FrameBase(const FrameType & _tp, const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _v_ptr) :
            NodeLinked(MID, "FRAME"),
            type_(_tp),
            time_stamp_(_ts),
			status_(ST_ESTIMATED),
			p_ptr_(_p_ptr),
            o_ptr_(_o_ptr),
            v_ptr_(_v_ptr)
{
    //
}
                
FrameBase::~FrameBase()
{
	//std::cout << "deleting FrameBase " << nodeId() << std::endl;
    is_deleting_ = true;

	// Remove Frame State Blocks
	if (p_ptr_ != nullptr)
	{
        if (getWolfProblem() != nullptr)
            getWolfProblem()->removeStateBlockPtr(p_ptr_);
	    delete p_ptr_;
	}
    if (o_ptr_ != nullptr)
    {
        if (getWolfProblem() != nullptr)
            getWolfProblem()->removeStateBlockPtr(o_ptr_);
        delete o_ptr_;
    }
    if (v_ptr_ != nullptr)
    {
        if (getWolfProblem() != nullptr)
            getWolfProblem()->removeStateBlockPtr(v_ptr_);
        delete v_ptr_;
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

void FrameBase::registerNewStateBlocks()
{
    if (getWolfProblem() != nullptr)
    {
        if (p_ptr_ != nullptr)
            getWolfProblem()->addStateBlockPtr(p_ptr_);

        if (o_ptr_ != nullptr)
            getWolfProblem()->addStateBlockPtr(o_ptr_);

        if (v_ptr_ != nullptr)
            getWolfProblem()->addStateBlockPtr(v_ptr_);
    }
}

void FrameBase::removeConstrainedBy(ConstraintBase* _ctr_ptr)
{
    constrained_by_list_.remove(_ctr_ptr);

    if (constrained_by_list_.empty())
        this->destruct();
}

void FrameBase::setKey()
{
    if (type_ != KEY_FRAME)
    {
        type_ = KEY_FRAME;
        registerNewStateBlocks();
    }
}

void FrameBase::setState(const Eigen::VectorXs& _st)
{

    assert(_st.size() == ((p_ptr_==nullptr ? 0 : p_ptr_->getSize())  +
                          (o_ptr_==nullptr ? 0 : o_ptr_->getSize())  +
                          (v_ptr_==nullptr ? 0 : v_ptr_->getSize())) &&
                          "In FrameBase::setState wrong state size");

    unsigned int index = 0;
    if (p_ptr_!=nullptr)
    {
        p_ptr_->setVector(_st.head(p_ptr_->getSize()));
        index += p_ptr_->getSize();
    }
    if (o_ptr_!=nullptr)
    {
        o_ptr_->setVector(_st.segment(index, o_ptr_->getSize()));
        index += p_ptr_->getSize();
    }
    if (v_ptr_!=nullptr)
    {
        v_ptr_->setVector(_st.segment(index, v_ptr_->getSize()));
        //   index += v_ptr_->getSize();
    }
}

Eigen::VectorXs FrameBase::getState() const
{
    Eigen::VectorXs state((p_ptr_==nullptr ? 0 : p_ptr_->getSize()) +
                          (o_ptr_==nullptr ? 0 : o_ptr_->getSize())  +
                          (v_ptr_==nullptr ? 0 : v_ptr_->getSize()));

    unsigned int index = 0;
    if (p_ptr_!=nullptr)
    {
        state.head(p_ptr_->getSize()) = p_ptr_->getVector();
        index += p_ptr_->getSize();
    }
    if (o_ptr_!=nullptr)
    {
        state.segment(index, o_ptr_->getSize()) = o_ptr_->getVector();
        index += p_ptr_->getSize();
    }
    if (v_ptr_!=nullptr)
    {
        state.segment(index, v_ptr_->getSize()) = v_ptr_->getVector();
        //   index += v_ptr_->getSize();
    }

    return state;
}

CaptureBaseIter FrameBase::hasCaptureOf(const SensorBase* _sensor_ptr)
{
    for (auto capture_it = getCaptureListPtr()->begin(); capture_it != getCaptureListPtr()->end(); capture_it++)
        if ((*capture_it)->getSensorPtr() == _sensor_ptr)
            return capture_it;
    return getCaptureListPtr()->end();
}

void FrameBase::getConstraintList(ConstraintBaseList & _ctr_list)
{
	for(auto c_it = getCaptureListPtr()->begin(); c_it != getCaptureListPtr()->end(); ++c_it)
		(*c_it)->getConstraintList(_ctr_list);
}

FrameBase* FrameBase::getPreviousFrame() const
{
    //std::cout << "finding previous frame of " << this->node_id_ << std::endl;
    if (getTrajectoryPtr() == nullptr)
        //std::cout << "This Frame is not linked to any trajectory" << std::endl;

    assert(getTrajectoryPtr() != nullptr && "This Frame is not linked to any trajectory");

    //look for the position of this node in the upper list (frame list of trajectory)
    for (auto f_it = getTrajectoryPtr()->getFrameListPtr()->rbegin(); f_it != getTrajectoryPtr()->getFrameListPtr()->rend(); f_it++ )
    {
        if ( this->node_id_ == (*f_it)->nodeId() )
        {
        	f_it++;
        	if (f_it != getTrajectoryPtr()->getFrameListPtr()->rend())
            {
                //std::cout << "previous frame found!" << std::endl;
                return *f_it;
            }
        	else
        	{
        	    //std::cout << "previous frame not found!" << std::endl;
        	    return nullptr;
        	}
        }
    }
    //std::cout << "previous frame not found!" << std::endl;
    return nullptr;
}

FrameBase* FrameBase::getNextFrame() const
{
    //std::cout << "finding next frame of " << this->node_id_ << std::endl;
	auto f_it = getTrajectoryPtr()->getFrameListPtr()->rbegin();
	f_it++; //starting from second last frame

    //look for the position of this node in the frame list of trajectory
    while (f_it != getTrajectoryPtr()->getFrameListPtr()->rend())
    {
        if ( this->node_id_ == (*f_it)->nodeId())
        {
        	f_it--;
			return *f_it;
        }
    	f_it++;
    }
    std::cout << "next frame not found!" << std::endl;
    return nullptr;
}

void FrameBase::setStatus(StateStatus _st)
{
    // TODO: Separate the three fixes and unfixes to the wolfproblem lists
    status_ = _st;
    // State Blocks
    if (status_ == ST_FIXED)
    {
        if (p_ptr_ != nullptr)
        {
            p_ptr_->fix();
            if (getWolfProblem() != nullptr)
                getWolfProblem()->updateStateBlockPtr(p_ptr_);
        }
        if (o_ptr_ != nullptr)
        {
            o_ptr_->fix();
            if (getWolfProblem() != nullptr)
                getWolfProblem()->updateStateBlockPtr(o_ptr_);
        }
        if (v_ptr_ != nullptr)
        {
            v_ptr_->fix();
            if (getWolfProblem() != nullptr)
                getWolfProblem()->updateStateBlockPtr(v_ptr_);
        }
    }
    else if (status_ == ST_ESTIMATED)
    {
        if (p_ptr_ != nullptr)
        {
            p_ptr_->unfix();
            if (getWolfProblem() != nullptr)
                getWolfProblem()->updateStateBlockPtr(p_ptr_);
        }
        if (o_ptr_ != nullptr)
        {
            o_ptr_->unfix();
            if (getWolfProblem() != nullptr)
                getWolfProblem()->updateStateBlockPtr(o_ptr_);
        }
        if (v_ptr_ != nullptr)
        {
            v_ptr_->unfix();
            if (getWolfProblem() != nullptr)
                getWolfProblem()->updateStateBlockPtr(v_ptr_);
        }
    }
}
