
#include "frame_base.h"

FrameBase::FrameBase(const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _o_ptr) :
            NodeLinked(MID, "FRAME"),
            type_(REGULAR_FRAME),
            time_stamp_(_ts),
			status_(ST_ESTIMATED),
			p_ptr_(_p_ptr),
			o_ptr_(_o_ptr)
{
    //
}

FrameBase::FrameBase(const FrameType & _tp, const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _o_ptr) :
            NodeLinked(MID, "FRAME"),
            type_(_tp),
            time_stamp_(_ts),
			status_(ST_ESTIMATED),
			p_ptr_(_p_ptr),
			o_ptr_(_o_ptr)
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
	    getTop()->removeStateBlockPtr(p_ptr_);
	    delete p_ptr_;
	}
	if (o_ptr_ != nullptr)
	{
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

void FrameBase::destruct()
{
    if (!is_deleting_)
        up_node_ptr_->removeDownNode(this);
}

void FrameBase::addConstraintTo(ConstraintBase* _ctr_ptr)
{
    constraint_to_list_.push_back(_ctr_ptr);
}

void FrameBase::removeConstraintTo(ConstraintBase* _ctr_ptr)
{
    constraint_to_list_.remove(_ctr_ptr);

    if (constraint_to_list_.empty())
        this->destruct();
}

unsigned int FrameBase::getHits() const
{
    return constraint_to_list_.size();
}

std::list<ConstraintBase*>* FrameBase::getConstraintToListPtr()
{
    return &constraint_to_list_;
}

void FrameBase::setStatus(StateStatus _st)
{
    status_ = _st;

    // State Blocks
    if (status_ == ST_FIXED)
    {
        if (p_ptr_!=nullptr)
        {
            p_ptr_->fix();
            getTop()->updateStateBlockPtr(p_ptr_);
        }
        if (o_ptr_!=nullptr)
        {
            o_ptr_->fix();
            getTop()->updateStateBlockPtr(o_ptr_);
        }
    }
    else if(status_ == ST_ESTIMATED)
    {
        if (p_ptr_!=nullptr)
        {
            p_ptr_->unfix();
            getTop()->updateStateBlockPtr(p_ptr_);
        }
        if (o_ptr_!=nullptr)
        {
            o_ptr_->unfix();
            getTop()->updateStateBlockPtr(o_ptr_);
        }
    }
}

void FrameBase::fix()
{
    //std::cout << "Fixing frame " << nodeId() << std::endl;
    this->setStatus(ST_FIXED);
}

void FrameBase::unfix()
{
    //std::cout << "Unfixing frame " << nodeId() << std::endl;
    this->setStatus(ST_ESTIMATED);
}

bool FrameBase::isKey() const
{
    if ( type_ == KEY_FRAME ) return true;
    else return false; 
}

void FrameBase::setType(FrameType _ft)
{
    type_ = _ft;
}

void FrameBase::setTimeStamp(const TimeStamp & _ts)
{
    time_stamp_ = _ts;
}

TimeStamp FrameBase::getTimeStamp() const
{
    return time_stamp_.get();
}
        
void FrameBase::getTimeStamp(TimeStamp & _ts) const
{
    _ts = time_stamp_.get();
}

StateStatus FrameBase::getStatus() const
{
    return status_;
}

void FrameBase::setState(const Eigen::VectorXs& _st)
{

	assert(_st.size() == ((p_ptr_==nullptr ? 0 : p_ptr_->getSize())  +
                          (o_ptr_==nullptr ? 0 : o_ptr_->getSize())) &&
                          "In FrameBase::setState wrong state size");

	if (p_ptr_!=nullptr)
        p_ptr_->setVector(_st.head(p_ptr_->getSize()));
    if (o_ptr_!=nullptr)
        o_ptr_->setVector(_st.segment((p_ptr_==nullptr ? 0 : p_ptr_->getSize()),
                                       o_ptr_->getSize()));
}

Eigen::VectorXs FrameBase::getState() const
{
    Eigen::VectorXs state((p_ptr_==nullptr ? 0 : p_ptr_->getSize()) +
                          (o_ptr_==nullptr ? 0 : o_ptr_->getSize()));
    state << p_ptr_->getVector(), o_ptr_->getVector();
    return state;
}

void FrameBase::addCapture(CaptureBase* _capt_ptr)
{
    addDownNode(_capt_ptr);
}

void FrameBase::removeCapture(CaptureBaseIter& _capt_iter)
{
	//std::cout << "removing capture " << (*_capt_iter)->nodeId() << " from Frame " << nodeId() << std::endl;
	removeDownNode(_capt_iter);
}

TrajectoryBase* FrameBase::getTrajectoryPtr() const
{
    return upperNodePtr();
}

CaptureBaseList* FrameBase::getCaptureListPtr()
{
    return getDownNodeListPtr();
}

void FrameBase::getConstraintList(ConstraintBaseList & _ctr_list)
{
	for(auto c_it = getCaptureListPtr()->begin(); c_it != getCaptureListPtr()->end(); ++c_it)
		(*c_it)->getConstraintList(_ctr_list);
}

FrameBase* FrameBase::getPreviousFrame() const
{
    //std::cout << "finding previous frame of " << this->node_id_ << std::endl;

    //look for the position of this node in the upper list (frame list of trajectory)
    for (auto f_it = getTrajectoryPtr()->getFrameListPtr()->rbegin(); f_it != getTrajectoryPtr()->getFrameListPtr()->rend(); f_it++ )
    {
        if ( this->node_id_ == (*f_it)->nodeId() )
        {
        	f_it++;
			return *f_it;
        }
    }
    std::cout << "previous frame not found!" << std::endl;
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

StateBlock* FrameBase::getPPtr() const
{
	return p_ptr_;
}

StateBlock* FrameBase::getOPtr() const
{
	return o_ptr_;
}

CaptureBaseIter FrameBase::hasCaptureOf(const SensorBase* _sensor_ptr)
{
    for (auto capture_it = getCaptureListPtr()->begin(); capture_it != getCaptureListPtr()->end(); capture_it++)
        if ((*capture_it)->getSensorPtr() == _sensor_ptr)
            return capture_it;

    return getCaptureListPtr()->end();
}

void FrameBase::printSelf(unsigned int _ntabs, std::ostream& _ost) const
{
    NodeLinked::printSelf(_ntabs, _ost);
    if (p_ptr_)
    {
    	printTabs(_ntabs);
    	_ost << "\tPosition : \n";
    	printTabs(_ntabs);
    	p_ptr_->print(_ntabs,_ost);
    }
    if (o_ptr_)
    {
    	printTabs(_ntabs);
		_ost << "\tOrientation : \n";
    	printTabs(_ntabs);
		o_ptr_->print(_ntabs,_ost);
    }
}
