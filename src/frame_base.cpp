
#include "frame_base.h"

FrameBase::FrameBase(const TimeStamp& _ts, StateBase* _p_ptr, StateOrientation* _o_ptr, StateBase* _v_ptr, StateBase* _w_ptr) :
            //NodeLinked(MID, "FRAME", _traj_ptr),
            NodeLinked(MID, "FRAME"),
            type_(REGULAR_FRAME),
            time_stamp_(_ts),
			status_(ST_ESTIMATED),
			p_ptr_(_p_ptr),
			o_ptr_(_o_ptr),
			v_ptr_(_v_ptr),
			w_ptr_(_w_ptr)
{
	//
}

FrameBase::FrameBase(const FrameType & _tp, const TimeStamp& _ts, StateBase* _p_ptr, StateOrientation* _o_ptr, StateBase* _v_ptr, StateBase* _w_ptr) :
            //NodeLinked(MID, "FRAME", _traj_ptr),
            NodeLinked(MID, "FRAME"),
            type_(_tp),
            time_stamp_(_ts),
			status_(ST_ESTIMATED),
			p_ptr_(_p_ptr),
			o_ptr_(_o_ptr),
			v_ptr_(_v_ptr),
			w_ptr_(_w_ptr)
{
    //
}
                
FrameBase::~FrameBase()
{
	//std::cout << "deleting FrameBase " << nodeId() << std::endl;

	// Remove Frame State Units
	if (p_ptr_ != nullptr)
		getTop()->removeState(p_ptr_);
	if (o_ptr_ != nullptr)
		getTop()->removeState(o_ptr_);
	if (v_ptr_ != nullptr)
		getTop()->removeState(v_ptr_);
	if (w_ptr_ != nullptr)
		getTop()->removeState(w_ptr_);
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

void FrameBase::fix()
{
	//std::cout << "Fixing frame " << nodeId() << std::endl;
	status_ = ST_FIXED;

	// Frame State Units
	if (p_ptr_!=nullptr)
		p_ptr_->setStateStatus(ST_FIXED);
	if (o_ptr_!=nullptr)
		o_ptr_->setStateStatus(ST_FIXED);
	if (v_ptr_!=nullptr)
		v_ptr_->setStateStatus(ST_FIXED);
	if (w_ptr_!=nullptr)
		w_ptr_->setStateStatus(ST_FIXED);
}

void FrameBase::unfix()
{
	//std::cout << "Unfixing frame " << nodeId() << std::endl;
	status_ = ST_ESTIMATED;

	// Frame State Units
	if (p_ptr_!=nullptr)
		p_ptr_->setStateStatus(ST_ESTIMATED);
	if (o_ptr_!=nullptr)
		o_ptr_->setStateStatus(ST_ESTIMATED);
	if (v_ptr_!=nullptr)
		v_ptr_->setStateStatus(ST_ESTIMATED);
	if (w_ptr_!=nullptr)
		w_ptr_->setStateStatus(ST_ESTIMATED);
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

void FrameBase::setStatus(const StateStatus& _status)
{
	status_ = _status;
}

StateStatus FrameBase::getStatus() const
{
    return status_;
}

void FrameBase::setState(const Eigen::VectorXs& _st)
{

	assert(_st.size() == ((p_ptr_==nullptr ? 0 : p_ptr_->getStateSize()) +
                        (o_ptr_==nullptr ? 0 : o_ptr_->getStateSize()) +
                        (v_ptr_==nullptr ? 0 : v_ptr_->getStateSize()) +
                        (w_ptr_==nullptr ? 0 : w_ptr_->getStateSize())) &&
                        "In FrameBase::setState wrong state size");
	assert(p_ptr_!=nullptr && "in FrameBase::setState(), p_ptr_ is nullptr");

	Eigen::Map<Eigen::VectorXs> state_map(p_ptr_->getPtr(), _st.size());
	//std::cout << "setting state\noriginal: " << state_map.transpose() << "\nnew: " << _st.transpose() << std::endl;
	state_map = _st;
  //std::cout << "setted state: " << *p_ptr_->getPtr() << " " << *(p_ptr_->getPtr()+1) << std::endl;
}

Eigen::Map<Eigen::VectorXs> FrameBase::getState() const
{
    return Eigen::Map<Eigen::VectorXs>(p_ptr_->getPtr(),
                                       p_ptr_->getStateSize() + o_ptr_->getStateSize() + v_ptr_->getStateSize() + w_ptr_->getStateSize());
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
    //std::cout << "finding previous frame of " << this->node_id_ << std::endl;
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

StateBase* FrameBase::getPPtr() const
{
	return p_ptr_;
}

StateOrientation* FrameBase::getOPtr() const
{
	return o_ptr_;
}

StateBase* FrameBase::getVPtr() const
{
	return v_ptr_;
}

StateBase* FrameBase::getWPtr() const
{
	return w_ptr_;
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
    if (v_ptr_)
    {
    	printTabs(_ntabs);
    	_ost << "\tVelocity : \n";
    	printTabs(_ntabs);
    	v_ptr_->print(_ntabs,_ost);
    }
    if (w_ptr_)
    {
    	printTabs(_ntabs);
    	_ost << "\tAngular velocity : \n";
    	printTabs(_ntabs);
    	v_ptr_->print(_ntabs,_ost);
    }
}
