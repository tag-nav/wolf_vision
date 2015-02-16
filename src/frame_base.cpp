
#include "frame_base.h"

FrameBase::FrameBase(const TrajectoryBasePtr & _traj_ptr, const TimeStamp& _ts, const StateBaseShPtr& _p_ptr, const StateBaseShPtr& _o_ptr, const StateBaseShPtr& _v_ptr, const StateBaseShPtr& _w_ptr) :
            NodeLinked(MID, "FRAME", _traj_ptr),
            type_(REGULAR_FRAME),
            time_stamp_(_ts),
			p_ptr_(_p_ptr),
			o_ptr_(_o_ptr),
			v_ptr_(_v_ptr),
			w_ptr_(_w_ptr)
{
	//
}

FrameBase::FrameBase(const TrajectoryBasePtr & _traj_ptr, const FrameType & _tp, const TimeStamp& _ts, const StateBaseShPtr& _p_ptr, const StateBaseShPtr& _o_ptr, const StateBaseShPtr& _v_ptr, const StateBaseShPtr& _w_ptr) :
            NodeLinked(MID, "FRAME", _traj_ptr),
            type_(_tp),
            time_stamp_(_ts),
			p_ptr_(_p_ptr),
			o_ptr_(_o_ptr),
			v_ptr_(_v_ptr),
			w_ptr_(_w_ptr)
{
    //
}
                
FrameBase::~FrameBase()
{
    //
}

inline bool FrameBase::isKey() const
{
    if ( type_ == KEY_FRAME ) return true;
    else return false; 
}

inline void FrameBase::setType(FrameType _ft)
{
    type_ = _ft;
}

inline void FrameBase::setTimeStamp(const TimeStamp & _ts)
{
    time_stamp_ = _ts;
}

TimeStamp FrameBase::getTimeStamp() const
{
    return time_stamp_.get();
}
        
inline void FrameBase::getTimeStamp(TimeStamp & _ts) const
{
    _ts = time_stamp_.get();
}

void FrameBase::setState(const Eigen::VectorXs& _st)
{
	assert(_st.size() == ((!p_ptr_ ? 0 : p_ptr_->getStateSize()) +
						  (!o_ptr_ ? 0 : o_ptr_->getStateSize()) +
						  (!v_ptr_ ? 0 : v_ptr_->getStateSize()) +
						  (!w_ptr_ ? 0 : w_ptr_->getStateSize())) &&
						  "In FrameBase::setState wrong state size");
	assert(!!p_ptr_ && "in FrameBase::setState(), p_ptr_ is nullptr");

	Eigen::Map<Eigen::VectorXs> state_map(p_ptr_->getPtr(), _st.size());
	state_map = _st;
}

void FrameBase::addCapture(CaptureBaseShPtr & _capt_ptr)
{
    addDownNode(_capt_ptr);
}

inline const TrajectoryBasePtr FrameBase::getTrajectoryPtr() const
{
    return upperNodePtr();
}

// inline const CaptureBaseList & FrameBase::captureList() const
// {
//     return downNodeList();
// }

CaptureBaseList* FrameBase::getCaptureListPtr()
{
    return getDownNodeListPtr();
}

FrameBase* FrameBase::getPreviousFrame() const
{
    std::list<FrameBaseShPtr>::iterator f_it;
    std::list<FrameBaseShPtr>* f_list_ptr = this->up_node_ptr_->getFrameListPtr();

    //look for the position of this node in the upper list (frame list of trajectory)
    for ( f_it = f_list_ptr->begin(); f_it != f_list_ptr->end(); ++f_it )
    {
        if ( this->node_id_ == (f_it->get())->nodeId() ){
        	f_it--;
			return f_it->get();
        }
    }
    std::cout << "previous frame not found!" << std::endl;
    return nullptr;
}

//inline const Eigen::Vector3s & FrameBase::state() const
//{
//    return state_;
//}

StateBaseShPtr FrameBase::getPPtr() const
{
	return p_ptr_;
}

StateBaseShPtr FrameBase::getOPtr() const
{
	return o_ptr_;
}

StateBaseShPtr FrameBase::getVPtr() const
{
	return v_ptr_;
}

StateBaseShPtr FrameBase::getWPtr() const
{
	return w_ptr_;
}

void FrameBase::printSelf(unsigned int _ntabs, std::ostream& _ost) const
{
    NodeLinked::printSelf(_ntabs, _ost);
    if (p_ptr_.get() != nullptr)
    {
    	_ost << "\tPosition : \n";
    	p_ptr_->printSelf(_ntabs,_ost);
    }
    if (o_ptr_.get() != nullptr)
    {
		_ost << "\tOrientation : \n";
		o_ptr_->printSelf(_ntabs,_ost);
    }
    if (v_ptr_.get() != nullptr)
    {
    	_ost << "\tVelocity : \n";
    	v_ptr_->printSelf(_ntabs,_ost);
    }
    if (w_ptr_.get() != nullptr)
    {
    	_ost << "\tAngular velocity : \n";
    	v_ptr_->printSelf(_ntabs,_ost);
    }

}



