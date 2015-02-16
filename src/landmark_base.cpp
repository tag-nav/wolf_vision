
#include "landmark_base.h"

LandmarkBase::LandmarkBase(const MapBasePtr& _traj_ptr, const LandmarkType & _tp,const StateBaseShPtr& _p_ptr) :
            NodeLinked(MID, "FRAME", _traj_ptr),
            type_(_tp),
			st_list_({_p_ptr})
{
    //
}
LandmarkBase::LandmarkBase(const MapBasePtr& _traj_ptr, const LandmarkType & _tp, const StateBaseList& _st_list) :
            NodeLinked(MID, "FRAME", _traj_ptr),
            type_(_tp),
			st_list_(_st_list)
{
    //
}
                
LandmarkBase::~LandmarkBase()
{
    //
}

void LandmarkBase::setType(LandmarkType _ft)
{
    type_ = _ft;
}

const StateBaseShPtr LandmarkBase::getPPtr()
{
	return st_list_.front();
}

StateBaseList* LandmarkBase::getStateListPtr()
{
	return &st_list_;
}

//void LandmarkBase::printSelf(unsigned int _ntabs, std::ostream& _ost) const
//{
//    NodeLinked::printSelf(_ntabs, _ost);
//    if (p_ptr_.get() != nullptr)
//    {
//    	_ost << "\tPosition : \n";
//    	p_ptr_->printSelf(_ntabs,_ost);
//    }
//    if (o_ptr_.get() != nullptr)
//    {
//		_ost << "\tOrientation : \n";
//		o_ptr_->printSelf(_ntabs,_ost);
//    }
//    if (v_ptr_.get() != nullptr)
//    {
//    	_ost << "\tVelocity : \n";
//    	v_ptr_->printSelf(_ntabs,_ost);
//    }
//    if (w_ptr_.get() != nullptr)
//    {
//    	_ost << "\tAngular velocity : \n";
//    	v_ptr_->printSelf(_ntabs,_ost);
//    }
//}



