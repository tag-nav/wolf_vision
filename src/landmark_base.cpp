
#include "landmark_base.h"

LandmarkBase::LandmarkBase(const LandmarkType & _tp, const StateBaseShPtr & _p_ptr, const StateBaseShPtr & _o_ptr, const StateBaseShPtr & _v_ptr, const StateBaseShPtr & _w_ptr) :
            NodeLinked(MID, "LANDMARK"),
            type_(_tp),
            status_(LANDMARK_CANDIDATE),
            hit_count_(1),
			p_ptr_(_p_ptr),
			o_ptr_(_o_ptr),
			v_ptr_(_v_ptr),
			w_ptr_(_w_ptr)
{
    //
}

//LandmarkBase::LandmarkBase(const LandmarkType & _tp, const StateBasePtrList& _st_list) :
//            NodeLinked(MID, "LANDMARK"),
//            type_(_tp),
//            status_(LANDMARK_CANDIDATE),
//            hit_count_(1),
//			st_list_(_st_list)
//{
//    //
//}
                
LandmarkBase::~LandmarkBase()
{
    //
}

void LandmarkBase::setStatus(LandmarkStatus _st)
{
    status_ = _st;
}

void LandmarkBase::hit()
{
    hit_count_ ++;
}

unsigned int LandmarkBase::getHits() const
{
    return hit_count_;
}

StateBaseShPtr LandmarkBase::getPPtr() const
{
	return p_ptr_;
}

StateBaseShPtr LandmarkBase::getOPtr() const
{
	return o_ptr_;
}

StateBaseShPtr LandmarkBase::getVPtr() const
{
	return v_ptr_;
}

StateBaseShPtr LandmarkBase::getWPtr() const
{
	return w_ptr_;
}

void LandmarkBase::setDescriptor(const Eigen::VectorXs& _descriptor)
{
	descriptor_ = _descriptor;
}

const Eigen::VectorXs LandmarkBase::getDescriptor() const
{
	return descriptor_;
}

//const StateBasePtr LandmarkBase::getStatePtr() const
//{
//	return st_list_.front();
//}

//const StateBasePtrList* LandmarkBase::getStateListPtr() const
//{
//	return &st_list_;
//}

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
