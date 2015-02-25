
#include "landmark_base.h"

LandmarkBase::LandmarkBase(const LandmarkType & _tp, const StateBasePtr & _p_ptr) :
            NodeLinked(MID, "FRAME"),
            type_(_tp),
            status_(LANDMARK_CANDIDATE),
            hit_count_(1),
			st_list_({_p_ptr})
{
    //
}

LandmarkBase::LandmarkBase(const LandmarkType & _tp, const StateBasePtrList& _st_list) :
            NodeLinked(MID, "FRAME"),
            type_(_tp),
            status_(LANDMARK_CANDIDATE),
            hit_count_(1),
			st_list_(_st_list)
{
    //
}
                
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

const StateBasePtr LandmarkBase::getStatePtr() const
{
	return st_list_.front();
}

const StateBasePtrList* LandmarkBase::getStateListPtr() const
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
