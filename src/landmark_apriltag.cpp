
#include "landmark_apriltag.h"

namespace wolf {

LandmarkApriltag::LandmarkApriltag(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr,  const int& _tagid,  const Scalar& _tag_width) :
	LandmarkBase("APRILTAG", _p_ptr, _o_ptr), tag_width_(_tag_width)
{
  	setDescriptor(Eigen::VectorXs::Constant(1,_tagid)); //change tagid to int ? do not use descriptor vector ?
}

LandmarkApriltag::~LandmarkApriltag()
{
    //
}


Scalar LandmarkApriltag::getTagWidth() const
{
    return tag_width_;
}

int LandmarkApriltag::getTagId() const
{
    return round(descriptor_(0));
}

} // namespace wolf
