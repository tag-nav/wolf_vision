
#include "landmark_apriltag.h"

namespace wolf {

LandmarkApriltag::LandmarkApriltag(Eigen::Vector7s& pose, const int& _tagid, const Scalar& _tag_width) :
	LandmarkBase("APRILTAG", std::make_shared<StateBlock>(pose.head(3)), std::make_shared<StateQuaternion>(pose.tail(4))),
	tag_width_(_tag_width)
{
  	setDescriptor(Eigen::VectorXs::Constant(1,_tagid)); //change tagid to int ? do not use descriptor vector ?
    setId(_tagid);
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
