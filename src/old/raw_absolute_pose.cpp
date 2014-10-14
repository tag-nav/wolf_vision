#include "raw_absolute_pose.h"

RawAbsolutePose::RawAbsolutePose(const double & _ts, const VectorXs & _pose) :
    RawBase(_ts, _pose),
    pose_(_pose)
{
    //
}

RawAbsolutePose::~RawAbsolutePose()
{
    //
}
