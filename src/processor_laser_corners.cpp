#include "processor_laser_corners.h"

ProcessorLaserCorners::ProcessorLaserCorners::~ProcessorLaserCorners()
{
}

ProcessorLaserCorners::ProcessorLaserCorners::ProcessorLaserCorners() :
        ProcessorTracker(PRC_TRACKER_LIDAR, true)
{
}

unsigned int ProcessorLaserCorners::detectNewFeatures()
{
    return 0;
}

unsigned int ProcessorLaserCorners::track(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out)
{
    return 0;
}

LandmarkBase* ProcessorLaserCorners::createLandmark(FeatureBase* _feature_ptr)
{
    return nullptr;
}

ConstraintBase* ProcessorLaserCorners::createConstraint(FeatureBase* _feature_ptr, NodeBase* _node_ptr)
{
    return nullptr;
}

bool ProcessorLaserCorners::voteForKeyFrame()
{
    return false;
}
