#include "processor_tracker_landmark_apriltag.h"
 
namespace wolf {

// Constructor
// TODO Modify this default API to suit your class needs
ProcessorTrackerLandmarkApriltag::ProcessorTrackerLandmarkApriltag( ProcessorParamsTrackerLandmarkPtr _params_tracker_landmark) :
        ProcessorTrackerLandmark("TRACKER_LANDMARK_APRILTAG",  _params_tracker_landmark )
{
    // TODO Add your code or remove this comment
}

// Destructor
ProcessorTrackerLandmarkApriltag::~ProcessorTrackerLandmarkApriltag()
{
}

ConstraintBasePtr ProcessorTrackerLandmarkApriltag::createConstraint(FeatureBasePtr _feature_ptr, LandmarkBasePtr _landmark_ptr)
{
  std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerLandmarkApriltag::createConstraint is empty." << std::endl;
  ConstraintBasePtr return_var{}; //TODO: fill this variable
  return return_var;
}

LandmarkBasePtr ProcessorTrackerLandmarkApriltag::createLandmark(FeatureBasePtr _feature_ptr)
{
  std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerLandmarkApriltag::createLandmark is empty." << std::endl;
  LandmarkBasePtr return_var{}; //TODO: fill this variable
  return return_var;
}

unsigned int ProcessorTrackerLandmarkApriltag::detectNewFeatures(const unsigned int& _max_features, FeatureBaseList& _feature_list_out)
{
  std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerLandmarkApriltag::detectNewFeatures is empty." << std::endl;
  unsigned int return_var{}; //TODO: fill this variable
  return return_var;
}

bool ProcessorTrackerLandmarkApriltag::voteForKeyFrame()
{
  std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerLandmarkApriltag::voteForKeyFrame is empty." << std::endl;
  bool return_var{}; //TODO: fill this variable
  return return_var;
}

LandmarkMatchMap& ProcessorTrackerLandmarkApriltag::_feature_landmark_correspondences)(LandmarkMatchMap& _feature_landmark_correspondences)
{
  std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerLandmarkApriltag::_feature_landmark_correspondences) is empty." << std::endl;
  LandmarkMatchMap& return_var{}; //TODO: fill this variable
  return return_var;
}

} // namespace wolf
