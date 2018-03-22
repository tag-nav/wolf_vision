#include "processor_tracker_feature_trifocal.h"
 
#include "sensor_camera.h"
namespace wolf {

// Constructor
// TODO Modify this default API to suit your class needs
ProcessorTrackerFeatureTrifocal::ProcessorTrackerFeatureTrifocal(const ProcessorParamsTrackerFeatureTrifocal& _params) :
        ProcessorTrackerFeature("TRACKER_FEATURE_TRIFOCAL", _params.time_tolerance, _params.max_new_features ),
        cell_width_(0), cell_height_(0), // These will be initialized to proper values taken from the sensor via function configure()
        prev_origin_ptr_(nullptr),
        initialized_(false)
{
    // TODO Add your code or remove this comment
}

// Destructor
ProcessorTrackerFeatureTrifocal::~ProcessorTrackerFeatureTrifocal()
{
}

ConstraintBasePtr ProcessorTrackerFeatureTrifocal::createConstraint(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr)
{
  std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerFeatureTrifocal::createConstraint is empty." << std::endl;
  ConstraintBasePtr return_var{}; //TODO: fill this variable
  return return_var;
}

unsigned int ProcessorTrackerFeatureTrifocal::detectNewFeatures(const unsigned int& _max_features)
{
  std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerFeatureTrifocal::detectNewFeatures is empty." << std::endl;
  unsigned int return_var{}; //TODO: fill this variable
  return return_var;
}

bool ProcessorTrackerFeatureTrifocal::voteForKeyFrame()
{
  std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerFeatureTrifocal::voteForKeyFrame is empty." << std::endl;
  bool return_var{}; //TODO: fill this variable
  return return_var;
}

bool ProcessorTrackerFeatureTrifocal::correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature, FeatureBasePtr _incoming_feature)
{
  std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerFeatureTrifocal::correctFeatureDrift is empty." << std::endl;
  bool return_var{}; //TODO: fill this variable
  return return_var;
}

unsigned int ProcessorTrackerFeatureTrifocal::trackFeatures(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out, FeatureMatchMap& _feature_correspondences)
{
  std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerFeatureTrifocal::trackFeatures is empty." << std::endl;
  unsigned int return_var{}; //TODO: fill this variable
  return return_var;
}

void ProcessorTrackerFeatureTrifocal::resetDerived()
{
    // We also reset here the list of correspondences, which passes from origin--last to prev_origin--origin.
    matches_prev_origin_from_origin_ = std::move(matches_origin_from_last_);
    // Print resulting list
    for (auto match : matches_prev_origin_from_origin_)
    {
        WOLF_DEBUG("Matches reset prev <-- orig: track: ",
                   match.first->trackId(), "-", match.second->feature_ptr_->trackId(),
                   " origin: ", match.second->feature_ptr_->id(),
                   " last: ", match.first->id());
    }

    // Call parent class method
    ProcessorTrackerFeature::resetDerived();

    // Conditionally assign the prev_origin pointer
    if (initialized_)
    {
        prev_origin_ptr_ = origin_ptr_;
    }
}

void ProcessorTrackerFeatureTrifocal::preProcess()
{
    if (!initialized_)
    {
        if (origin_ptr_ && last_ptr_ && (last_ptr_ != origin_ptr_) && prev_origin_ptr_ == nullptr)
            prev_origin_ptr_ = origin_ptr_;

        if (prev_origin_ptr_ && origin_ptr_ && last_ptr_ && prev_origin_ptr_ != origin_ptr_)
            initialized_ = true;
    }
}

void ProcessorTrackerFeatureTrifocal::establishConstraints()
{
    if (initialized_)
    {
        // From ProcessorTrackerFeature::establishConstraints() :
        //                for (auto match : matches_origin_from_last_)
        //                {
        //                    auto ctr = createConstraint(match.first, match.second->feature_ptr_);
        //                    match.first->addConstraint(ctr);
        //                    match.second->feature_ptr_->addConstrainedBy(ctr);
        //                }
        //                for (auto match : matches_origin_from_last_)
        //                {
        //                    WOLF_DEBUG( "Constraint: track: " , match.second->feature_ptr_->trackId() ,
        //                                " origin: " , match.second->feature_ptr_->id() ,
        //                                " from last: " , match.first->id() );
        //                }
    }
}

void ProcessorTrackerFeatureTrifocal::configure(SensorBasePtr _sensor)
{
    SensorCameraPtr camera = std::static_pointer_cast<SensorCamera>(_sensor);

    active_search_ptr_->initAlg(camera->getImgWidth(), camera->getImgHeight() , det_ptr_->getPatternRadius());

    params_activesearch_ptr_ = std::static_pointer_cast<vision_utils::AlgorithmParamsACTIVESEARCH>( active_search_ptr_->getParams() );

    cell_width_  = camera->getImgWidth()  / params_activesearch_ptr_->n_cells_h;
    cell_height_ = camera->getImgHeight() / params_activesearch_ptr_->n_cells_v;
}

} // namespace wolf
