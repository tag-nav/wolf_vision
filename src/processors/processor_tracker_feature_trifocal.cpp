
// wolf
#include "processor_tracker_feature_trifocal.h"

#include "sensor_camera.h"
#include "feature_point_image.h"
#include "constraints/constraint_autodiff_trifocal.h"
#include "capture_image.h"
#include "feature_point_image.h"

// vision_utils
#include <vision_utils/vision_utils.h>
#include <vision_utils/detectors.h>
#include <vision_utils/descriptors.h>
#include <vision_utils/matchers.h>
#include <vision_utils/algorithms.h>

#include <memory>

namespace wolf {


//// DEBUG =====================================
//debug_tTmp = clock();
//WOLF_TRACE("======== DetectNewFeatures =========");
//// ===========================================
//
//// DEBUG =====================================
//debug_comp_time_ = (double)(clock() - debug_tTmp) / CLOCKS_PER_SEC;
//WOLF_TRACE("--> TIME: Detect New Features: detect ",debug_comp_time_);
//// ===========================================


// Constructor
ProcessorTrackerFeatureTrifocal::ProcessorTrackerFeatureTrifocal(ProcessorParamsTrackerFeatureTrifocalPtr _params_tracker_feature_trifocal) :
        ProcessorTrackerFeature("TRACKER FEATURE TRIFOCAL", _params_tracker_feature_trifocal ),
        params_tracker_feature_trifocal_(_params_tracker_feature_trifocal),
        capture_last_(nullptr),
        capture_incoming_(nullptr),
        prev_origin_ptr_(nullptr),
        initialized_(false)
{
    setName(_params_tracker_feature_trifocal->name);
    assert(!(params_tracker_feature_trifocal_->yaml_file_params_vision_utils.empty()) && "Missing YAML file with vision_utils parameters!");
    assert(file_exists(params_tracker_feature_trifocal_->yaml_file_params_vision_utils) && "Cannot setup processor: vision_utils' YAML file does not exist.");

    // Detector
    std::string det_name = vision_utils::readYamlType(params_tracker_feature_trifocal_->yaml_file_params_vision_utils, "detector");
    det_ptr_ = vision_utils::setupDetector(det_name, det_name + " detector", params_tracker_feature_trifocal_->yaml_file_params_vision_utils);

    if (det_name.compare("ORB") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorORB>(det_ptr_);
    else if (det_name.compare("FAST") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorFAST>(det_ptr_);
    else if (det_name.compare("SIFT") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorSIFT>(det_ptr_);
    else if (det_name.compare("SURF") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorSURF>(det_ptr_);
    else if (det_name.compare("BRISK") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorBRISK>(det_ptr_);
    else if (det_name.compare("MSER") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorMSER>(det_ptr_);
    else if (det_name.compare("GFTT") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorGFTT>(det_ptr_);
    else if (det_name.compare("HARRIS") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorHARRIS>(det_ptr_);
    else if (det_name.compare("SBD") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorSBD>(det_ptr_);
    else if (det_name.compare("KAZE") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorKAZE>(det_ptr_);
    else if (det_name.compare("AKAZE") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorAKAZE>(det_ptr_);
    else if (det_name.compare("AGAST") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorAGAST>(det_ptr_);

    // Descriptor
    std::string des_name = vision_utils::readYamlType(params_tracker_feature_trifocal_->yaml_file_params_vision_utils, "descriptor");
    des_ptr_ = vision_utils::setupDescriptor(des_name, des_name + " descriptor", params_tracker_feature_trifocal_->yaml_file_params_vision_utils);

    if (des_name.compare("ORB") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorORB>(des_ptr_);
    else if (des_name.compare("SIFT") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorSIFT>(des_ptr_);
    else if (des_name.compare("SURF") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorSURF>(des_ptr_);
    else if (des_name.compare("BRISK") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorBRISK>(des_ptr_);
    else if (des_name.compare("KAZE") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorKAZE>(des_ptr_);
    else if (des_name.compare("AKAZE") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorAKAZE>(des_ptr_);
    else if (des_name.compare("LATCH") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorLATCH>(des_ptr_);
    else if (des_name.compare("FREAK") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorFREAK>(des_ptr_);
    else if (des_name.compare("BRIEF") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorBRIEF>(des_ptr_);
    else if (des_name.compare("DAISY") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorDAISY>(des_ptr_);
    else if (des_name.compare("LUCID") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorLUCID>(des_ptr_);

    // Matcher
    std::string mat_name = vision_utils::readYamlType(params_tracker_feature_trifocal_->yaml_file_params_vision_utils, "matcher");
    mat_ptr_ = vision_utils::setupMatcher(mat_name, mat_name + " matcher", params_tracker_feature_trifocal_->yaml_file_params_vision_utils);

    if (mat_name.compare("FLANNBASED") == 0)
        mat_ptr_ = std::static_pointer_cast<vision_utils::MatcherFLANNBASED>(mat_ptr_);
    if (mat_name.compare("BRUTEFORCE") == 0)
        mat_ptr_ = std::static_pointer_cast<vision_utils::MatcherBRUTEFORCE>(mat_ptr_);
    if (mat_name.compare("BRUTEFORCE_L1") == 0)
        mat_ptr_ = std::static_pointer_cast<vision_utils::MatcherBRUTEFORCE_L1>(mat_ptr_);
    if (mat_name.compare("BRUTEFORCE_HAMMING") == 0)
        mat_ptr_ = std::static_pointer_cast<vision_utils::MatcherBRUTEFORCE_HAMMING>(mat_ptr_);
    if (mat_name.compare("BRUTEFORCE_HAMMING_2") == 0)
        mat_ptr_ = std::static_pointer_cast<vision_utils::MatcherBRUTEFORCE_HAMMING_2>(mat_ptr_);

    // DEBUG VIEW
    cv::startWindowThread();
    cv::namedWindow("DEBUG VIEW", cv::WINDOW_NORMAL);
}

// Destructor
ProcessorTrackerFeatureTrifocal::~ProcessorTrackerFeatureTrifocal()
{
//    cv::destroyAllWindows();
}

bool ProcessorTrackerFeatureTrifocal::isInlier(const cv::KeyPoint& _kp_last, const cv::KeyPoint& _kp_incoming)
{
    // List of conditions
    bool inlier = true;

    // A. Check euclidean norm
    inlier = inlier && (cv::norm(_kp_incoming.pt-_kp_last.pt) < params_tracker_feature_trifocal_->max_euclidean_distance);

    return inlier;
}


unsigned int ProcessorTrackerFeatureTrifocal::detectNewFeatures(const unsigned int& _max_new_features, FeatureBaseList& _feature_list_out)
{
    // DEBUG =====================================
    clock_t debug_tStart;
    double debug_comp_time_;
    debug_tStart = clock();
    WOLF_TRACE("======== DetectNewFeatures =========");
    // ===========================================

    unsigned int n_new_features = 0;

    for (unsigned int n_iterations = 0; n_iterations < _max_new_features; ++n_iterations)
    {
        Eigen::Vector2i cell_last;
        if (capture_last_->grid_features_->pickEmptyCell(cell_last))
        {
            // Get best keypoint in cell
            vision_utils::FeatureIdxMap cell_feat_map = capture_last_->grid_features_->getFeatureIdxMap(cell_last(0),
                                                                                                        cell_last(1),
                                                                                                        params_tracker_feature_trifocal_->min_response_new_feature);

            bool found_feature_in_cell = false;

            for (auto target_last_pair_response_idx : cell_feat_map)
            {
                // Get last KeyPoint
                unsigned int index_last = target_last_pair_response_idx.second;
                cv::KeyPoint kp_last = capture_last_->keypoints_.at(index_last);
                assert(target_last_pair_response_idx.first == kp_last.response && "[ProcessorTrackerFeatureTrifocal::detectNewFeatures]: response mismatch.");

                // Check if there is match with incoming, if not we do not want it
                if (capture_last_->map_index_to_next_.count(index_last))
                {
                    unsigned int index_incoming = capture_last_->map_index_to_next_[index_last];
                    cv::KeyPoint kp_incoming = capture_incoming_->keypoints_.at(index_incoming);

                    if (isInlier( kp_incoming, kp_last))
                    {
                        // Create WOLF feature
                        FeaturePointImagePtr last_point_ptr = std::make_shared<FeaturePointImage>(
                                kp_last,
                                index_last,
                                capture_last_->descriptors_.row(index_last),
                                Eigen::Matrix2s::Identity() * params_tracker_feature_trifocal_->pixel_noise_std * params_tracker_feature_trifocal_->pixel_noise_std);

                        _feature_list_out.push_back(last_point_ptr);

                        // hit cell to acknowledge there's a tracked point in that cell
                        capture_last_->grid_features_->hitOccupancyGrid(kp_last);

                        n_new_features++;

                        found_feature_in_cell = true;

                        break; // Good kp found for this grid.
                    }
                }
            }
            if (!found_feature_in_cell)
                capture_last_->grid_features_->blockCell(cell_last);
        }
        else
            break; // There are no empty cells
    }

    WOLF_TRACE("DetectNewFeatures - Number of new features detected: " , n_new_features );

    // DEBUG
    debug_comp_time_ = (double)(clock() - debug_tStart) / CLOCKS_PER_SEC;
    WOLF_TRACE("--> TIME: detect new features: TOTAL ",debug_comp_time_);
    WOLF_TRACE("======== ========= =========");

    return n_new_features;
}

bool ProcessorTrackerFeatureTrifocal::voteForKeyFrame()
{
    // List of conditions

	// A. crossing voting threshold with ascending number of features
	bool vote_up = true;
	// 1. vote if we did not have enough features before
	vote_up = vote_up && (getNewFeaturesListLast().size() < params_tracker_feature_trifocal_->min_features_for_keyframe);
    // 2. vote if we have enough features now
    vote_up = vote_up && (getNewFeaturesListIncoming().size() >= params_tracker_feature_trifocal_->min_features_for_keyframe);

	// B. crossing voting threshold with descending number of features
    bool vote_down = true;
    // 1. vote if we had enough features before
    vote_down = vote_down && (getNewFeaturesListLast().size() >= params_tracker_feature_trifocal_->min_features_for_keyframe);
    // 2. vote if we have not enough features now
    vote_down = vote_down && (getNewFeaturesListIncoming().size() < params_tracker_feature_trifocal_->min_features_for_keyframe);

    return vote_up || vote_down;
}

bool ProcessorTrackerFeatureTrifocal::correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature, FeatureBasePtr _incoming_feature)
{
    return true;
}

unsigned int ProcessorTrackerFeatureTrifocal::trackFeatures(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out, FeatureMatchMap& _feature_matches)
{
    // DEBUG =====================================
    clock_t debug_tStart;
    double debug_comp_time_;
    debug_tStart = clock();
    WOLF_TRACE("======== TrackFeatures =========");
    // ===========================================

    for (auto feature_base_last_ : _feature_list_in)
    {

        WOLF_TRACE("");


        FeaturePointImagePtr feature_last_ = std::static_pointer_cast<FeaturePointImage>(feature_base_last_);

        WOLF_TRACE("");

        if ( capture_last_->map_index_to_next_.count(feature_last_->getIndexKeyPoint()) )
        {
            int index_kp_incoming = capture_last_->map_index_to_next_[feature_last_->getIndexKeyPoint()];

            WOLF_TRACE("");

            if (capture_incoming_->matches_normalized_scores_.at(index_kp_incoming) > mat_ptr_->getParams()->min_norm_score )
            {

                WOLF_TRACE("");

                // Check Euclidean distance between keypoints
                cv::KeyPoint kp_incoming = capture_incoming_->keypoints_.at(index_kp_incoming);
                cv::KeyPoint kp_last = capture_last_->keypoints_.at(feature_last_->getIndexKeyPoint());

                WOLF_TRACE("");

                if (isInlier(kp_last, kp_incoming))
                {
                    WOLF_TRACE("");

                    FeaturePointImagePtr incoming_point_ptr = std::make_shared<FeaturePointImage>(
                            kp_incoming,
                            index_kp_incoming,
                            capture_incoming_->descriptors_.row(index_kp_incoming),
                            Eigen::Matrix2s::Identity() * params_tracker_feature_trifocal_->pixel_noise_std * params_tracker_feature_trifocal_->pixel_noise_std);

                    WOLF_TRACE("");

                    _feature_list_out.push_back(incoming_point_ptr);


                    WOLF_TRACE("index_kp_incoming ", index_kp_incoming);
                    WOLF_TRACE("capture_incoming_->matches_normalized_scores_ ", capture_last_->matches_normalized_scores_.size());

                    _feature_matches[incoming_point_ptr] = std::make_shared<FeatureMatch>(FeatureMatch({feature_last_, capture_incoming_->matches_normalized_scores_.at(index_kp_incoming)}));

                    WOLF_TRACE("");

                    // hit cell to acknowledge there's a tracked point in that cell
                    capture_incoming_->grid_features_->hitOccupancyGrid(kp_incoming);

                    WOLF_TRACE("");
                }
            }
        }
    }

    // DEBUG
    debug_comp_time_ = (double)(clock() - debug_tStart) / CLOCKS_PER_SEC;
    WOLF_TRACE("--> TIME: track: ",debug_comp_time_);
    WOLF_TRACE("======== ========= =========");

    return _feature_list_out.size();
}

void ProcessorTrackerFeatureTrifocal::advanceDerived()
{
    ProcessorTrackerFeature::advanceDerived();
}

void ProcessorTrackerFeatureTrifocal::resetDerived()
{
    // Call parent class method
    ProcessorTrackerFeature::resetDerived();

    // Conditionally assign the prev_origin pointer
    if (initialized_)
        prev_origin_ptr_ = origin_ptr_;

//    // Print resulting list
//    TrackMatches matches_prevorig_origin = track_matrix_.matches(prev_origin_ptr_, origin_ptr_);
//    for (auto const & pair_trkid_pair : matches_prevorig_origin)
//    {
//        FeatureBasePtr feature_in_prev   = pair_trkid_pair.second.first;
//        FeatureBasePtr feature_in_origin = pair_trkid_pair.second.second;
//
//        WOLF_DEBUG("Matches reset prev <-- orig: track: ", pair_trkid_pair.first,
//                   " prev: ", feature_in_prev->id(),
//                   " origin: ", feature_in_origin->id());
//    }
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

    // Get capture
    capture_incoming_ = std::static_pointer_cast<CaptureImage>(incoming_ptr_);

    // Detect INC KeyPoints
    capture_incoming_->keypoints_ = det_ptr_->detect(capture_incoming_->getImage());

    // Get INC descriptors
    capture_incoming_->descriptors_ = des_ptr_->getDescriptor(capture_incoming_->getImage(), capture_incoming_->keypoints_);

    // Create and fill incoming grid
    capture_incoming_->grid_features_ = std::make_shared<vision_utils::FeatureIdxGrid>(capture_incoming_->getImage().rows, capture_incoming_->getImage().cols, params_tracker_feature_trifocal_->n_cells_v, params_tracker_feature_trifocal_->n_cells_h);

    capture_incoming_->grid_features_->insert(capture_incoming_->keypoints_);

    // If last_ptr_ is not null, then we can do some computation here.
    if (last_ptr_ != nullptr)
    {
        // Get capture
        capture_last_ = std::static_pointer_cast<CaptureImage>(last_ptr_);

        // Match full image (faster)
        // We exchange the order of the descriptors to fill better the map hereafter (map does not allow a single key)
        capture_incoming_->matches_normalized_scores_ = mat_ptr_->match(capture_incoming_->descriptors_, capture_last_->descriptors_, des_ptr_->getSize(), capture_incoming_->matches_from_precedent_);

        // Set capture map of match indices
        for (auto match : capture_incoming_->matches_from_precedent_)
            capture_last_->map_index_to_next_[match.trainIdx] = match.queryIdx; // map[last] = incoming
    }
}

void ProcessorTrackerFeatureTrifocal::postProcess()
{
    // DEBUG
    std::vector<vision_utils::KeyPointEnhanced> kps_e;

    for (auto const & feat_base : last_ptr_->getFeatureList())
    {
        FeaturePointImagePtr feat = std::static_pointer_cast<FeaturePointImage>(feat_base);
        unsigned int id = feat->id();
        unsigned int track_id = feat->trackId();
        vision_utils::KeyPointEnhanced kp_e(feat->getKeypoint(), id, track_id, track_matrix_.trackSize(track_id), feat->getMeasurementCovariance());
        kps_e.push_back(kp_e);
    }

    // DEBUG
    cv::Mat img = (std::static_pointer_cast<CaptureImage>(last_ptr_))->getImage();
    cv::Mat img_proc = vision_utils::buildImageProcessed(img, kps_e);

    cv::imshow("DEBUG VIEW", img_proc);
    cv::waitKey(1);
}

ConstraintBasePtr ProcessorTrackerFeatureTrifocal::createConstraint(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr)
{
    // NOTE: This function cannot be implemented because the API lacks an input to feature_prev_origin.
    // Therefore, we implement establishConstraints() instead and do all the job there.
    // This function remains empty, and with a warning or even an error issued in case someone calls it.
    std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerFeatureTrifocal::createConstraint is empty." << std::endl;
    ConstraintBasePtr return_var{}; //TODO: fill this variable
    return return_var;
}

void ProcessorTrackerFeatureTrifocal::establishConstraints()
{
    if (initialized_)
    {
        // get tracks between prev, origin and last
        TrackMatches matches = track_matrix_.matches(prev_origin_ptr_, last_ptr_); // it's guaranteed by construction that the track also includes origin

        for (auto pair_trkid_match : matches) // OMG this will add potentially a loooot of constraints! TODO see a smarter way of adding constraints
        {
            // get track ID
            size_t trk_id = pair_trkid_match.first;

            // get the three features for this track
            // FeatureBasePtr ftr_prev = track_matrix_.feature(trk_id, prev_origin_ptr_); // left here for ref, but implemented in a quicker way below
            // FeatureBasePtr ftr_last = track_matrix_.feature(trk_id, last_ptr_); // same here
            FeatureBasePtr ftr_prev = pair_trkid_match.second.first;
            FeatureBasePtr ftr_orig = track_matrix_.feature(trk_id, origin_ptr_); // because it's a tracker, this feature in the middle of prev and last exists for sure!
            FeatureBasePtr ftr_last = pair_trkid_match.second.second;

            // make constraint
            ConstraintAutodiffTrifocalPtr ctr = std::make_shared<ConstraintAutodiffTrifocal>(ftr_prev, ftr_orig, ftr_last, shared_from_this(), false, CTR_ACTIVE);

            // link to wolf tree
            ftr_last->addConstraint(ctr);
            ftr_orig->addConstrainedBy(ctr);
            ftr_prev->addConstrainedBy(ctr);
        }
    }
}

void ProcessorTrackerFeatureTrifocal::configure(SensorBasePtr _sensor)
{
    SensorCameraPtr camera = std::static_pointer_cast<SensorCamera>(_sensor);

    camera->setNoiseStd(Vector2s::Ones() * params_tracker_feature_trifocal_->pixel_noise_std);
}

ProcessorBasePtr ProcessorTrackerFeatureTrifocal::create(const std::string& _unique_name,
                                                         const ProcessorParamsBasePtr _params,
                                                         const SensorBasePtr _sensor_ptr)
{
  const auto params = std::static_pointer_cast<ProcessorParamsTrackerFeatureTrifocal>(_params);

  ProcessorBasePtr prc_ptr = std::make_shared<ProcessorTrackerFeatureTrifocal>(params);
  prc_ptr->setName(_unique_name);
  prc_ptr->configure(_sensor_ptr);
  return prc_ptr;
}

} // namespace wolf

// Register in the ProcessorFactory
#include "processor_factory.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR("TRACKER FEATURE TRIFOCAL", ProcessorTrackerFeatureTrifocal)
} // namespace wolf
