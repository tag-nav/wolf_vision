
// wolf
#include "processor_tracker_feature_trifocal.h"

#include "sensor_camera.h"
#include "feature_point_image.h"
#include "constraints/constraint_autodiff_trifocal.h"
#include "capture_image.h"

// vision_utils
#include <vision_utils.h>
#include <detectors.h>
#include <descriptors.h>
#include <matchers.h>
#include <algorithms.h>

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
        capture_image_last_(nullptr),
        capture_image_incoming_(nullptr),
        prev_origin_ptr_(nullptr),
        initialized_(false)
{
    setName(_params_tracker_feature_trifocal->name);
    assert(!(params_tracker_feature_trifocal_->yaml_file_params_vision_utils.empty()) && "Missing YAML file with vision_utils parameters!");
    assert(file_exists(params_tracker_feature_trifocal_->yaml_file_params_vision_utils) && "Cannot setup processor: vision_utils' YAML file does not exist.");

    pixel_cov_ = Eigen::Matrix2s::Identity() * params_tracker_feature_trifocal_->pixel_noise_std * params_tracker_feature_trifocal_->pixel_noise_std;

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

//    // DEBUG VIEW
    cv::startWindowThread();
    cv::namedWindow("DEBUG VIEW", cv::WINDOW_NORMAL);
//    cv::namedWindow("DEBUG MATCHES", cv::WINDOW_NORMAL);
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
    inlier = inlier && (cv::norm(_kp_incoming.pt-_kp_last.pt) < mat_ptr_->getParams()->max_match_euclidean_dist);

    return inlier;
}


unsigned int ProcessorTrackerFeatureTrifocal::detectNewFeatures(const unsigned int& _max_new_features, FeatureBaseList& _features_last_out)
{
//    // DEBUG =====================================
//    clock_t debug_tStart;
//    double debug_comp_time_;
//    debug_tStart = clock();
//    WOLF_TRACE("======== DetectNewFeatures =========");
//    // ===========================================

    for (unsigned int n_iterations = 0; n_iterations < _max_new_features; ++n_iterations)
    {
        Eigen::Vector2i cell_last;
        if (capture_image_last_->grid_features_->pickEmptyTrackingCell(cell_last))
        {
            // Get best keypoint in cell
            vision_utils::FeatureIdxMap cell_feat_map = capture_image_last_->grid_features_->getFeatureIdxMap(cell_last(0), cell_last(1));
            bool found_feature_in_cell = false;

            for (auto target_last_pair_response_idx : cell_feat_map)
            {
                // Get KeyPoint in last
                unsigned int index_last = target_last_pair_response_idx.second;
                cv::KeyPoint kp_last = capture_image_last_->keypoints_.at(index_last);
                assert(target_last_pair_response_idx.first == kp_last.response && "[ProcessorTrackerFeatureTrifocal::detectNewFeatures]: response mismatch.");

                // Check if there is match with incoming, if not we do not want it
                if (capture_image_last_->map_index_to_next_.count(index_last))
                {
                    // matching keypoint in incoming
                    unsigned int index_incoming = capture_image_last_->map_index_to_next_[index_last];
                    cv::KeyPoint kp_incoming = capture_image_incoming_->keypoints_.at(index_incoming);

                    // validate match with extra tests
                    if (isInlier( kp_incoming, kp_last))
                    {
                        // Create WOLF feature
                        FeaturePointImagePtr ftr_point_last = std::make_shared<FeaturePointImage>(
                                kp_last,
                                index_last,
                                capture_image_last_->descriptors_.row(index_last),
                                pixel_cov_);

                        _features_last_out.push_back(ftr_point_last);

                        // hit cell to acknowledge there's a tracked point in that cell
                        capture_image_last_->grid_features_->hitTrackingCell(kp_last);

                        found_feature_in_cell = true;

                        break; // Good kp found for this grid.
                    }
                }
            }
            if (!found_feature_in_cell)
                capture_image_last_->grid_features_->blockTrackingCell(cell_last);
        }
        else
            break; // There are no empty cells
    }

//    // DEBUG
//    WOLF_TRACE("DetectNewFeatures - Number of new features detected: " , _feature_list_out.size() );
//    debug_comp_time_ = (double)(clock() - debug_tStart) / CLOCKS_PER_SEC;
//    WOLF_TRACE("--> TIME: detect new features: TOTAL ",debug_comp_time_);
//    WOLF_TRACE("======== END DETECT NEW FEATURES =========");

    return _features_last_out.size();
}

unsigned int ProcessorTrackerFeatureTrifocal::trackFeatures(const FeatureBaseList& _features_last_in, FeatureBaseList& _features_incoming_out, FeatureMatchMap& _feature_matches)
{
//    // DEBUG =====================================
//    clock_t debug_tStart;
//    double debug_comp_time_;
//    debug_tStart = clock();
//    WOLF_TRACE("======== TrackFeatures =========");
//    // ===========================================

    for (auto feature_base_last_ : _features_last_in)
    {
        FeaturePointImagePtr feature_last_ = std::static_pointer_cast<FeaturePointImage>(feature_base_last_);

        if ( capture_image_last_->map_index_to_next_.count(feature_last_->getIndexKeyPoint()) )
        {
            int index_incoming = capture_image_last_->map_index_to_next_[feature_last_->getIndexKeyPoint()];

            if (capture_image_incoming_->matches_normalized_scores_.at(index_incoming) > mat_ptr_->getParams()->min_norm_score )
            {
                // Check Euclidean distance between keypoints
                cv::KeyPoint kp_incoming = capture_image_incoming_->keypoints_.at(index_incoming);
                cv::KeyPoint kp_last = capture_image_last_->keypoints_.at(feature_last_->getIndexKeyPoint());

                if (isInlier(kp_last, kp_incoming))
                {
                    FeaturePointImagePtr ftr_point_incoming = std::make_shared<FeaturePointImage>(
                            kp_incoming,
                            index_incoming,
                            capture_image_incoming_->descriptors_.row(index_incoming),
                            pixel_cov_);

                    _features_incoming_out.push_back(ftr_point_incoming);

                    _feature_matches[ftr_point_incoming] = std::make_shared<FeatureMatch>(FeatureMatch({feature_last_, capture_image_incoming_->matches_normalized_scores_.at(index_incoming)}));

                    // hit cell to acknowledge there's a tracked point in that cell
                    capture_image_incoming_->grid_features_->hitTrackingCell(kp_incoming);
                }
            }
        }
    }

//    // DEBUG
//    WOLF_TRACE("TrAckFeatures - Number of features tracked: " , _feature_list_out.size() );
//    debug_comp_time_ = (double)(clock() - debug_tStart) / CLOCKS_PER_SEC;
//    WOLF_TRACE("--> TIME: track: ",debug_comp_time_);
//    WOLF_TRACE("======== END TRACK FEATURES =========");

    return _features_incoming_out.size();
}

bool ProcessorTrackerFeatureTrifocal::correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature, FeatureBasePtr _incoming_feature)
{
    return true;
}

bool ProcessorTrackerFeatureTrifocal::voteForKeyFrame()
{
//    // A. crossing voting threshold with ascending number of features
    bool vote_up = false;
//    // 1. vote if we did not have enough features before
//    vote_up = vote_up && (previousNumberOfTracks() < params_tracker_feature_trifocal_->min_features_for_keyframe);
//    // 2. vote if we have enough features now
//    vote_up = vote_up && (incoming_ptr_->getFeatureList().size() >= params_tracker_feature_trifocal_->min_features_for_keyframe);

    // B. crossing voting threshold with descending number of features
    bool vote_down = true;
    // 1. vote if we had enough features before
//    vote_down = vote_down && (last_ptr_->getFeatureList().size() >= params_tracker_feature_trifocal_->min_features_for_keyframe);
    // 2. vote if we have not enough features now
    vote_down = vote_down && (incoming_ptr_->getFeatureList().size() < params_tracker_feature_trifocal_->min_features_for_keyframe);

//    // C. Time-based policies
    bool vote_time = false;
////    vote_time = vote_time || (incoming_ptr_->getTimeStamp()-origin_ptr_->getTimeStamp() > 1.0);
//
//    if (vote_up)
//        WOLF_TRACE("VOTE UP");
//    if (vote_down)
//        WOLF_TRACE("VOTE DOWN");
//    if (vote_time)
//        WOLF_TRACE("VOTE TIME");

    return vote_up || vote_down || vote_time;
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
//    //DEBUG
//    WOLF_TRACE("-------- Image ", getIncomingPtr()->id(), " -- t = ", getIncomingPtr()->getTimeStamp(), " s ----------");

    if (!initialized_)
    {
        if (origin_ptr_ && last_ptr_ && (last_ptr_ != origin_ptr_) && prev_origin_ptr_ == nullptr)
            prev_origin_ptr_ = origin_ptr_;

        if (prev_origin_ptr_ && origin_ptr_ && last_ptr_ && prev_origin_ptr_ != origin_ptr_)
            initialized_ = true;
    }

    // Get capture
    capture_image_incoming_ = std::static_pointer_cast<CaptureImage>(incoming_ptr_);

    // Detect INC KeyPoints
    capture_image_incoming_->keypoints_ = det_ptr_->detect(capture_image_incoming_->getImage());

    // Get INC descriptors
    capture_image_incoming_->descriptors_ = des_ptr_->getDescriptor(capture_image_incoming_->getImage(), capture_image_incoming_->keypoints_);

    // Create and fill incoming grid
    capture_image_incoming_->grid_features_ = std::make_shared<vision_utils::FeatureIdxGrid>(capture_image_incoming_->getImage().rows, capture_image_incoming_->getImage().cols, params_tracker_feature_trifocal_->n_cells_v, params_tracker_feature_trifocal_->n_cells_h);

    capture_image_incoming_->grid_features_->insert(capture_image_incoming_->keypoints_);

    // If last_ptr_ is not null, then we can do some computation here.
    if (last_ptr_ != nullptr)
    {
        // Get capture
        capture_image_last_ = std::static_pointer_cast<CaptureImage>(last_ptr_);

        // Match full image (faster)
        // We exchange the order of the descriptors to fill better the map hereafter (map does not allow a single key)
        capture_image_incoming_->matches_normalized_scores_ = mat_ptr_->robustMatch(capture_image_incoming_->keypoints_,
                                                                              capture_image_last_->keypoints_,
                                                                              capture_image_incoming_->descriptors_,
                                                                              capture_image_last_->descriptors_,
                                                                              des_ptr_->getSize(),
                                                                              capture_image_incoming_->matches_from_precedent_);

        // Set capture map of match indices
        for (auto match : capture_image_incoming_->matches_from_precedent_)
            capture_image_last_->map_index_to_next_[match.trainIdx] = match.queryIdx; // map[last] = incoming

        // DEBUG
//        cv::Mat img_last = (std::static_pointer_cast<CaptureImage>(last_ptr_))->getImage();
//        cv::Mat img_incoming = (std::static_pointer_cast<CaptureImage>(incoming_ptr_))->getImage();
//
//        cv::putText(img_last, "LAST",    cv::Point(img_last.cols/2,20), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 10.0);
//        cv::putText(img_incoming, "INCOMING",cv::Point(img_last.cols/2,20), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 10.0);
//
//        cv::Mat img_matches;
//        cv::drawMatches(img_incoming, capture_incoming_->keypoints_, img_last, capture_last_->keypoints_, capture_incoming_->matches_from_precedent_, img_matches);
//        cv::imshow("DEBUG MATCHES", img_matches);
//        cv::waitKey(0);
    }
}

void ProcessorTrackerFeatureTrifocal::postProcess()
{
    // DEBUG
    std::vector<vision_utils::KeyPointEnhanced> kps_e;

    std::map<int,std::list<vision_utils::KeyPointEnhanced> > map_of_features_trackedkps;

    for (auto const & feat_base : last_ptr_->getFeatureList())
    {
        FeaturePointImagePtr feat = std::static_pointer_cast<FeaturePointImage>(feat_base);
        unsigned int feat_id = feat->id();
        unsigned int track_id = feat->trackId();

        // tracks
        std::list<vision_utils::KeyPointEnhanced> track_list;
        for (auto feat_base : track_matrix_.trackAsVector(track_id))
        {
            FeaturePointImagePtr feat_track = std::static_pointer_cast<FeaturePointImage>(feat_base);
            vision_utils::KeyPointEnhanced kp_e(feat_track->getKeypoint(), feat_id, track_id, track_matrix_.trackSize(track_id), feat_track->getMeasurementCovariance());
            track_list.push_back(kp_e);
        }

        map_of_features_trackedkps[feat_id] = track_list;
    }

    // DEBUG
    image_debug_ = vision_utils::buildImageProcessed((std::static_pointer_cast<CaptureImage>(last_ptr_))->getImage(), map_of_features_trackedkps);

    cv::imshow("DEBUG VIEW", image_debug_);
    cv::waitKey(1);
}

ConstraintBasePtr ProcessorTrackerFeatureTrifocal::createConstraint(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr)
{
    // NOTE: This function cannot be implemented because the API lacks an input to feature_prev_origin.
    // Therefore, we implement establishConstraints() instead and do all the job there.
    // This function remains empty, and with a warning or even an error issued in case someone calls it.
    std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerFeatureTrifocal::createConstraint is empty." << std::endl;
    ConstraintBasePtr return_var{};
    return return_var;
}

void ProcessorTrackerFeatureTrifocal::establishConstraints()
{
//    WOLF_TRACE("===== ESTABLISH CONSTRAINT =====");

    if (initialized_)
    {
        // get tracks between prev, origin and last
        TrackMatches matches = track_matrix_.matches(prev_origin_ptr_, last_ptr_); // it's guaranteed by construction that the track also includes origin

        for (auto pair_trkid_match : matches) // OMG this will add potentially a loooot of constraints! TODO see a smarter way of adding constraints
        {                                     // Currently reduced by creating constraints for large tracks
            // get track ID
            SizeStd trk_id = pair_trkid_match.first;

            size_t trk_length = track_matrix_.trackSize(trk_id);

            if (trk_length >= params_tracker_feature_trifocal_->min_track_length_for_constraint)
            {
                // get the last feature in the track
                FeatureBasePtr ftr_last = pair_trkid_match.second.second;

                // get the first feature in the whole track
                FeatureBasePtr ftr_first = track_matrix_.firstFeature(trk_id);

                // get the middle feature of the track using the average of the time stamps
                FeatureBasePtr ftr_mid = nullptr;

                TimeStamp ts_first      = ftr_first->getCapturePtr()->getTimeStamp();
                TimeStamp ts_last       = ftr_last->getCapturePtr()->getTimeStamp();
                Scalar    Dt2           = (ts_last - ts_first) / 2.0;
                TimeStamp ts_ave        = ts_first + Dt2;

                Scalar dt_err = Dt2;
                auto track = track_matrix_.track(trk_id);
                for (auto ftr_it = track.begin() ; ftr_it != track.end() ; ftr_it ++)
                {
                    if ( ftr_it->second->getCapturePtr() != nullptr ) // have capture
                    {
                        if ( auto kf_mid = ftr_it->second->getCapturePtr()->getFramePtr() ) // have frame
                        {
                            TimeStamp ts_mid    = kf_mid->getTimeStamp();

                            auto dt_err_curr = fabs(ts_mid - ts_ave);
                            if (dt_err_curr <= dt_err)
                            {
                                dt_err  = dt_err_curr;
                                ftr_mid = ftr_it->second;
                            }
                            else //if (dt_err_increasing)
                                break;
                        }
                    }
                }

//                // DEBUG
//                std::cout << " TRACK "  << trk_id
//                        << " prev: "    << prev_origin_ptr_->getTimeStamp() << ", "  << prev_origin_ptr_
//                        << " origin: "  << origin_ptr_->getTimeStamp() << ", "  << origin_ptr_ << (origin_ptr_->getFramePtr()->isKey() ? " KF" : " NO kf")
//                        << " last: "    << last_ptr_->getTimeStamp() << ", "  << last_ptr_
//                        << " incoming " << incoming_ptr_->getTimeStamp() << ", "  << incoming_ptr_
//                        << std::endl;
//                for (auto ftr_it = track.begin() ; ftr_it != track.end() ; ftr_it ++)
//                {
//                    if ( ftr_it->second->getCapturePtr() != nullptr ) // have capture
//                        std::cout
//                        << " ts: " << ftr_it->first << ", "
//                        << ftr_it->second->id() << ", "
//                        << ftr_it->second->getCapturePtr()->getTimeStamp() << ", "
//                        << ftr_it->second->getCapturePtr() << std::endl;
//                    else
//                        std::cout << " ts: " << ftr_it->first << ", " << ftr_it->second->id() << ", -- || ";
//                }
//                std::cout << std::endl;

//                FeatureBasePtr ftr_mid  = track_matrix_.feature(trk_id, ts_mid - 1e-4); // 1e-4 to be on the safe side if numerical errors occur

                assert(ftr_mid != nullptr   && "Middle feature is nullptr!");
                assert(ftr_mid->getCapturePtr()->getFramePtr() != nullptr   && "Middle feature's frame is nullptr!");
                assert(ftr_mid != ftr_first && "First and middle features are the same!");
                assert(ftr_mid != ftr_last  && "Last and middle features are the same!");

//                WOLF_TRACE("first ", ftr_first->id(), ", mid ", ftr_mid->id(), ", last ", ftr_last->id());
//                WOLF_TRACE("OLD first ", pair_trkid_match.second.first->id(), ", mid ", track_matrix_.feature(trk_id, origin_ptr_)->id(), ", last ", ftr_last->id());

                // make constraint
                ConstraintAutodiffTrifocalPtr ctr = std::make_shared<ConstraintAutodiffTrifocal>(ftr_first, ftr_mid, ftr_last, shared_from_this(), false, CTR_ACTIVE);

                // link to wolf tree
                ctr         ->  setFeaturePtr   (ftr_last);
                ftr_first   ->  addConstrainedBy(ctr);
                ftr_mid     ->  addConstrainedBy(ctr);
                ftr_last    ->  addConstraint   (ctr);
            }
        }
    }

//    WOLF_TRACE("===== END ESTABLISH CONSTRAINT =====");

}

void ProcessorTrackerFeatureTrifocal::setParams(const ProcessorParamsTrackerFeatureTrifocalPtr _params)
{
    params_tracker_feature_trifocal_ = _params;
}

void ProcessorTrackerFeatureTrifocal::configure(SensorBasePtr _sensor)
{
    _sensor->setNoiseStd(Vector2s::Ones() * params_tracker_feature_trifocal_->pixel_noise_std);
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
