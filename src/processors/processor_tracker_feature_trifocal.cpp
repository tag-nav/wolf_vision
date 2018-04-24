
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

// Constructor
ProcessorTrackerFeatureTrifocal::ProcessorTrackerFeatureTrifocal(ProcessorParamsTrackerFeatureTrifocalPtr _params_tracker_feature_trifocal) :
        ProcessorTrackerFeature("TRACKER FEATURE TRIFOCAL", _params_tracker_feature_trifocal ),
        cell_width_(0), cell_height_(0), // These will be initialized to proper values taken from the sensor via function configure()
        params_tracker_feature_trifocal_(_params_tracker_feature_trifocal),
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

    // Active search grid
    vision_utils::AlgorithmBasePtr alg_ptr = vision_utils::setupAlgorithm("ACTIVESEARCH", "ACTIVESEARCH algorithm", params_tracker_feature_trifocal_->yaml_file_params_vision_utils);
    active_search_ptr_ = std::static_pointer_cast<vision_utils::AlgorithmACTIVESEARCH>(alg_ptr);

//    // DEBUG VIEW
//    cv::startWindowThread();
//    cv::namedWindow("DEBUG VIEW", cv::WINDOW_NORMAL);
}

// Destructor
ProcessorTrackerFeatureTrifocal::~ProcessorTrackerFeatureTrifocal()
{
//    cv::destroyAllWindows();
}

unsigned int ProcessorTrackerFeatureTrifocal::detectNewFeatures(const unsigned int& _max_new_features, FeatureBaseList& _feature_list_out)
{
    unsigned int n_new_features = 0;

    KeyPointVector kps;
    cv::Mat desc;
    cv::Rect roi;
    KeyPointVector new_keypoints;
    cv::Mat new_descriptors;
    cv::KeyPointsFilter keypoint_filter;

    // Debug
    int feat_det = 0;
    int roi_empty = 0;
    int non_good_features = 0;

    for (unsigned int n_iterations = 0; _max_new_features == 0 || n_iterations < _max_new_features; n_iterations++)
    {
        if (active_search_ptr_->pickEmptyRoi(roi))
        {
            // Debug ROI
            cv::Rect roi_original = roi;

            kps  = det_ptr_->detect       (image_last_, roi);
            desc = des_ptr_->getDescriptor(image_last_, kps);

            if (kps.size() > 0)
            {
                feat_det++;

                KeyPointVector list_keypoints = kps;
                unsigned int index = 0;
                keypoint_filter.retainBest(kps,1);
                for(unsigned int i = 0; i < list_keypoints.size(); i++)
                {
                    if(list_keypoints[i].pt == kps[0].pt)
                        index = i;
                }

                if(kps[0].response > params_tracker_feature_trifocal_activesearch_ptr_->min_response_new_feature)
                {
//                    std::cout << "response: " << kps[0].response << std::endl;
                    FeaturePointImagePtr point = std::make_shared<FeaturePointImage>(
                            kps[0],
                            desc.row(index),
                            Eigen::Matrix2s::Identity() * params_tracker_feature_trifocal_->pixel_noise_std * params_tracker_feature_trifocal_->pixel_noise_std);
                    point->setIsKnown(true);
                    _feature_list_out.push_back(point);

                    active_search_ptr_->hitCell(kps[0]);

                    // Debug ROI
                    vision_utils::ROIEnhanced roi_e(roi_original,true);
                    debug_roi_enh_.push_back(roi_e);

                    n_new_features++;
                }
                else
                {
                    active_search_ptr_->blockCell(roi);

                    // Debug ROI
                    vision_utils::ROIEnhanced roi_e(roi_original,false);
                    debug_roi_enh_.push_back(roi_e);

                    non_good_features++;
                }
            }
            else
            {
                roi_empty++;
                active_search_ptr_->blockCell(roi);

                // Debug ROI
                vision_utils::ROIEnhanced roi_e(roi_original,false);
                debug_roi_enh_.push_back(roi_e);
            }
        }
        else
        {
            WOLF_TRACE("Cannot pick roi. n_iterations: ", n_iterations);
            break;
        }
    }

    WOLF_TRACE("Empty ROIs: ",roi_empty);
    WOLF_TRACE("NUM Detected Features: ",feat_det);
    WOLF_TRACE("Non Good features: ",non_good_features);
    WOLF_TRACE("DetectNewFeatures - Number of new features detected: " , n_new_features );

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
    KeyPointVector kps;
    cv::Mat desc;

    for (auto feature_base_ptr : _feature_list_in)
    {
        // Feature and descriptor
        FeaturePointImagePtr target_feature = std::static_pointer_cast<FeaturePointImage>(feature_base_ptr);
        cv::Mat target_descriptor = target_feature->getDescriptor();

        // Hit cell
        active_search_ptr_->hitCell(target_feature->getKeypoint());

        // Set ROI
        cv::Rect roi = vision_utils::setRoi(target_feature->getKeypoint().pt.x, target_feature->getKeypoint().pt.y, cell_width_, cell_height_);

        // Detect features in ROI
        kps = det_ptr_->detect(image_incoming_, roi);

        if (kps.size() > 0)
        {
            // New descriptors in ROI
            desc = des_ptr_->getDescriptor(image_incoming_, kps);

            // Match
            cv::DMatch cv_match;
            Scalar normalized_score = mat_ptr_->match(target_descriptor, desc, des_ptr_->getSize(), cv_match);

            if ( normalized_score > mat_ptr_->getParams()->min_norm_score )
            {
                cv::KeyPoint tracked_kp = kps[cv_match.trainIdx];
                cv::Mat tracked_desc = desc(cv::Rect(0,cv_match.trainIdx,desc.cols,1));

                FeaturePointImagePtr incoming_point_ptr = std::make_shared<FeaturePointImage>(
                        tracked_kp, tracked_desc,
                        Eigen::Matrix2s::Identity() * params_tracker_feature_trifocal_->pixel_noise_std * params_tracker_feature_trifocal_->pixel_noise_std);

                incoming_point_ptr->setIsKnown(target_feature->isKnown());

                _feature_list_out.push_back(incoming_point_ptr);

                _feature_matches[incoming_point_ptr] = std::make_shared<FeatureMatch>(FeatureMatch({feature_base_ptr, normalized_score}));
            }
        }
    }

    return _feature_list_out.size();
}

void ProcessorTrackerFeatureTrifocal::advanceDerived()
{
    ProcessorTrackerFeature::advanceDerived();
    image_last_ = image_incoming_;
}

void ProcessorTrackerFeatureTrifocal::resetDerived()
{
    // Call parent class method
    ProcessorTrackerFeature::resetDerived();

    image_last_ = image_incoming_;

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

    image_incoming_ = std::static_pointer_cast<CaptureImage>(incoming_ptr_)->getImage();

    active_search_ptr_->renew();

    //The visualization functions and variables
    debug_roi_enh_.clear();
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

    cv::Mat img = (std::static_pointer_cast<CaptureImage>(last_ptr_))->getImage();
    cv::Mat img_proc = vision_utils::buildImageProcessed(img, kps_e, debug_roi_enh_);

//    cv::imshow("DEBUG VIEW", img_proc);
//    cv::waitKey(1);
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

    active_search_ptr_->initAlg(camera->getImgWidth(), camera->getImgHeight() , det_ptr_->getPatternRadius());

    params_tracker_feature_trifocal_activesearch_ptr_ = std::static_pointer_cast<vision_utils::AlgorithmParamsACTIVESEARCH>( active_search_ptr_->getParams() );

    cell_width_  = camera->getImgWidth()  / params_tracker_feature_trifocal_activesearch_ptr_->n_cells_h;
    cell_height_ = camera->getImgHeight() / params_tracker_feature_trifocal_activesearch_ptr_->n_cells_v;
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
