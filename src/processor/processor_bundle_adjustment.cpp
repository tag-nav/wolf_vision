/*
 * processor_bundle_adjustment.cpp
 *
 *  Created on: May 3, 2019
 *      Author: ovendrell
 */

//wolf
#include "vision_utils/vision_utils.h"
#include "vision_utils/detectors.h"
#include "vision_utils/descriptors.h"
#include "vision_utils/matchers.h"

//standard
#include <memory>
#include "vision/processor/processor_bundle_adjustment.h"

namespace wolf{

ProcessorBundleAdjustment::ProcessorBundleAdjustment(ProcessorParamsBundleAdjustmentPtr _params_bundle_adjustment) :
                ProcessorTrackerFeature("TRACKER BUNDLE ADJUSTMENT", _params_bundle_adjustment),
                params_bundle_adjustment_(_params_bundle_adjustment),
                capture_image_last_(nullptr),
                capture_image_incoming_(nullptr)
{

	pixel_cov_ = Eigen::Matrix2s::Identity() * params_bundle_adjustment_->pixel_noise_std * params_bundle_adjustment_->pixel_noise_std;

    // Detector yaml file
    std::string det_name = vision_utils::readYamlType(params_bundle_adjustment_->yaml_file_params_vision_utils, "detector");
    // Create Detector
    det_ptr_ = vision_utils::setupDetector(det_name, det_name + " detector", params_bundle_adjustment_->yaml_file_params_vision_utils);

    // Descriptor yaml file
    std::string des_name = vision_utils::readYamlType(params_bundle_adjustment_->yaml_file_params_vision_utils, "descriptor");
    // Create Descriptor
    des_ptr_ = vision_utils::setupDescriptor(des_name, des_name + " descriptor", params_bundle_adjustment_->yaml_file_params_vision_utils);

    // Matcher yaml file
    std::string mat_name = vision_utils::readYamlType(params_bundle_adjustment_->yaml_file_params_vision_utils, "matcher");
    // Create Matcher
    mat_ptr_ = vision_utils::setupMatcher(mat_name, mat_name + " matcher", params_bundle_adjustment_->yaml_file_params_vision_utils);

}

void ProcessorBundleAdjustment::configure(SensorBasePtr _sensor)
{
    //TODO: Implement
}

void ProcessorBundleAdjustment::preProcess()
{
    // This method implements all Vision algorithms concerning OpenCV, so wolf algorithm only has to manage the data obtained
    // Get Capture
    capture_image_incoming_ = std::static_pointer_cast<CaptureImage>(incoming_ptr_);
    // Detect KeyPoints
    capture_image_incoming_->keypoints_ = det_ptr_->detect(capture_image_incoming_->getImage());
    // Compute Descriptors
    capture_image_incoming_->descriptors_ = des_ptr_->getDescriptor(capture_image_incoming_->getImage(), capture_image_incoming_->keypoints_);


    if (last_ptr_ != nullptr)
    {
        // Get Capture
        capture_image_last_ = std::static_pointer_cast<CaptureImage>(last_ptr_);
        // Match image
        capture_image_incoming_->matches_normalized_scores_ = mat_ptr_->robustMatch(capture_image_incoming_->keypoints_, capture_image_last_->keypoints_,
                                                                                    capture_image_incoming_->descriptors_, capture_image_last_->descriptors_,
                                                                                    des_ptr_->getSize(), capture_image_incoming_->matches_from_precedent_);
        //TODO: Get only the best ones
        if (params_bundle_adjustment_->delete_ambiguities) //filter ambiguities
        {
        	std::map<int,int> ambiguities_map;
        	for (auto match : capture_image_incoming_->matches_from_precedent_)
        		if (ambiguities_map.count(match.trainIdx))
        			ambiguities_map[match.trainIdx] = 0;
        		else
        			ambiguities_map[match.trainIdx] = 1;
        	// Set capture map of match indices
        	for (auto match : capture_image_incoming_->matches_from_precedent_)
        	{
        		if (!ambiguities_map.at(match.trainIdx))
        		capture_image_last_->map_index_to_next_[match.trainIdx] = match.queryIdx; // map[last] = incoming
        	}
        }
        else
        {
        	// Set capture map of match indices
        	for (auto match : capture_image_incoming_->matches_from_precedent_)
        		capture_image_last_->map_index_to_next_[match.trainIdx] = match.queryIdx; // map[last] = incoming

        }
    }
}

void ProcessorBundleAdjustment::postProcess()
{
	//TODO: visualization

    std::map<int,std::list<vision_utils::KeyPointEnhanced> > kp_enh_tracks;

    for (auto const & feat_base : last_ptr_->getFeatureList())
    {
        FeaturePointImagePtr feat = std::static_pointer_cast<FeaturePointImage>(feat_base);
        unsigned int feat_id = feat->id();
        unsigned int track_id = feat->trackId();

        // tracks
        std::list<vision_utils::KeyPointEnhanced> kp_enh_track;
        for (auto feat_base : track_matrix_.trackAsVector(track_id))
        {
            FeaturePointImagePtr feat_img = std::static_pointer_cast<FeaturePointImage>(feat_base);
            vision_utils::KeyPointEnhanced kp_enh(feat_img->getKeypoint(),
                                                  feat_id,
                                                  track_id,
                                                  track_matrix_.trackSize(track_id),
                                                  feat_img->getMeasurementCovariance());
            kp_enh_track.push_back(kp_enh);
        }

        kp_enh_tracks[feat_id] = kp_enh_track;
    }

    // DEBUG
    image_debug_ = vision_utils::buildImageProcessed((std::static_pointer_cast<CaptureImage>(last_ptr_))->getImage(), kp_enh_tracks);

    cv::imshow("DEBUG VIEW", image_debug_);
    cv::waitKey(1);
}


unsigned int ProcessorBundleAdjustment::trackFeatures(const FeatureBasePtrList& _features_last_in, FeatureBasePtrList& _features_incoming_out, FeatureMatchMap& _feature_correspondences)
{
    for (auto feat_base: _features_last_in)
    {
        FeaturePointImagePtr feat_last = std::static_pointer_cast<FeaturePointImage>(feat_base);

        if (capture_image_last_->map_index_to_next_.count(feat_last->getIndexKeyPoint())) //If the track exists
        {
            int index_inc = capture_image_last_->map_index_to_next_.at(feat_last->getIndexKeyPoint());

            if (capture_image_incoming_->matches_normalized_scores_.at(index_inc) > mat_ptr_->getParams()->min_norm_score )
            {
                // Get kp incoming and keypoint last
                cv::KeyPoint kp_inc = capture_image_incoming_->keypoints_.at(index_inc);
                //cv::KeyPoint kp_last = capture_image_last_->keypoints_.at(feat_last_->getIndexKeyPoint());

                FeaturePointImagePtr feat_inc = std::make_shared<FeaturePointImage>(kp_inc, index_inc,
                                                                                        capture_image_incoming_->descriptors_.row(index_inc),
                                                                                        pixel_cov_);

                _features_incoming_out.push_back(feat_inc);

                auto feature_match_ptr = std::make_shared<FeatureMatch>();
                feature_match_ptr->feature_ptr_= feat_last;
                feature_match_ptr->normalized_score_ = capture_image_incoming_->matches_normalized_scores_.at(index_inc);
                _feature_correspondences[feat_inc] = feature_match_ptr;

            }
        }
    }

    return _feature_correspondences.size();
}


bool ProcessorBundleAdjustment::is_tracked(int& _kp_idx)
{

    for (auto ftr : known_features_incoming_)
    {
        FeaturePointImagePtr ftr_kp = std::static_pointer_cast<FeaturePointImage>(ftr);
        if (ftr_kp->getIndexKeyPoint() == _kp_idx)
        {
            return true;
        }
    }
    return false;
}

unsigned int ProcessorBundleAdjustment::detectNewFeatures(const int& _max_new_features, FeatureBasePtrList& _features_last_out)
{
    //TODO: efficient implementation?

    typedef std::map<int,int>::iterator iter;

    for (iter it = capture_image_last_->map_index_to_next_.begin(); it!=capture_image_last_->map_index_to_next_.end(); ++it)
    {
        if (_features_last_out.size() >= _max_new_features)
        		break;

        else if(!is_tracked(it->second))
        {
            //add to _features_last_out
            int idx_last = it->first;
            cv::KeyPoint kp_last = capture_image_last_->keypoints_.at(idx_last);
            FeaturePointImagePtr feat_last_ = std::make_shared<FeaturePointImage>(kp_last, idx_last,
                                                                                 capture_image_last_->descriptors_.row(idx_last),
                                                                                 pixel_cov_);
            _features_last_out.push_back(feat_last_);

        }
    }

    return _features_last_out.size();
}


bool ProcessorBundleAdjustment::correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature, FeatureBasePtr _incoming_feature)
{
    //TODO: Implement
    return false;
}

bool ProcessorBundleAdjustment::voteForKeyFrame()
{
	//    // A. crossing voting threshold with ascending number of features
	    bool vote_up = false;
	//    // 1. vote if we did not have enough features before
	//    vote_up = vote_up && (previousNumberOfTracks() < params_bundle_adjustment_->min_features_for_keyframe);
	//    // 2. vote if we have enough features now
	//    vote_up = vote_up && (incoming_ptr_->getFeatureList().size() >= params_bundle_adjustment_->min_features_for_keyframe);

	    // B. crossing voting threshold with descending number of features
	    bool vote_down = true;
	    // 1. vote if we had enough features before
	//    vote_down = vote_down && (last_ptr_->getFeatureList().size() >= params_bundle_adjustment_->min_features_for_keyframe);
	    // 2. vote if we have not enough features now
	    vote_down = vote_down && (incoming_ptr_->getFeatureList().size() < params_bundle_adjustment_->min_features_for_keyframe);

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


FactorBasePtr ProcessorBundleAdjustment::createFactor(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr)
{
    /* This function is no creating any factor.
     *  We create factors with establishFactors()
     */
    return FactorBasePtr();
}

LandmarkBasePtr ProcessorBundleAdjustment::createLandmark(FeatureBasePtr _feature_ptr)
{

    FeaturePointImagePtr feat_point_image_ptr = std::static_pointer_cast<FeaturePointImage>( _feature_ptr);
    //FrameBasePtr anchor_frame = getLast()->getFrame();

    Eigen::Vector2s point2D;
    point2D[0] = feat_point_image_ptr->getKeypoint().pt.x;
    point2D[1] = feat_point_image_ptr->getKeypoint().pt.y;

    //Scalar distance = params_bundle_adjustment_->distance; // arbitrary value
    Eigen::Vector4s vec_homogeneous;

    point2D = pinhole::depixellizePoint(getSensor()->getIntrinsic()->getState(),point2D);
    point2D = pinhole::undistortPoint((std::static_pointer_cast<SensorCamera>(getSensor()))->getCorrectionVector(),point2D);

    Eigen::Vector3s point3D;
    point3D.head<2>() = point2D;
    point3D(2) = 1;

    point3D.normalize();

    vec_homogeneous = {point3D(0),point3D(1),point3D(2),1/1};//distance};

    LandmarkHPPtr lmk_hp_ptr = std::make_shared<LandmarkHP>(vec_homogeneous, getSensor(), feat_point_image_ptr->getDescriptor());
    _feature_ptr->setLandmarkId(lmk_hp_ptr->id());
    return lmk_hp_ptr;
}

void ProcessorBundleAdjustment::establishFactors()
{

	TrackMatches matches_origin_inc = track_matrix_.matches(origin_ptr_, incoming_ptr_);

    for (auto const & pair_trkid_pair : matches_origin_inc)
    {
        size_t trkid = pair_trkid_pair.first;
        FeatureBasePtr feature_origin = pair_trkid_pair.second.first;
        FeatureBasePtr feature_inc   = pair_trkid_pair.second.second;

        //TODO: don't use map[track_id] = landmarkptr.
        if (lmk_track_map_.count(trkid)==0) //if the track doesn't have landmark associated -> we need a map: map[track_id] = landmarkptr
        {
        	//createLandmark
        	LandmarkBasePtr lmk = createLandmark(feature_origin);
        	LandmarkHPPtr lmk_hp = std::static_pointer_cast<LandmarkHP>(lmk);
        	//add Landmark to map: map[track_id] = landmarkptr
        	lmk_track_map_[trkid] = lmk;

        	//create factors
        	FactorPixelHPPtr fac_ptr_origin = std::make_shared<FactorPixelHP>(feature_origin, lmk_hp, shared_from_this());
            if (fac_ptr_origin != nullptr) // factor links
            {
            	 feature_origin->addFactor(fac_ptr_origin);
            	 lmk->addConstrainedBy(fac_ptr_origin);
            }
        	FactorPixelHPPtr fac_ptr_inc = std::make_shared<FactorPixelHP>(feature_inc, lmk_hp, shared_from_this());
            if (fac_ptr_inc != nullptr) // factor links
            {
            	 feature_inc->addFactor(fac_ptr_inc);
            	 lmk->addConstrainedBy(fac_ptr_inc);
            }
        }
        else
        {
        	LandmarkBasePtr lmk = lmk_track_map_[trkid];
        	LandmarkHPPtr lmk_hp = std::static_pointer_cast<LandmarkHP>(lmk);

        	//Create factor
        	FactorPixelHPPtr fac_ptr_inc = std::make_shared<FactorPixelHP>(feature_inc, lmk_hp, shared_from_this());
            if (fac_ptr_inc != nullptr) // factor links
            {
            	 feature_inc->addFactor(fac_ptr_inc);
            	 lmk->addConstrainedBy(fac_ptr_inc);
            }
        }

    }
}



ProcessorBasePtr ProcessorBundleAdjustment::create(const std::string& _unique_name,
                                                         const ProcessorParamsBasePtr _params,
                                                         const SensorBasePtr _sensor_ptr)
{
  const auto params = std::static_pointer_cast<ProcessorParamsBundleAdjustment>(_params);

  ProcessorBasePtr prc_ptr = std::make_shared<ProcessorBundleAdjustment>(params);
  prc_ptr->setName(_unique_name);
  prc_ptr->configure(_sensor_ptr);
  return prc_ptr;
}

} //namespace wolf

// Register in the ProcessorFactory
#include "core/processor/processor_factory.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR("TRACKER BUNDLE ADJUSTMENT", ProcessorBundleAdjustment)
} // namespace wolf

