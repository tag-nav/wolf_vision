/*
 * processor_tracker_feature_oriol.cpp
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

ProcessorBundleAdjustment::ProcessorBundleAdjustment(ProcessorParamsBundleAdjustmentPtr _params_tracker_feature_oriol) :
                ProcessorTrackerFeature("TRACKER BUNDLE ADJUSTMENT", _params_tracker_feature_oriol),
                params_tracker_feature_oriol_(_params_tracker_feature_oriol),
                capture_image_last_(nullptr),
                capture_image_incoming_(nullptr)
{

    // Detector yaml file
    std::string det_name = vision_utils::readYamlType(params_tracker_feature_oriol_->yaml_file_params_vision_utils, "detector");
    // Create Detector
    det_ptr_ = vision_utils::setupDetector(det_name, det_name + " detector", params_tracker_feature_oriol_->yaml_file_params_vision_utils);

    // Descriptor yaml file
    std::string des_name = vision_utils::readYamlType(params_tracker_feature_oriol_->yaml_file_params_vision_utils, "descriptor");
    // Create Descriptor
    des_ptr_ = vision_utils::setupDescriptor(des_name, des_name + " descriptor", params_tracker_feature_oriol_->yaml_file_params_vision_utils);

    // Matcher yaml file
    std::string mat_name = vision_utils::readYamlType(params_tracker_feature_oriol_->yaml_file_params_vision_utils, "matcher");
    // Create Matcher
    mat_ptr_ = vision_utils::setupMatcher(mat_name, mat_name + " matcher", params_tracker_feature_oriol_->yaml_file_params_vision_utils);

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
        // Set capture map of match indices
        for (auto match : capture_image_incoming_->matches_from_precedent_)
        {
            capture_image_last_->map_index_to_next_[match.trainIdx] = match.queryIdx; // map[last] = incoming
            //TODO: Nomes es queda l'ultim si hi ha ambiguitat?
        }
    }
}

void ProcessorBundleAdjustment::postProcess()
{

}


unsigned int ProcessorBundleAdjustment::trackFeatures(const FeatureBasePtrList& _features_last_in, FeatureBasePtrList& _features_incoming_out, FeatureMatchMap& _feature_correspondences)
{
    for (auto feat_base: _features_last_in)
    {
        FeaturePointImagePtr feat_last_ = std::static_pointer_cast<FeaturePointImage>(feat_base);

        if (capture_image_last_->map_index_to_next_.count(feat_last_->getIndexKeyPoint())) //If the track exists
        {
            int index_inc = capture_image_last_->map_index_to_next_.at(feat_last_->getIndexKeyPoint());

            if (capture_image_incoming_->matches_normalized_scores_.at(index_inc) > mat_ptr_->getParams()->min_norm_score )
            {
                // Get kp incoming and keypoint last
                cv::KeyPoint kp_inc = capture_image_incoming_->keypoints_.at(index_inc);
                //cv::KeyPoint kp_last = capture_image_last_->keypoints_.at(feat_last_->getIndexKeyPoint());

                FeaturePointImagePtr feat_inc_ = std::make_shared<FeaturePointImage>(kp_inc, index_inc,
                                                                                        capture_image_incoming_->descriptors_.row(index_inc),
                                                                                        pixel_cov_);

                _features_incoming_out.push_back(feat_inc_);

                //TODO: Check ambiguities?? Aqui no

                _feature_correspondences[feat_inc_] = std::make_shared<FeatureMatch>(feat_last_,
                                                                                     capture_image_incoming_->matches_normalized_scores_.at(index_inc));
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
    /*TODO: Implement
     * Detect features again : NO fa falta
     * Filter to get only the new features compared to the ones we had tracked before
     * \param _features_last_out is new_last_features que s'ha d'omplir
     *  tenir en compte que es fa trackfeatures amb el resultat i no s'han de perdre els tracks ja obtinguts (i si hi ha indexs duplicats?)
     */

    typedef std::map<int,int>::iterator iter;

    for (iter it = capture_image_last_->map_index_to_next_.begin(); it!=capture_image_last_->map_index_to_next_.end(); ++it)
    {
        if(!is_tracked(it->second))
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


bool correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature, FeatureBasePtr _incoming_feature)
{
    //TODO: Implement
    return false;
}

bool voteForKeyFrame()
{
    //TODO: Implement
    return false;
}

FactorBasePtr createFactor(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr)
{
    //TODO: Implement
    FactorBasePtr a;
    return a;
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

