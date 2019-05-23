// Wolf includes
#include "vision/processor/processor_tracker_feature_image.h"

// Vision utils
#include <detectors.h>
#include <descriptors.h>
#include <matchers.h>
#include <algorithms.h>

// standard libs
#include <bitset>
#include <algorithm>

namespace wolf
{

ProcessorTrackerFeatureImage::ProcessorTrackerFeatureImage(ProcessorParamsTrackerFeatureImagePtr _params_tracker_feature_image) :
    ProcessorTrackerFeature("IMAGE", _params_tracker_feature_image),
    cell_width_(0), cell_height_(0),  // These will be initialized to proper values taken from the sensor via function configure()
    params_tracker_feature_image_(_params_tracker_feature_image)
{
	// Detector
    std::string det_name = vision_utils::readYamlType(params_tracker_feature_image_->yaml_file_params_vision_utils, "detector");
    det_ptr_ = vision_utils::setupDetector(det_name, det_name + " detector", params_tracker_feature_image_->yaml_file_params_vision_utils);

    // Descriptor
    std::string des_name = vision_utils::readYamlType(params_tracker_feature_image_->yaml_file_params_vision_utils, "descriptor");
    des_ptr_ = vision_utils::setupDescriptor(des_name, des_name + " descriptor", params_tracker_feature_image_->yaml_file_params_vision_utils);

    // Matcher
    std::string mat_name = vision_utils::readYamlType(params_tracker_feature_image_->yaml_file_params_vision_utils, "matcher");
    mat_ptr_ = vision_utils::setupMatcher(mat_name, mat_name + " matcher", params_tracker_feature_image_->yaml_file_params_vision_utils);

    // Active search grid
    vision_utils::AlgorithmBasePtr alg_ptr = vision_utils::setupAlgorithm("ACTIVESEARCH", "ACTIVESEARCH algorithm", params_tracker_feature_image_->yaml_file_params_vision_utils);
    active_search_ptr_ = std::static_pointer_cast<vision_utils::AlgorithmACTIVESEARCH>(alg_ptr);
}

//Destructor
ProcessorTrackerFeatureImage::~ProcessorTrackerFeatureImage()
{
    //
}

void ProcessorTrackerFeatureImage::configure(SensorBasePtr _sensor)
{
    SensorCameraPtr camera = std::static_pointer_cast<SensorCamera>(_sensor);

    image_.width_ = camera->getImgWidth();
    image_.height_ = camera->getImgHeight();

    active_search_ptr_->initAlg(camera->getImgWidth(), camera->getImgHeight() , det_ptr_->getPatternRadius());

    params_tracker_feature_image_activesearch_ptr_ = std::static_pointer_cast<vision_utils::AlgorithmParamsACTIVESEARCH>( active_search_ptr_->getParams() );

    cell_width_ = image_.width_ / params_tracker_feature_image_activesearch_ptr_->n_cells_h;
    cell_height_ = image_.height_ / params_tracker_feature_image_activesearch_ptr_->n_cells_v;
}

void ProcessorTrackerFeatureImage::preProcess()
{
    image_incoming_ = std::static_pointer_cast<CaptureImage>(incoming_ptr_)->getImage();

    active_search_ptr_->renew();

    //The visualization functions and variables
    if(last_ptr_ != nullptr)
        resetVisualizationFlag(last_ptr_->getFeatureList());

    tracker_roi_.clear();
    detector_roi_.clear();
    tracker_target_.clear();
}

void ProcessorTrackerFeatureImage::postProcess()
{
}

unsigned int ProcessorTrackerFeatureImage::trackFeatures(const FeatureBasePtrList& _features_last_in, FeatureBasePtrList& _features_incoming_out,
                                           FeatureMatchMap& _feature_matches)
{
    KeyPointVector candidate_keypoints;
    cv::Mat candidate_descriptors;
    DMatchVector cv_matches;

    for (auto feature_base_ptr : _features_last_in)
    {
        FeaturePointImagePtr feature_ptr = std::static_pointer_cast<FeaturePointImage>(feature_base_ptr);

        cv::Rect roi = vision_utils::setRoi(feature_ptr->getKeypoint().pt.x, feature_ptr->getKeypoint().pt.y, cell_width_, cell_height_);

        active_search_ptr_->hitCell(feature_ptr->getKeypoint());

        cv::Mat target_descriptor = feature_ptr->getDescriptor();

        //lists used to debug
        tracker_target_.push_back(feature_ptr->getKeypoint().pt);
        tracker_roi_.push_back(roi);

        if (detect(image_incoming_, roi, candidate_keypoints, candidate_descriptors))
        {
            Scalar normalized_score = match(target_descriptor,candidate_descriptors,cv_matches);
            if ( normalized_score > mat_ptr_->getParams()->min_norm_score )
            {
                FeaturePointImagePtr incoming_point_ptr = std::make_shared<FeaturePointImage>(
                        candidate_keypoints[cv_matches[0].trainIdx],
                        cv_matches[0].trainIdx,
                        (candidate_descriptors.row(cv_matches[0].trainIdx)),
                        Eigen::Matrix2s::Identity()*params_tracker_feature_image_->pixel_noise_var);
                incoming_point_ptr->setIsKnown(feature_ptr->isKnown());
                _features_incoming_out.push_back(incoming_point_ptr);

                _feature_matches[incoming_point_ptr] = std::make_shared<FeatureMatch>(FeatureMatch({feature_base_ptr, normalized_score}));
            }
            else
            {
                //lists used to debug
                tracker_target_.pop_back();
                tracker_roi_.pop_back();
            }
        }
        else
        {
           //lists used to debug
            tracker_target_.pop_back();
            tracker_roi_.pop_back();
        }
    }
//    std::cout << "TrackFeatures - Number of Features tracked: " << _features_incoming_out.size() << std::endl;
    return _features_incoming_out.size();
}

bool ProcessorTrackerFeatureImage::correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature, FeatureBasePtr _incoming_feature)
{
    DMatchVector matches_mat;
    FeaturePointImagePtr feat_incoming_ptr = std::static_pointer_cast<FeaturePointImage>(_incoming_feature);
    FeaturePointImagePtr feat_origin_ptr = std::static_pointer_cast<FeaturePointImage>(_origin_feature);

    cv::Mat origin_descriptor = feat_origin_ptr->getDescriptor();
    cv::Mat incoming_descriptor = feat_incoming_ptr->getDescriptor();

    KeyPointVector origin_keypoint;
    origin_keypoint.push_back(feat_origin_ptr->getKeypoint());

    Scalar normalized_score = match(origin_descriptor,incoming_descriptor,matches_mat);

    if(normalized_score > mat_ptr_->getParams()->min_norm_score)
        return true;
    else
    {
        /* CORRECT */

        unsigned int roi_width = cell_width_;
        unsigned int roi_heigth = cell_height_;
        unsigned int roi_x;
        unsigned int roi_y;

        KeyPointVector correction_keypoints;
        cv::Mat correction_descriptors;
        DMatchVector correction_matches;

        FeaturePointImagePtr feat_last_ptr = std::static_pointer_cast<FeaturePointImage>(_last_feature);

        active_search_ptr_->hitCell(feat_last_ptr->getKeypoint());

        roi_x = (feat_last_ptr->getKeypoint().pt.x) - (roi_heigth / 2);
        roi_y = (feat_last_ptr->getKeypoint().pt.y) - (roi_width / 2);
        cv::Rect roi(roi_x, roi_y, roi_width, roi_heigth);

        if (detect(image_incoming_, roi, correction_keypoints, correction_descriptors))
        {
            Scalar normalized_score_correction = match(origin_descriptor,correction_descriptors,correction_matches);
            if(normalized_score_correction > mat_ptr_->getParams()->min_norm_score )
            {
                feat_incoming_ptr->setKeypoint(correction_keypoints[correction_matches[0].trainIdx]);
                feat_incoming_ptr->setDescriptor(correction_descriptors.row(correction_matches[0].trainIdx));
                return true;
            }
            else
            {
                return false;
            }
        }
        return false;
    }
}

unsigned int ProcessorTrackerFeatureImage::detectNewFeatures(const int& _max_new_features, FeatureBasePtrList& _features_last_out)
{
    cv::Rect roi;
    KeyPointVector new_keypoints;
    cv::Mat new_descriptors;
    cv::KeyPointsFilter keypoint_filter;
    unsigned int n_new_features = 0;

    for (unsigned int n_iterations = 0; _max_new_features == -1 || n_iterations < _max_new_features; n_iterations++)
    {

        if (active_search_ptr_->pickEmptyRoi(roi))
        {
            detector_roi_.push_back(roi);
            if (detect(image_last_, roi, new_keypoints, new_descriptors))
            {
                KeyPointVector list_keypoints = new_keypoints;
                unsigned int index = 0;
                keypoint_filter.retainBest(new_keypoints,1);
                for(unsigned int i = 0; i < list_keypoints.size(); i++)
                {
                    if(list_keypoints[i].pt == new_keypoints[0].pt)
                        index = i;
                }
                if(new_keypoints[0].response > params_tracker_feature_image_activesearch_ptr_->min_response_new_feature)
                {
                    std::cout << "response: " << new_keypoints[0].response << std::endl;
                    FeaturePointImagePtr point_ptr = std::make_shared<FeaturePointImage>(
                            new_keypoints[0],
                            0,
                            new_descriptors.row(index),
                            Eigen::Matrix2s::Identity()*params_tracker_feature_image_->pixel_noise_var);
                    point_ptr->setIsKnown(false);
                    _features_last_out.push_back(point_ptr);

                    active_search_ptr_->hitCell(new_keypoints[0]);

                    n_new_features++;
                }
            }
            else
            {
                active_search_ptr_->blockCell(roi);
            }
        }
        else
            break;
    }

    WOLF_DEBUG( "DetectNewFeatures - Number of new features detected: " , n_new_features );
    return n_new_features;
}

//============================================================

Scalar ProcessorTrackerFeatureImage::match(cv::Mat _target_descriptor, cv::Mat _candidate_descriptors, DMatchVector& _cv_matches)
{
    mat_ptr_->match(_target_descriptor, _candidate_descriptors, des_ptr_->getSize(), _cv_matches);
    Scalar normalized_score = 1 - (Scalar)(_cv_matches[0].distance)/(des_ptr_->getSize()*8);
    return normalized_score;
}

FactorBasePtr ProcessorTrackerFeatureImage::createFactor(FeatureBasePtr _feature_ptr,
                                                          FeatureBasePtr _feature_other_ptr)
{
    FactorFeatureDummyPtr const_dummy_ptr = std::make_shared<FactorFeatureDummy>(_feature_ptr, _feature_other_ptr,
                                                                                    shared_from_this());
    //    _feature_ptr->addFactor(const_epipolar_ptr);
    //    _feature_other_ptr->addConstrainedBy(const_epipolar_ptr);
    return const_dummy_ptr;
}

unsigned int ProcessorTrackerFeatureImage::detect(cv::Mat _image, cv::Rect& _roi, std::vector<cv::KeyPoint>& _new_keypoints,
                                    cv::Mat& _new_descriptors)
{
    _new_keypoints = det_ptr_->detect(_image, _roi);
    _new_descriptors = des_ptr_->getDescriptor(_image, _new_keypoints);
    return _new_keypoints.size();
}

void ProcessorTrackerFeatureImage::resetVisualizationFlag(FeatureBasePtrList& _feature_list_last)
{
    for (auto feature_base_last_ptr : _feature_list_last)
    {
        FeaturePointImagePtr feature_last_ptr = std::static_pointer_cast<FeaturePointImage>(feature_base_last_ptr);
        feature_last_ptr->setIsKnown(true);
    }
}

// draw functions ===================================================================

void ProcessorTrackerFeatureImage::drawTarget(cv::Mat _image, std::list<cv::Point> _target_list)
{
    if (last_ptr_!=nullptr)
    {
        // draw the target of the tracking
        for(auto target_point : _target_list)
            cv::circle(_image, target_point, 7, cv::Scalar(255.0, 0.0, 255.0), 1, 3, 0);
    }
}

void ProcessorTrackerFeatureImage::drawRoi(cv::Mat _image, std::list<cv::Rect> _roi_list, cv::Scalar _color)
{
    if (last_ptr_!=nullptr)
    {
        for (auto roi : _roi_list)
            cv::rectangle(_image, roi, _color, 1, 8, 0);
    }
}

void ProcessorTrackerFeatureImage::drawFeatures(cv::Mat _image)
{
    if (last_ptr_!=nullptr)
    {
        unsigned int known_feature_counter = 0;
        unsigned int new_feature_counter = 0;

        for (auto feature_ptr : last_ptr_->getFeatureList())
        {
            FeaturePointImagePtr point_ptr = std::static_pointer_cast<FeaturePointImage>(feature_ptr);
            if (point_ptr->isKnown())
            {
                cv::circle(_image, point_ptr->getKeypoint().pt, 4, cv::Scalar(51.0, 255.0, 51.0), -1, 3, 0);
                known_feature_counter++;
            }
            else
            {
                cv::circle(_image, point_ptr->getKeypoint().pt, 4, cv::Scalar(0.0, 0.0, 255.0), -1, 3, 0);
                new_feature_counter++;
            }

            cv::putText(_image, std::to_string(feature_ptr->trackId()), point_ptr->getKeypoint().pt,
                        cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 255.0, 0.0));
        }
        std::cout << "\nKnown: " << known_feature_counter << "\tNew: " << new_feature_counter << std::endl;
    }
}

ProcessorBasePtr ProcessorTrackerFeatureImage::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr _sensor_ptr)
{
    ProcessorTrackerFeatureImagePtr prc_ptr = std::make_shared<ProcessorTrackerFeatureImage>(std::static_pointer_cast<ProcessorParamsTrackerFeatureImage>(_params));
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

} // namespace wolf

// Register in the SensorFactory
#include "core/processor/processor_factory.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR("IMAGE FEATURE", ProcessorTrackerFeatureImage)
} // namespace wolf

