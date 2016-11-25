// Wolf includes
#include "processor_image_feature.h"

// OpenCV includes

// other includes
#include <bitset>
#include <algorithm>

namespace wolf
{

ProcessorImageFeature::ProcessorImageFeature(ProcessorParamsImage _params) :
    ProcessorTrackerFeature("IMAGE", _params.algorithm.max_new_features),
    matcher_ptr_(nullptr),
    detector_descriptor_ptr_(nullptr),
    params_(_params),
    active_search_grid_()
{
    // 1. detector-descriptor params
    DetectorDescriptorParamsBasePtr _dd_params = _params.detector_descriptor_params_ptr;
    switch (_dd_params->type){
        case DD_BRISK:
            {
            std::shared_ptr<DetectorDescriptorParamsBrisk> params_brisk = std::static_pointer_cast<DetectorDescriptorParamsBrisk>(_dd_params);
            detector_descriptor_ptr_ = std::make_shared<cv::BRISK>(params_brisk->threshold, //
                                                     params_brisk->octaves, //
                                                     params_brisk->pattern_scale);

            detector_descriptor_params_.pattern_radius_ = std::max((unsigned int)((params_brisk->nominal_pattern_radius)*pow(2,params_brisk->octaves)),
                                                                   (unsigned int)((params_brisk->nominal_pattern_radius)*params_brisk->pattern_scale));
            detector_descriptor_params_.size_bits_ = detector_descriptor_ptr_->descriptorSize() * 8;

            break;
            }
        case DD_ORB:
            {
            std::shared_ptr<DetectorDescriptorParamsOrb> params_orb = std::static_pointer_cast<DetectorDescriptorParamsOrb>(_dd_params);
            detector_descriptor_ptr_ = std::make_shared<cv::ORB>(params_orb->nfeatures, //
                                                   params_orb->scaleFactor, //
                                                   params_orb->nlevels, //
                                                   params_orb->edgeThreshold, //
                                                   params_orb->firstLevel, //
                                                   params_orb->WTA_K, //
                                                   params_orb->scoreType, //
                                                   params_orb->patchSize);

            detector_descriptor_params_.pattern_radius_ = params_orb->edgeThreshold;
            detector_descriptor_params_.size_bits_ = detector_descriptor_ptr_->descriptorSize() * 8;

            break;
            }
        default:
            throw std::runtime_error("Unknown detector-descriptor type");
    }

    // 2. matcher params
    matcher_ptr_ = std::make_shared<cv::BFMatcher>(_params.matcher.similarity_norm);

}

//Destructor
ProcessorImageFeature::~ProcessorImageFeature()
{
    //
}

void ProcessorImageFeature::setup(SensorCameraPtr _camera_ptr)
{
    image_.width_ = _camera_ptr->getImgWidth();
    image_.height_ = _camera_ptr->getImgHeight();

    active_search_grid_.setup(image_.width_,image_.height_,
            params_.active_search.grid_width, params_.active_search.grid_height,
            detector_descriptor_params_.pattern_radius_,
            params_.active_search.separation);
}

void ProcessorImageFeature::preProcess()
{
    image_incoming_ = std::static_pointer_cast<CaptureImage>(incoming_ptr_)->getImage();
    active_search_grid_.renew();


    //The visualization functions and variables
    if(last_ptr_ != nullptr)
        resetVisualizationFlag(last_ptr_->getFeatureList());

    tracker_roi_.clear();
    detector_roi_.clear();
    tracker_target_.clear();
}

void ProcessorImageFeature::postProcess()
{
    if (last_ptr_!=nullptr)
    {
        cv::Mat image = image_last_.clone();
        if(params_.draw.primary_drawing) drawFeatures(image);
        if(params_.draw.detector_roi) drawRoi(image,detector_roi_,cv::Scalar(0.0,255.0, 255.0));   //active search roi
        if(params_.draw.tracker_roi) drawRoi(image,tracker_roi_, cv::Scalar(255.0, 0.0, 255.0));  //tracker roi
        if(params_.draw.secondary_drawing) drawTarget(image,tracker_target_);
    }
}

unsigned int ProcessorImageFeature::trackFeatures(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out,
                                           FeatureMatchMap& _feature_matches)
{

    unsigned int roi_width = params_.matcher.roi_width;
    unsigned int roi_heigth = params_.matcher.roi_height;
    unsigned int roi_x;
    unsigned int roi_y;

    std::vector<cv::KeyPoint> candidate_keypoints;
    cv::Mat candidate_descriptors;
    std::vector<cv::DMatch> cv_matches;

    for (auto feature_base_ptr : _feature_list_in)
    {
        FeaturePointImagePtr feature_ptr = std::static_pointer_cast<FeaturePointImage>(feature_base_ptr);

        roi_x = (feature_ptr->getKeypoint().pt.x) - (roi_heigth / 2);
        roi_y = (feature_ptr->getKeypoint().pt.y) - (roi_width / 2);
        cv::Rect roi(roi_x, roi_y, roi_width, roi_heigth);

        active_search_grid_.hitCell(feature_ptr->getKeypoint());
        complete_target_size_++;
        cv::Mat target_descriptor = feature_ptr->getDescriptor();

        //lists used to debug
        tracker_target_.push_back(feature_ptr->getKeypoint().pt);
        tracker_roi_.push_back(roi);

        if (detect(image_incoming_, roi, candidate_keypoints, candidate_descriptors))
        {
            Scalar normalized_score = match(target_descriptor,candidate_descriptors,cv_matches);
            if (normalized_score > params_.matcher.min_normalized_score)
            {
                FeaturePointImagePtr incoming_point_ptr = std::make_shared<FeaturePointImage>(
                        candidate_keypoints[cv_matches[0].trainIdx], (candidate_descriptors.row(cv_matches[0].trainIdx)),
                        Eigen::Matrix2s::Identity()*params_.noise.pixel_noise_var);
                incoming_point_ptr->setIsKnown(feature_ptr->isKnown());
                _feature_list_out.push_back(incoming_point_ptr);

                incoming_point_ptr->setTrackId(feature_ptr->trackId());

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
//    std::cout << "TrackFeatures - Number of Features tracked: " << _feature_list_out.size() << std::endl;
    return _feature_list_out.size();
}

bool ProcessorImageFeature::correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature, FeatureBasePtr _incoming_feature)
{
    std::vector<cv::DMatch> matches_mat;
    FeaturePointImagePtr feat_incoming_ptr = std::static_pointer_cast<FeaturePointImage>(_incoming_feature);
    FeaturePointImagePtr feat_origin_ptr = std::static_pointer_cast<FeaturePointImage>(_origin_feature);

    cv::Mat origin_descriptor = feat_origin_ptr->getDescriptor();
    cv::Mat incoming_descriptor = feat_incoming_ptr->getDescriptor();

    std::vector<cv::KeyPoint> origin_keypoint;
    origin_keypoint.push_back(feat_origin_ptr->getKeypoint());

    Scalar normalized_score = match(origin_descriptor,incoming_descriptor,matches_mat);

    if(normalized_score > params_.matcher.min_normalized_score)
        return true;
    else
    {
        /* CORRECT */

        unsigned int roi_width = params_.matcher.roi_width;
        unsigned int roi_heigth = params_.matcher.roi_height;
        unsigned int roi_x;
        unsigned int roi_y;

        std::vector<cv::KeyPoint> correction_keypoints;
        cv::Mat correction_descriptors;
        std::vector<cv::DMatch> correction_matches;

        FeaturePointImagePtr feat_last_ptr = std::static_pointer_cast<FeaturePointImage>(_last_feature);
        active_search_grid_.hitCell(feat_last_ptr->getKeypoint());

        roi_x = (feat_last_ptr->getKeypoint().pt.x) - (roi_heigth / 2);
        roi_y = (feat_last_ptr->getKeypoint().pt.y) - (roi_width / 2);
        cv::Rect roi(roi_x, roi_y, roi_width, roi_heigth);

        if (detect(image_incoming_, roi, correction_keypoints, correction_descriptors))
        {
            Scalar normalized_score_correction = match(origin_descriptor,correction_descriptors,correction_matches);
            if(normalized_score_correction > params_.matcher.min_normalized_score)
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

unsigned int ProcessorImageFeature::detectNewFeatures(const unsigned int& _max_new_features)
{
    cv::Rect roi;
    std::vector<cv::KeyPoint> new_keypoints;
    cv::Mat new_descriptors;
    cv::KeyPointsFilter keypoint_filter;
    unsigned int n_new_features = 0;

    for (unsigned int n_iterations = 0; _max_new_features == 0 || n_iterations < _max_new_features; n_iterations++)
    {
        if (active_search_grid_.pickRoi(roi))
        {
            detector_roi_.push_back(roi);
            if (detect(image_last_, roi, new_keypoints, new_descriptors))
            {
                std::vector<cv::KeyPoint> list_keypoints = new_keypoints;
                unsigned int index = 0;
                keypoint_filter.retainBest(new_keypoints,1);
                for(unsigned int i = 0; i < list_keypoints.size(); i++)
                {
                    if(list_keypoints[i].pt == new_keypoints[0].pt)
                        index = i;
                }
                if(new_keypoints[0].response > params_.algorithm.min_response_for_new_features)
                {
                    std::cout << "response: " << new_keypoints[0].response << std::endl;
                    FeaturePointImagePtr point_ptr = std::make_shared<FeaturePointImage>(
                            new_keypoints[0],
                            new_descriptors.row(index),
                            Eigen::Matrix2s::Identity()*params_.noise.pixel_noise_var);
                    point_ptr->setIsKnown(false);
                    point_ptr->setTrackId(point_ptr->id());
                    addNewFeatureLast(point_ptr);
                    active_search_grid_.hitCell(new_keypoints[0]);

                    n_new_features++;
                }
            }
            else
                active_search_grid_.blockCell(roi);
        }
        else
            break;
    }

    WOLF_DEBUG( "DetectNewFeatures - Number of new features detected: " , n_new_features );
    return n_new_features;
}

//============================================================

Scalar ProcessorImageFeature::match(cv::Mat _target_descriptor, cv::Mat _candidate_descriptors, std::vector<cv::DMatch>& _cv_matches)
{
    matcher_ptr_->match(_target_descriptor, _candidate_descriptors, _cv_matches);
    Scalar normalized_score = 1 - (Scalar)(_cv_matches[0].distance)/detector_descriptor_params_.size_bits_;
//    std::cout << "target descriptor: " << _target_descriptor.row(0) << std::endl;
//    std::cout << "normalized score: " << normalized_score << std::endl;
    return normalized_score;
}

unsigned int ProcessorImageFeature::detect(cv::Mat _image, cv::Rect& _roi, std::vector<cv::KeyPoint>& _new_keypoints,
                                    cv::Mat& new_descriptors)
{
    cv::Mat _image_roi;
    adaptRoi(_image_roi, _image, _roi);

    detector_descriptor_ptr_->detect(_image_roi, _new_keypoints);
    detector_descriptor_ptr_->compute(_image_roi, _new_keypoints, new_descriptors);

    for (unsigned int i = 0; i < _new_keypoints.size(); i++)
    {
        _new_keypoints[i].pt.x = _new_keypoints[i].pt.x + _roi.x;
        _new_keypoints[i].pt.y = _new_keypoints[i].pt.y + _roi.y;
    }
    return _new_keypoints.size();
}

void ProcessorImageFeature::resetVisualizationFlag(FeatureBaseList& _feature_list_last)
{
    for (auto feature_base_last_ptr : _feature_list_last)
    {
        FeaturePointImagePtr feature_last_ptr = std::static_pointer_cast<FeaturePointImage>(feature_base_last_ptr);
        feature_last_ptr->setIsKnown(true);
    }
}

void ProcessorImageFeature::trimRoi(cv::Rect& _roi)
{
    if(_roi.x < 0)
    {
        int diff_x = -_roi.x;
        _roi.x = 0;
        _roi.width = _roi.width - diff_x;
    }
    if(_roi.y < 0)
    {
        int diff_y = -_roi.y;
        _roi.y = 0;
        _roi.height = _roi.height - diff_y;
    }
    if((unsigned int)(_roi.x + _roi.width) > image_.width_)
    {
        int diff_width = image_.width_ - (_roi.x + _roi.width);
        _roi.width = _roi.width+diff_width;
    }
    if((unsigned int)(_roi.y + _roi.height) > image_.height_)
    {
        int diff_height = image_.height_ - (_roi.y + _roi.height);
        _roi.height = _roi.height+diff_height;
    }
}

void ProcessorImageFeature::inflateRoi(cv::Rect& _roi)
{
    int inflation_rate = detector_descriptor_params_.pattern_radius_;

    _roi.x = _roi.x - inflation_rate;
    _roi.y = _roi.y - inflation_rate;
    _roi.width = _roi.width + 2*inflation_rate;
    _roi.height = _roi.height + 2*inflation_rate;
}

void ProcessorImageFeature::adaptRoi(cv::Mat& _image_roi, cv::Mat _image, cv::Rect& _roi)
{
    inflateRoi(_roi);
    trimRoi(_roi);
    _image_roi = _image(_roi);
}

// draw functions ===================================================================

void ProcessorImageFeature::drawTarget(cv::Mat _image, std::list<cv::Point> _target_list)
{
    // draw the target of the tracking
    for(auto target_point : _target_list)
        cv::circle(_image, target_point, 7, cv::Scalar(255.0, 0.0, 255.0), 1, 3, 0);

    cv::imshow("Feature tracker", _image);
}

void ProcessorImageFeature::drawRoi(cv::Mat _image, std::list<cv::Rect> _roi_list, cv::Scalar _color)
{
    for (auto roi : _roi_list)
        cv::rectangle(_image, roi, _color, 1, 8, 0);

    cv::imshow("Feature tracker", _image);
}

void ProcessorImageFeature::drawFeatures(cv::Mat _image)
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
    cv::imshow("Feature tracker", _image);

    std::cout << "\nKnown: " << known_feature_counter << "\tNew: " << new_feature_counter << std::endl;
    std::cout << "Known features tracked: " << known_feature_counter << "/" << target_size_ << std::endl;
    std::cout << "Percentage known: " << ((float)known_feature_counter/(float)target_size_)*100 << "%" << std::endl;
    std::cout << "Features tracked: " << (last_ptr_->getFeatureList()).size() << "/" << complete_target_size_ << std::endl;
    std::cout << "Percentage: " << ((float)(last_ptr_->getFeatureList()).size()/(float)complete_target_size_)*100 << "%" << std::endl;

    target_size_ = (last_ptr_->getFeatureList()).size();
    complete_target_size_ = 0;
}


ProcessorBasePtr ProcessorImageFeature::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr sensor_ptr)
{
    ProcessorImageFeaturePtr prc_ptr = std::make_shared<ProcessorImageFeature>(*(std::static_pointer_cast<ProcessorParamsImage>(_params)));
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}


} // namespace wolf


// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR("IMAGE FEATURE", ProcessorImageFeature)
} // namespace wolf

