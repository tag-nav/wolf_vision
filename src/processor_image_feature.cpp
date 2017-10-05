// Wolf includes
#include "processor_image_feature.h"

// other includes
#include <bitset>
#include <algorithm>

namespace wolf
{

ProcessorImageFeature::ProcessorImageFeature(ProcessorParamsImage _params) :
    ProcessorTrackerFeature("IMAGE", _params.algorithm.max_new_features),
    params_(_params),
    active_search_grid_()
{
	// Detector
    std::string det_name = vision_utils::readYamlType(params_.yaml_file_params_vision_utils, "detector");
    det_ptr_ = vision_utils::setupDetector(det_name, det_name + " detector", params_.yaml_file_params_vision_utils);

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
    std::string des_name = vision_utils::readYamlType(params_.yaml_file_params_vision_utils, "descriptor");
    des_ptr_ = vision_utils::setupDescriptor(des_name, des_name + " descriptor", params_.yaml_file_params_vision_utils);

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
    std::string mat_name = vision_utils::readYamlType(params_.yaml_file_params_vision_utils, "matcher");
    mat_ptr_ = vision_utils::setupMatcher(mat_name, mat_name + " matcher", params_.yaml_file_params_vision_utils);

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
            det_ptr_->getPatternRadius(),
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
    std::vector<cv::KeyPoint> candidate_keypoints;
    cv::Mat candidate_descriptors;
    std::vector<cv::DMatch> cv_matches;

    for (auto feature_base_ptr : _feature_list_in)
    {
        FeaturePointImagePtr feature_ptr = std::static_pointer_cast<FeaturePointImage>(feature_base_ptr);

        cv::Rect roi = vision_utils::setRoi(feature_ptr->getKeypoint().pt.x, feature_ptr->getKeypoint().pt.y, mat_ptr_->getParams()->roi_width, mat_ptr_->getParams()->roi_height);

        active_search_grid_.hitCell(feature_ptr->getKeypoint());
        complete_target_size_++;
        cv::Mat target_descriptor = feature_ptr->getDescriptor();

        //lists used to debug
        tracker_target_.push_back(feature_ptr->getKeypoint().pt);
        tracker_roi_.push_back(roi);

        if (detect(image_incoming_, roi, candidate_keypoints, candidate_descriptors))
        {
            Scalar normalized_score = match(target_descriptor,candidate_descriptors,cv_matches);
            if (normalized_score > mat_ptr_->getParams()->min_norm_score)
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

    if(normalized_score > mat_ptr_->getParams()->min_norm_score)
        return true;
    else
    {
        /* CORRECT */

        unsigned int roi_width = mat_ptr_->getParams()->roi_width;
        unsigned int roi_heigth = mat_ptr_->getParams()->roi_height;
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
            if(normalized_score_correction > mat_ptr_->getParams()->min_norm_score)
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
    mat_ptr_->match(_target_descriptor, _candidate_descriptors, _cv_matches);
    Scalar normalized_score = 1 - (Scalar)(_cv_matches[0].distance)/(des_ptr_->getSize()*8);
    return normalized_score;
}

unsigned int ProcessorImageFeature::detect(cv::Mat _image, cv::Rect& _roi, std::vector<cv::KeyPoint>& _new_keypoints,
                                    cv::Mat& new_descriptors)
{
    _new_keypoints = det_ptr_->detect(_image, _roi);
    new_descriptors = des_ptr_->getDescriptor(_image, _new_keypoints);
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

