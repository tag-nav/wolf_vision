#include "processor_image_landmark.h"

#include "landmark_corner_2D.h"
#include "landmark_AHP.h"
#include "constraint_corner_2D.h"
#include "constraint_AHP.h"
#include "sensor_camera.h"
#include "pinholeTools.h"
#include "trajectory_base.h"
#include "map_base.h"

#include <Eigen/Geometry>
#include <iomanip> //setprecision

namespace wolf
{

ProcessorImageLandmark::ProcessorImageLandmark(const ProcessorParamsImage& _params) :
    ProcessorTrackerLandmark("IMAGE LANDMARK", _params.algorithm.max_new_features, _params.algorithm.time_tolerance),
    matcher_ptr_(nullptr),
    detector_descriptor_ptr_(nullptr),
    params_(_params),
    active_search_grid_(),
    n_feature_(0),
    landmark_idx_non_visible_(0)
{
    // 1. detector-descriptor params
    std::shared_ptr<DetectorDescriptorParamsBase> _dd_params = _params.detector_descriptor_params_ptr;
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

ProcessorImageLandmark::~ProcessorImageLandmark()
{
    //
}

void ProcessorImageLandmark::setup(SensorCamera::Ptr _camera_ptr)
{
    image_.width_ = _camera_ptr->getImgWidth();
    image_.height_ = _camera_ptr->getImgHeight();

    active_search_grid_.setup(image_.width_,image_.height_,
            params_.active_search.grid_width, params_.active_search.grid_height,
            detector_descriptor_params_.pattern_radius_,
            params_.active_search.separation);
}

void ProcessorImageLandmark::preProcess()
{
    image_incoming_ = std::static_pointer_cast<CaptureImage>(incoming_ptr_)->getImage();
    active_search_grid_.renew();

    detector_roi_.clear();
    feat_lmk_found_.clear();
}

void ProcessorImageLandmark::postProcess()
{
    if (last_ptr_!=nullptr)
    {
        cv::Mat image = image_last_.clone();
        if(params_.draw.tracker_roi) drawRoi(image, std::static_pointer_cast<CaptureImage>(last_ptr_), cv::Scalar(255.0, 0.0, 255.0)); //tracker roi
        if(params_.draw.detector_roi) drawRoi(image, detector_roi_, cv::Scalar(0.0,255.0, 255.0)); //active search roi
        if(params_.draw.primary_drawing) drawLandmarks(image);
        if(params_.draw.secondary_drawing) drawFeaturesFromLandmarks(image);
    }
    detector_roi_.clear();
    feat_lmk_found_.clear();
}

unsigned int ProcessorImageLandmark::findLandmarks(const LandmarkBaseList& _landmark_list_in,
                                                          FeatureBaseList& _feature_list_out,
                                                          LandmarkMatchMap& _feature_landmark_correspondences)
{

    unsigned int roi_width = params_.matcher.roi_width;
    unsigned int roi_heigth = params_.matcher.roi_height;
    unsigned int roi_x;
    unsigned int roi_y;
    std::vector<cv::KeyPoint> candidate_keypoints;
    cv::Mat candidate_descriptors;
    std::vector<cv::DMatch> cv_matches;

    for (auto landmark_in_ptr : _landmark_list_in)
    {
//        std::cout << "Landmark number [" << lmk_nbr << "] in " << std::endl;

        /* project */
        std::shared_ptr<LandmarkAHP> landmark_ptr = std::static_pointer_cast<LandmarkAHP>(landmark_in_ptr);
        Eigen::Vector4s point3D_hmg;
        Eigen::Vector3s point2D_hmg;
        Eigen::Vector2s point2D;

        LandmarkInCurrentCamera(incoming_ptr_, landmark_ptr, point3D_hmg);

        point2D_hmg = point3D_hmg.head(3);
        point2D = point2D_hmg.head(2)/point2D_hmg(2);
        point2D = pinhole::distortPoint((std::static_pointer_cast<SensorCamera>(this->getSensorPtr()))->getDistortionVector(),point2D);
        point2D = pinhole::pixellizePoint(this->getSensorPtr()->getIntrinsicPtr()->getVector(),point2D);


        if(pinhole::isInImage(point2D,image_.width_,image_.height_))
        {
            roi_x = (point2D[0]) - (roi_heigth / 2);
            roi_y = (point2D[1]) - (roi_width / 2);
            cv::Rect roi(roi_x, roi_y, roi_width, roi_heigth);

            active_search_grid_.hitCell(point2D);

            cv::Mat target_descriptor = landmark_ptr->getCvDescriptor();

            if (detect(image_incoming_, roi, candidate_keypoints, candidate_descriptors))
            {
                Scalar normalized_score = match(target_descriptor,candidate_descriptors,cv_matches);

                if (normalized_score > params_.matcher.min_normalized_score)
                {
                    std::shared_ptr<FeaturePointImage> incoming_point_ptr = std::make_shared<FeaturePointImage>(
                            candidate_keypoints[cv_matches[0].trainIdx],
                            candidate_descriptors.row(cv_matches[0].trainIdx),
                            Eigen::Matrix2s::Identity()*params_.noise.pixel_noise_var);

                    incoming_point_ptr->setTrackId(landmark_in_ptr->id());
                    incoming_point_ptr->setLandmarkId(landmark_in_ptr->id());
                    incoming_point_ptr->setScore(normalized_score);
                    _feature_list_out.push_back(incoming_point_ptr);

                    incoming_point_ptr->setExpectation(point2D);

                    _feature_landmark_correspondences[_feature_list_out.back()] = std::make_shared<LandmarkMatch>(landmark_in_ptr, normalized_score);

                    feat_lmk_found_.push_back(incoming_point_ptr);

                    // To visualize
                    cv::Rect roi2 = roi;
                    trimRoi(roi2);
                    incoming_point_ptr->setTrackerRoi(roi2);

                }
//                else
//                    std::cout << "NOT FOUND" << std::endl;
            }
//            else
//                std::cout << "NOT DETECTED/FOUND" << std::endl;
        }
//        else
//            std::cout << "NOT in the image" << std::endl;
    }
//    std::cout << "\tNumber of Features tracked: " << _feature_list_out.size() << std::endl;
    landmarks_tracked_ = _feature_list_out.size();
    return _feature_list_out.size();
}

bool ProcessorImageLandmark::voteForKeyFrame()
{

    return false;
//    return landmarks_tracked_ < params_.algorithm.min_features_for_keyframe;
}

unsigned int ProcessorImageLandmark::detectNewFeatures(const unsigned int& _max_features)
{

    cv::Rect roi;
    std::vector<cv::KeyPoint> new_keypoints;
    cv::Mat new_descriptors;
    cv::KeyPointsFilter keypoint_filter;
    unsigned int n_new_features = 0;

    for (unsigned int n_iterations = 0; n_iterations < _max_features; n_iterations++)
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
                    list_response_.push_back(new_keypoints[0].response);
                    std::shared_ptr<FeaturePointImage> point_ptr = std::make_shared<FeaturePointImage>(
                            new_keypoints[0],
                            new_descriptors.row(index),
                            Eigen::Matrix2s::Identity()*params_.noise.pixel_noise_var);
                    point_ptr->setIsKnown(false);
                    point_ptr->setTrackId(point_ptr->id());
                    point_ptr->setExpectation(Eigen::Vector2s(new_keypoints[0].pt.x,new_keypoints[0].pt.y));
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

    WOLF_DEBUG( "Number of new features detected: " , n_new_features );
    return n_new_features;
}

LandmarkBasePtr ProcessorImageLandmark::createLandmark(FeatureBasePtr _feature_ptr)
{

    std::shared_ptr<FeaturePointImage> feat_point_image_ptr = std::static_pointer_cast<FeaturePointImage>( _feature_ptr);
    FrameBasePtr anchor_frame = getLastPtr()->getFramePtr();

    Eigen::Vector2s point2D;
    point2D[0] = feat_point_image_ptr->getKeypoint().pt.x;
    point2D[1] = feat_point_image_ptr->getKeypoint().pt.y;

    Scalar distance = params_.algorithm.distance; // arbitrary value
    Eigen::Vector4s vec_homogeneous;

    point2D = pinhole::depixellizePoint(getSensorPtr()->getIntrinsicPtr()->getVector(),point2D);
    point2D = pinhole::undistortPoint((std::static_pointer_cast<SensorCamera>(getSensorPtr()))->getCorrectionVector(),point2D);

    Eigen::Vector3s point3D;
    point3D.head<2>() = point2D;
    point3D(2) = 1;

    point3D.normalize();


    vec_homogeneous = {point3D(0),point3D(1),point3D(2),1/distance};

    std::shared_ptr<LandmarkAHP> lmk_ahp_ptr = std::make_shared<LandmarkAHP>(vec_homogeneous, anchor_frame, getSensorPtr(), feat_point_image_ptr->getDescriptor());
    _feature_ptr->setLandmarkId(lmk_ahp_ptr->id());
    return lmk_ahp_ptr;
}

ConstraintBasePtr ProcessorImageLandmark::createConstraint(FeatureBasePtr _feature_ptr, LandmarkBasePtr _landmark_ptr)
{

    if ((std::static_pointer_cast<LandmarkAHP>(_landmark_ptr))->getAnchorFrame() == last_ptr_->getFramePtr())
    {
        return std::shared_ptr<ConstraintBase>();
    }
    else
    {
        assert (last_ptr_ && "bad last ptr");
        assert (_landmark_ptr && "bad lmk ptr");
        auto current_frame = last_ptr_->getFramePtr();
        auto landmark_ahp = std::static_pointer_cast<LandmarkAHP>(_landmark_ptr);

        ConstraintAHP::Ptr constraint_ptr = ConstraintAHP::create(_feature_ptr, landmark_ahp, true);


        Eigen::Vector2s expectation_;
        Eigen::Vector3s current_frame_p = last_ptr_->getFramePtr()->getPPtr()->getVector();
        Eigen::Vector4s current_frame_o  = last_ptr_->getFramePtr()->getOPtr()->getVector();
        Eigen::Vector3s anchor_frame_p = landmark_ahp->getAnchorFrame()->getPPtr()->getVector();
        Eigen::Vector4s anchor_frame_o = landmark_ahp->getAnchorFrame()->getOPtr()->getVector();
        Eigen::Vector4s landmark_ = landmark_ahp->getPPtr()->getVector();

        (*constraint_ptr).expectation(current_frame_p.data(), current_frame_o.data(),
                anchor_frame_p.data(), anchor_frame_o.data(),
                landmark_.data(),expectation_.data());
//        std::cout << "====================expectation: " << expectation_.transpose() << std::endl;
        return constraint_ptr;
    }
}


// ==================================================================== My own functions

void ProcessorImageLandmark::LandmarkInCurrentCamera(CaptureBasePtr _capture, std::shared_ptr<LandmarkAHP> _landmark,Eigen::Vector4s& _point3D_hmg)
{
    Eigen::VectorXs current_state = getProblem()->getStateAtTimeStamp(_capture->getTimeStamp());

    assert((current_state.size() == 7 || current_state.size() == 16) && "Wrong state size! Should be 7 for 3D pose or 16 for IMU.");

    Eigen::Vector3s pwr1;
    Eigen::Vector4s quaternion_current_frame;

    // TODO: manage different state sizes better!
    switch(current_state.size())
    {
        case 7:
        {
            pwr1 = current_state.head<3>();
            quaternion_current_frame = current_state.tail<4>();
            break;
        }
        case 16:
        {
            pwr1 = current_state.head<3>();
            quaternion_current_frame = current_state.segment<4>(6);
            break;
        }
        default:
            std::runtime_error("Unrecognized state size. Use 7 or 16");
    }

    Eigen::Vector3s pwr0 = _landmark->getAnchorFrame()->getPPtr()->getVector();
    Eigen::Vector3s prc = this->getSensorPtr()->getPPtr()->getVector();

    Eigen::Translation<Scalar,3> twr1, twr0, trc;
    twr1.x() = pwr1(0); twr1.y() = pwr1(1); twr1.z() = pwr1(2);
    twr0.x() = pwr0(0); twr0.y() = pwr0(1); twr0.z() = pwr0(2);
    trc.x() = prc(0); trc.y() = prc(1); trc.z() = prc(2);


    Eigen::Quaternion<Scalar> qwr1, qwr0, qrc;
    Eigen::Vector4s quaternion_anchor = _landmark->getAnchorFrame()->getOPtr()->getVector();
    Eigen::Vector4s quaternion_sensor = this->getSensorPtr()->getOPtr()->getVector();
    qwr0 = quaternion_anchor;
    qwr1 = quaternion_current_frame;
    qrc = quaternion_sensor;

    Eigen::Vector4s landmark_hmg_c0 = _landmark->getPPtr()->getVector();


    Eigen::Transform<Scalar,3,Eigen::Affine> T_W_R0, T_W_R1, T_R0_C0, T_R1_C1;

    T_W_R0 = twr0 * qwr0;
    T_W_R1 = twr1 * qwr1;
    T_R0_C0 = trc * qrc;
    T_R1_C1 = T_R0_C0;

    Eigen::Vector4s landmark_hmg_c1;
    landmark_hmg_c1 = T_R1_C1.inverse(Eigen::Affine) * T_W_R1.inverse(Eigen::Affine) * T_W_R0 * T_R0_C0 * landmark_hmg_c0;
//    std::cout << "landmark_hmg_c1: " << landmark_hmg_c1 << std::endl;

    _point3D_hmg = landmark_hmg_c1;
}

Scalar ProcessorImageLandmark::match(cv::Mat _target_descriptor, cv::Mat _candidate_descriptors, std::vector<cv::DMatch>& _cv_matches)
{
    matcher_ptr_->match(_target_descriptor, _candidate_descriptors, _cv_matches);
    Scalar normalized_score = 1 - (Scalar)(_cv_matches[0].distance)/detector_descriptor_params_.size_bits_;
    //std::cout << "normalized score: " << normalized_score << std::endl;
    return normalized_score;
}

unsigned int ProcessorImageLandmark::detect(cv::Mat _image, cv::Rect& _roi, std::vector<cv::KeyPoint>& _new_keypoints, cv::Mat& new_descriptors)
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

void ProcessorImageLandmark::trimRoi(cv::Rect& _roi)
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

void ProcessorImageLandmark::inflateRoi(cv::Rect& _roi)
{
    int inflation_rate = detector_descriptor_params_.pattern_radius_;

    _roi.x = _roi.x - inflation_rate;
    _roi.y = _roi.y - inflation_rate;
    _roi.width = _roi.width + 2*inflation_rate;
    _roi.height = _roi.height + 2*inflation_rate;
}

void ProcessorImageLandmark::adaptRoi(cv::Mat& _image_roi, cv::Mat _image, cv::Rect& _roi)
{
    inflateRoi(_roi);
    trimRoi(_roi);
    _image_roi = _image(_roi);
}

void ProcessorImageLandmark::drawRoi(cv::Mat _image, CaptureImage::Ptr _capture, cv::Scalar _color)
{
    for (auto feature : _capture->getFeatureList())
        cv::rectangle(_image, std::static_pointer_cast<FeaturePointImage>(feature)->getTrackerRoi(), _color, 1, 8, 0);

    cv::imshow("Feature tracker", _image);
//    std::cout << "drawRoi" << std::endl;
//    cv::waitKey(0);
}

void ProcessorImageLandmark::drawRoi(cv::Mat _image, std::list<cv::Rect> _roi_list, cv::Scalar _color)
{
    for (auto roi : _roi_list)
        cv::rectangle(_image, roi, _color, 1, 8, 0);

    cv::imshow("Feature tracker", _image);
//    std::cout << "drawRoi" << std::endl;
//    cv::waitKey(0);
}

void ProcessorImageLandmark::drawFeaturesFromLandmarks(cv::Mat _image)
{
    FeaturePointImage::Ptr ftr;
    for(auto feature_point : feat_lmk_found_)
    {
        ftr = std::static_pointer_cast<FeaturePointImage>(feature_point);

        cv::Point2f point = ftr->getKeypoint().pt;
        cv::circle(_image, point, 2, cv::Scalar(255.0, 255.0, 0.0), -1, 8, 0);

        cv::Point2f point2 = point;
        point2.x = point2.x - 16;
        cv::putText(_image, std::to_string(ftr->landmarkId()) + "/" + std::to_string((int)(100*ftr->getScore())), point2,
                    cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 255.0, 0.0));
    }
    cv::imshow("Feature tracker", _image);
}

void ProcessorImageLandmark::drawLandmarks(cv::Mat _image)
{
    unsigned int num_lmks_in_img = 0;
//    cv::Mat image = image_incoming_.clone();
    LandmarkBaseList& last_landmark_list = getProblem()->getMapPtr()->getLandmarkList();

    for (auto landmark_base_ptr : last_landmark_list)
    {
        std::shared_ptr<LandmarkAHP> landmark_ptr = std::static_pointer_cast<LandmarkAHP>(landmark_base_ptr);
        Eigen::Vector4s point3D_hmg;
        Eigen::Vector3s point2D_hmg;
        Eigen::Vector2s point2D;

        LandmarkInCurrentCamera(last_ptr_, landmark_ptr,point3D_hmg);

        point2D_hmg = point3D_hmg.head(3);
        point2D = point2D_hmg.head(2)/point2D_hmg(2);
        point2D = pinhole::distortPoint((std::static_pointer_cast<SensorCamera>(getSensorPtr()))->getDistortionVector(),point2D);
        point2D = pinhole::pixellizePoint(this->getSensorPtr()->getIntrinsicPtr()->getVector(),point2D);

        if(pinhole::isInImage(point2D,image_.width_,image_.height_))
        {
            num_lmks_in_img++;

            cv::Point2f point;
            point.x = point2D[0];
            point.y = point2D[1];

            cv::circle(_image, point, 4, cv::Scalar(51.0, 51.0, 255.0), 1, 3, 0);
            cv::putText(_image, std::to_string(landmark_ptr->id()), point, cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(100.0, 100.0, 255.0) );
        }
    }
    cv::Point label_for_landmark_point;
    label_for_landmark_point.x = 3;
    label_for_landmark_point.y = 10;
    cv::putText(_image, std::to_string(landmarks_tracked_), label_for_landmark_point,
                cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 0.0, 255.0));

    cv::Point label_for_landmark_point2;
    label_for_landmark_point2.x = 3;
    label_for_landmark_point2.y = 20;
    cv::putText(_image, std::to_string(num_lmks_in_img), label_for_landmark_point2,
                cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 0.0, 255.0));

//    std::cout << "\t\tTotal landmarks: " << counter << std::endl;

    cv::imshow("Feature tracker", _image);
}


//namespace wolf{

ProcessorBasePtr ProcessorImageLandmark::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr _sen_ptr)
{
    std::shared_ptr<ProcessorImageLandmark> prc_ptr = std::make_shared<ProcessorImageLandmark>(*(std::static_pointer_cast<ProcessorParamsImage>(_params)));
    prc_ptr->setup(std::static_pointer_cast<SensorCamera>(_sen_ptr));
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

} // namespace wolf


// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR("IMAGE LANDMARK", ProcessorImageLandmark)
} // namespace wolf

