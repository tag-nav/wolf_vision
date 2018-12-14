#include "processor_tracker_landmark_apriltag.h"

#include "capture_image.h"
#include "sensor_camera.h"
#include "rotations.h"
#include "features/feature_apriltag.h"
#include "constraints/constraint_autodiff_apriltag.h"
#include "landmark_apriltag.h"
#include "state_quaternion.h"

// April tags
#include "common/homography.h"
#include "common/zarray.h"

#include <tag36h11.h>
#include <tag36h10.h>
#include <tag36artoolkit.h>
#include <tag25h9.h>
#include <tag25h7.h>

// #include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>

namespace wolf {


// Constructor
ProcessorTrackerLandmarkApriltag::ProcessorTrackerLandmarkApriltag( ProcessorParamsTrackerLandmarkApriltagPtr _params_tracker_landmark_apriltag) :
        ProcessorTrackerLandmark("TRACKER LANDMARK APRILTAG",  _params_tracker_landmark_apriltag ),
        tag_widths_(_params_tracker_landmark_apriltag->tag_widths_),
        tag_width_default_(_params_tracker_landmark_apriltag->tag_width_default_),
        std_xy_ (_params_tracker_landmark_apriltag->std_xy_ ),
        std_z_  (_params_tracker_landmark_apriltag->std_z_  ),
        std_rpy_(_params_tracker_landmark_apriltag->std_rpy_),
        min_time_vote_(_params_tracker_landmark_apriltag->min_time_vote),
        min_features_for_keyframe_(_params_tracker_landmark_apriltag->min_features_for_keyframe)
{
    // configure apriltag detector

    apriltag_family_t tag_family;
    std::string famname(_params_tracker_landmark_apriltag->tag_family_);
    if (famname == "tag36h11")
        tag_family = *tag36h11_create();
    else if (famname == "tag36h10")
        tag_family = *tag36h10_create();
    else if (famname == "tag36artoolkit")
        tag_family = *tag36artoolkit_create();
    else if (famname == "tag25h9")
        tag_family = *tag25h9_create();
    else if (famname == "tag25h7")
        tag_family = *tag25h7_create();
    else {
        WOLF_ERROR("Unrecognized tag family name. Use e.g. \"tag36h11\".");
        exit(-1);
    }

    tag_family.black_border     = _params_tracker_landmark_apriltag->tag_black_border_;

    detector_ = *apriltag_detector_create();
    apriltag_detector_add_family(&detector_, &tag_family);

    detector_.quad_decimate     = _params_tracker_landmark_apriltag->quad_decimate_;
    detector_.quad_sigma        = _params_tracker_landmark_apriltag->quad_sigma_;
    detector_.nthreads          = _params_tracker_landmark_apriltag->nthreads_;
    detector_.debug             = _params_tracker_landmark_apriltag->debug_;
    detector_.refine_edges      = _params_tracker_landmark_apriltag->refine_edges_;
    detector_.refine_decode     = _params_tracker_landmark_apriltag->refine_decode_;
    detector_.refine_pose       = _params_tracker_landmark_apriltag->refine_pose_;
}

// Destructor
ProcessorTrackerLandmarkApriltag::~ProcessorTrackerLandmarkApriltag()
{
}


ProcessorBasePtr ProcessorTrackerLandmarkApriltag::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr _sen_ptr)
{
    std::shared_ptr<ProcessorParamsTrackerLandmarkApriltag> prc_apriltag_params;
    if (_params)
        prc_apriltag_params = std::static_pointer_cast<ProcessorParamsTrackerLandmarkApriltag>(_params);
    else
        prc_apriltag_params = std::make_shared<ProcessorParamsTrackerLandmarkApriltag>();

    ProcessorTrackerLandmarkApriltagPtr prc_ptr = std::make_shared<ProcessorTrackerLandmarkApriltag>(prc_apriltag_params);
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

void ProcessorTrackerLandmarkApriltag::preProcess()
{
    //std::cout << "PreProcess: " << std::endl;
    // first, convert image to grayscale
    cv::cvtColor(std::static_pointer_cast<CaptureImage>(incoming_ptr_)->getImage(),
                 grayscale_image_,
                 cv::COLOR_BGR2GRAY);

    //detect tags in incoming image
    // Make an image_u8_t header for the Mat data
    image_u8_t im = {   .width  = grayscale_image_.cols,
                        .height = grayscale_image_.rows,
                        .stride = grayscale_image_.cols,
                        .buf    = grayscale_image_.data
                    };

    //defined camera parameters
    Eigen::Vector4s camera_intrinsics(getSensorPtr()->getIntrinsicPtr()->getState()); //[cx cy fx fy]
    double fx(camera_intrinsics(2));  //TODO: conversion warning -> wolf::Scalar to double    //test value: 809.135074
    double fy(camera_intrinsics(3));                                                          //test value: 809.410030
    double cx(camera_intrinsics(0));                                                          //test value: 335.684471
    double cy(camera_intrinsics(1));                                                          //test value: 257.352121

    zarray_t detections_ = *apriltag_detector_detect(&detector_, &im);
//    WOLF_TRACE(zarray_size(detections_incoming_), " tags detected");

    detections_incoming_.resize(zarray_size(&detections_));
    // Draw detection outlines
    for (int i = 0; i < detections_incoming_.size(); i++) {
        apriltag_detection_t *det;
        zarray_get(&detections_, i, &det);
        matd_t *pose_matrix = homography_to_pose(det->H, fx, fy, cx, cy);

        Eigen::Affine3ds t_M_c;
        for(int r=0; r<4; r++)
            for(int c=0; c<4; c++)
                t_M_c.matrix()(r,c) = matd_get(pose_matrix,r,c);

        Eigen::Affine3ds c_M_t(t_M_c.inverse());

        // Set the scale of the translation vector from the relation metric_width / units_width
        Eigen::Vector3s translation ( c_M_t.translation() ); // translation vector in apriltag units (tag width in units is 2 units)
        int    tag_id     = det->id;
        Scalar tag_width  = getTagWidth(tag_id);   // tag width in metric
        Scalar scale      = tag_width/2.0;         // (tag width in units is 2 units)
        translation       = scale * translation;

        // set the measured pose
        Eigen::Vector7s pose;
        pose << translation, R2q(c_M_t.linear()).coeffs();

        // compute the covariance
        Eigen::Vector6s var_vec = getVarVec();
        Eigen::Matrix6s cov = var_vec.asDiagonal() ;

        // add to list
        detections_incoming_.push_back(std::make_shared<FeatureApriltag>(pose, cov, tag_id));
    }
}

ConstraintBasePtr ProcessorTrackerLandmarkApriltag::createConstraint(FeatureBasePtr _feature_ptr,
                                                                     LandmarkBasePtr _landmark_ptr)
{
    ConstraintAutodiffApriltagPtr constraint = std::make_shared<ConstraintAutodiffApriltag>(
            getSensorPtr(),
            getLastPtr()->getFramePtr(),
            std::static_pointer_cast<LandmarkApriltag>(_landmark_ptr),
            std::static_pointer_cast<FeatureApriltag> (_feature_ptr ),
            false,
            CTR_ACTIVE
    );
    return constraint;
}

LandmarkBasePtr ProcessorTrackerLandmarkApriltag::createLandmark(FeatureBasePtr _feature_ptr)
{

    // world to rob
    Vector3s pos = getLastPtr()->getFramePtr()->getPPtr()->getState();
    Quaternions quat (getLastPtr()->getFramePtr()->getOPtr()->getState().data());
    Eigen::Affine3ds w_M_r = Eigen::Translation<Scalar,3>(pos.head(3)) * quat;

    // rob to camera
    pos = getSensorPtr()->getPPtr()->getState();
    quat.coeffs() = getSensorPtr()->getOPtr()->getState();
    Eigen::Affine3ds r_M_c = Eigen::Translation<Scalar,3>(pos.head(3)) * quat;

    // camera to lmk
    pos = _feature_ptr->getMeasurement().head(3);
    quat.coeffs() = _feature_ptr->getMeasurement().tail(4);
    Eigen::Affine3ds c_M_l   = Eigen::Translation<Scalar,3>(pos) * quat;

    // world to lmk
    Eigen::Affine3ds w_M_l = w_M_r * r_M_c * c_M_l;

    // make 7-vector for lmk pose
    pos = w_M_l.translation();
    quat = w_M_l.linear();
    Vector7s w_pose_l;
    w_pose_l << pos, quat.coeffs();

    FeatureApriltagPtr feat_april = std::static_pointer_cast<FeatureApriltag>(_feature_ptr);
    int tagid = feat_april->getTagId();

    LandmarkApriltagPtr new_landmark = std::make_shared<LandmarkApriltag>(w_pose_l, tagid, getTagWidth(tagid));

    return new_landmark;
}

unsigned int ProcessorTrackerLandmarkApriltag::detectNewFeatures(const unsigned int& _max_features, FeatureBaseList& _feature_list_out)
{
    for (auto feature_in_image : detections_incoming_)
    {
        bool feature_already_found(false);
        // features and landmarks must be tested with their ID !!
        // list of landmarks in the map
        LandmarkBaseList& landmark_list = getProblem()->getMapPtr()->getLandmarkList();

        //is the feature already associated to a landmark in the map ?
        for(auto it = landmark_list.begin(); it != landmark_list.end(); ++it)
            if(std::static_pointer_cast<LandmarkApriltag>(*it)->getTagId() == std::static_pointer_cast<FeatureApriltag>(feature_in_image)->getTagId())
                feature_already_found = true;

        //if the feature is not yet associated to a landmark in the map then we continue
        if (!feature_already_found)
        {
            // TODO: make detections more robust to decoding errors !!!
            // the detection can be wrong and may decode 2 different tags with the same id.
            // We need to do something if this happens and we need to do it here
            for (FeatureBaseList::iterator it=_feature_list_out.begin(); it != _feature_list_out.end(); ++it)
                if (std::static_pointer_cast<FeatureApriltag>(*it)->getTagId() == std::static_pointer_cast<FeatureApriltag>(feature_in_image)->getTagId())
                {
                    //we have a detection with the same id as the currently processed one. We remove the previous feature from the list for now
                    _feature_list_out.erase(it);
                    break; //it should not be possible two detection with the same id before getting there so we can stop here.
                }

            _feature_list_out.push_back(feature_in_image); // If the feature is not in the map and not in the list of newly detected features yet then we add it.
        } //otherwise we check the next feature
    }

    return _feature_list_out.size();
}

bool ProcessorTrackerLandmarkApriltag::voteForKeyFrame()
{
    Scalar dt_incoming_origin = getIncomingPtr()->getTimeStamp().get() - getOriginPtr()->getTimeStamp().get();
    if (dt_incoming_origin > min_time_vote_){
        return getLastPtr()->getFeatureList().size() >= min_features_for_keyframe_
        && getIncomingPtr()->getFeatureList().size() <  min_features_for_keyframe_;
    }
    return false;
}

unsigned int ProcessorTrackerLandmarkApriltag::findLandmarks(const LandmarkBaseList& _landmark_list_in,
                                                             FeatureBaseList& _feature_list_out,
                                                             LandmarkMatchMap& _feature_landmark_correspondences)
{   
    for (auto feature_in_image : detections_incoming_)
    {
        int tag_id(std::static_pointer_cast<FeatureApriltag>(feature_in_image)->getTagId());

        for (auto landmark_in_ptr : _landmark_list_in)
        {
            if(std::static_pointer_cast<LandmarkApriltag>(landmark_in_ptr)->getTagId() == tag_id)
            {
                _feature_list_out.push_back(feature_in_image);
                Scalar score(1.0);
                LandmarkMatchPtr matched_landmark = std::make_shared<LandmarkMatch>(landmark_in_ptr, score); //TODO: smarter score
                _feature_landmark_correspondences.emplace ( feature_in_image, matched_landmark );
                break;
            }
        }
    }

    return _feature_list_out.size();
}

wolf::Scalar ProcessorTrackerLandmarkApriltag::getTagWidth(int _id) const
{
    if (tag_widths_.find(_id) != tag_widths_.end())
        return tag_widths_.at(_id);
    else
        return tag_width_default_;
}

Eigen::Vector6s ProcessorTrackerLandmarkApriltag::getVarVec()
{
    Eigen::Vector6s var_vec;
    var_vec << std_xy_*std_xy_, std_xy_*std_xy_, std_z_*std_z_, std_rpy_*std_rpy_, std_rpy_*std_rpy_, std_rpy_*std_rpy_;

    return var_vec;
}

FeatureBaseList ProcessorTrackerLandmarkApriltag::getIncomingDetections() const
{
    return detections_incoming_;
}

FeatureBaseList ProcessorTrackerLandmarkApriltag::getLastDetections() const
{
    return detections_last_;
}

void ProcessorTrackerLandmarkApriltag::configure(SensorBasePtr _sensor)
{
    // set processor params from camera params if required

    // SensorCameraPtr camera = std::static_pointer_cast<SensorCamera>(_sensor);
    // [...] add more code if needed
}

} // namespace wolf

// Register in the SensorFactory
#include "processor_factory.h"

namespace wolf
{
WOLF_REGISTER_PROCESSOR("TRACKER LANDMARK APRILTAG", ProcessorTrackerLandmarkApriltag)
}
