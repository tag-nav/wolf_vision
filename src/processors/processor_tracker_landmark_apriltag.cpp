#include "processor_tracker_landmark_apriltag.h"
 
namespace wolf {


// Constructor
ProcessorTrackerLandmarkApriltag::ProcessorTrackerLandmarkApriltag( ProcessorParamsApriltagPtr _params_tracker_landmark_apriltag) :
        ProcessorTrackerLandmark("TRACKER LANDMARK APRILTAG",  _params_tracker_landmark_apriltag )
{
    // TODO: use parameters from constructor argument
    apriltag_family_t tag_family;
    std::string famname("tag36h11");
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
        exit(-1);  // TODO: default family instead?
    }

    tag_family.black_border = 1; //getopt_get_int(getopt, "border");

    detector_ = *apriltag_detector_create();
    apriltag_detector_add_family(&detector_, &tag_family);

    detector_.quad_decimate = 1.0; //getopt_get_double(getopt, "decimate");
    detector_.quad_sigma = 0.0; //getopt_get_double(getopt, "blur");
    detector_.nthreads = 1; //getopt_get_int(getopt, "threads");
    detector_.debug = 0; //getopt_get_bool(getopt, "debug");
    detector_.refine_edges = 1; //getopt_get_bool(getopt, "refine-edges");
    detector_.refine_decode = 0; //getopt_get_bool(getopt, "refine-decode");
    detector_.refine_pose = 0; //getopt_get_bool(getopt, "refine-pose");
}

// Destructor
ProcessorTrackerLandmarkApriltag::~ProcessorTrackerLandmarkApriltag()
{
}

void ProcessorTrackerLandmarkApriltag::preProcess()
{
    //std::cout << "PreProcess: " << std::endl;
    // first, convert image to grayscale
    cv::cvtColor(std::static_pointer_cast<CaptureImage>(incoming_ptr_)->getImage(), grayscale_image_, cv::COLOR_BGR2GRAY);

    //detect tags in incoming image
    // Make an image_u8_t header for the Mat data
    image_u8_t im = {   .width = grayscale_image_.cols,
                        .height = grayscale_image_.rows,
                        .stride = grayscale_image_.cols,
                        .buf = grayscale_image_.data};

    //defined camera parameters
    SensorBasePtr sensor = incoming_ptr_->getSensorPtr();
    Eigen::Matrix3s camera_intrinsics(std::static_pointer_cast<SensorCamera>(sensor)->getIntrinsicMatrix()); //[fx 0 cx; 0 fy cy; 0 0 1]
    double fx(camera_intrinsics(0,0));  //TODO: conversion warning -> wolf::Scalar to double    //test value: 809.135074
    double fy(camera_intrinsics(1,1));                                                          //test value: 809.410030
    double cx(camera_intrinsics(0,2));                                                          //test value: 335.684471
    double cy(camera_intrinsics(1,2));                                                          //test value: 257.352121

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
        {
            for(int c=0; c<4; c++)
            {
                t_M_c.matrix()(r,c) = matd_get(pose_matrix,r,c);
            }
        }

        Eigen::Affine3ds c_M_t(t_M_c.inverse());
        Eigen::Vector7s pose;
        pose << c_M_t.translation(), R2q(c_M_t.linear()).coeffs();

        Eigen::Matrix6s cov(Eigen::Matrix6s::Identity());
        cov.topLeftCorner(3,3) *= 1e-2;
        cov.bottomRightCorner(3,3) *= 1e-3;
        //get Pose matrix and covarince from det
        detections_incoming_.push_back(std::make_shared<FeatureApriltag>(pose, cov, *det)); //warning: pointer ?
    }
}

ConstraintBasePtr ProcessorTrackerLandmarkApriltag::createConstraint(FeatureBasePtr _feature_ptr, LandmarkBasePtr _landmark_ptr)
{
    ConstraintAutodiffApriltagPtr constraint = std::make_shared<ConstraintAutodiffApriltag>(
            getSensorPtr(),
            getLastPtr()->getFramePtr(),
            std::static_pointer_cast<LandmarkApriltag>(_landmark_ptr),
            std::static_pointer_cast<FeatureApriltag>(_feature_ptr),
            false,
            CTR_ACTIVE
    );
    return constraint;
}

LandmarkBasePtr ProcessorTrackerLandmarkApriltag::createLandmark(FeatureBasePtr _feature_ptr)
{
    //get parameters from sensor
    SensorBasePtr sensor = incoming_ptr_->getSensorPtr();
    Eigen::Matrix3s camera_intrinsics(std::static_pointer_cast<SensorCamera>(sensor)->getIntrinsicMatrix()); //[fx 0 cx; 0 fy cy; 0 0 1]
    double fx(camera_intrinsics(0,0));  //TODO: conversion warning -> wolf::Scalar to double    //test value: 809.135074
    double fy(camera_intrinsics(1,1));                                                          //test value: 809.410030
    double cx(camera_intrinsics(0,2));                                                          //test value: 335.684471
    double cy(camera_intrinsics(1,2));                                                          //test value: 257.352121

    matd_t *pose_matrix = homography_to_pose(std::static_pointer_cast<FeatureApriltag>(_feature_ptr)->getDetection().H, fx, fy, cx, cy);
    Eigen::Affine3ds t_M_c;

    for(int r=0; r<4; r++)
    {
        for(int c=0; c<4; c++)
        {
            t_M_c.matrix()(r,c) = matd_get(pose_matrix,r,c);
        }
    }

    Eigen::Affine3ds c_M_t(t_M_c.inverse());
    Eigen::Vector7s pose;
    pose << c_M_t.translation(), R2q(c_M_t.linear()).coeffs();

    LandmarkApriltagPtr new_landmark = std::make_shared<LandmarkApriltag>(std::make_shared<StateBlock>(pose.head<3>()), std::make_shared<StateQuaternion>(pose.tail<4>()), std::static_pointer_cast<FeatureApriltag>(_feature_ptr)->getDetection().id); //TODO: last parameter is width

    return new_landmark;
}

unsigned int ProcessorTrackerLandmarkApriltag::detectNewFeatures(const unsigned int& _max_features, FeatureBaseList& _feature_list_out)
{
  for (auto feature_in_image : detections_incoming_)
    {
        auto search = matches_landmark_from_incoming_.find(feature_in_image);

        if (search == matches_landmark_from_incoming_.end())
            _feature_list_out.push_back(feature_in_image);
    }

    return _feature_list_out.size();
}

bool ProcessorTrackerLandmarkApriltag::voteForKeyFrame()
{
  std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerLandmarkApriltag::voteForKeyFrame is empty." << std::endl;
  bool return_var{}; //TODO: fill this variable
  return return_var;
}

unsigned int findLandmarks(const LandmarkBaseList& _landmark_list_in, FeatureBaseList& _feature_list_out,
                                           LandmarkMatchMap& _feature_landmark_correspondences)
{   
    for (auto feature_in_image : detections_incoming_)
    {
        int tag_id(std::static_pointer_cast<FeatureApriltag>(feature_in_image)->getDetection().id);

        for (auto landmark_in_ptr : _landmark_list_in)
        {
            if(std::static_pointer_cast<LandmarkApriltag>(landmark_in_ptr)->getTagId() == tag_id)
            {
                _feature_list_out.push_back(feature_in_image);
                Scalar score(1.0);
                LandmarkMatchPtr matched_landmark = std::make_shared<LandmarkMatch>(landmark_in_ptr, score); //TODO: smarter score
                _feature_landmark_correspondences.insert ( std::pair<FeatureBasePtr, LandmarkMatchPtr>(feature_in_image,matched_landmark) );
                break;
            }
        }
    }

    return _feature_list_out.size();
}

// LandmarkMatchMap& ProcessorTrackerLandmarkApriltag::_feature_landmark_correspondences)(LandmarkMatchMap& _feature_landmark_correspondences)
// {
//   std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerLandmarkApriltag::_feature_landmark_correspondences) is empty." << std::endl;
//   LandmarkMatchMap& return_var{}; //TODO: fill this variable
//   return return_var;
// }

} // namespace wolf
