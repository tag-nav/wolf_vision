/**
 * \file processor_tracker_feature_corner.cpp
 *
 *  Created on: Apr 11, 2016
 *      \author: jvallve
 */

#include "processor_tracker_feature_corner.h"
#include "feature_corner_2D.h"

namespace wolf
{

ProcessorTrackerFeatureCorner::ProcessorTrackerFeatureCorner(ProcessorParamsTrackerFeatureCornerPtr _params_tracker_feature_corner) :
                ProcessorTrackerFeature("TRACKER FEATURE CORNER", _params_tracker_feature_corner),
                params_tracker_feature_corner_(_params_tracker_feature_corner),
                R_world_sensor_(Eigen::Matrix3s::Identity()),
                R_robot_sensor_(Eigen::Matrix3s::Identity()),
                extrinsics_transformation_computed_(false)
{
    //
}

ProcessorTrackerFeatureCorner::~ProcessorTrackerFeatureCorner()
{
    for (auto corner : corners_last_)
        corner->remove();
    for (auto corner : corners_incoming_)
        corner->remove();
}

void ProcessorTrackerFeatureCorner::preProcess()
{
    // extract corners of incoming
    extractCorners(std::static_pointer_cast<CaptureLaser2D>(incoming_ptr_), corners_incoming_);

    // store previous transformations
    R_world_sensor_prev_ = R_world_sensor_;
    t_world_sensor_prev_ = t_world_sensor_;

    // compute transformations
    t_world_robot_ = getProblem()->getState(incoming_ptr_->getTimeStamp());

    // world_robot
    Eigen::Matrix3s R_world_robot = Eigen::Matrix3s::Identity();
    R_world_robot.topLeftCorner<2, 2>() = Eigen::Rotation2Ds(t_world_robot_(2)).matrix();

    // robot_sensor (to be computed once if extrinsics are fixed and not dynamic)
    if (getSensorPtr()->extrinsicsInCaptures() || !getSensorPtr()->getPPtr()->isFixed()
            || !getSensorPtr()->getOPtr()->isFixed() || !extrinsics_transformation_computed_)
    {
        t_robot_sensor_.head<2>() = getSensorPtr()->getPPtr()->getState();
        t_robot_sensor_(2) = getSensorPtr()->getOPtr()->getState()(0);
        R_robot_sensor_.topLeftCorner<2, 2>() = Eigen::Rotation2Ds(t_robot_sensor_(2)).matrix();
        extrinsics_transformation_computed_ = true;
    }

    // global_sensor
    R_world_sensor_.topLeftCorner<2, 2>() = R_world_robot.topLeftCorner<2, 2>() * R_robot_sensor_.topLeftCorner<2, 2>();
    t_world_sensor_ = t_world_robot_ + R_robot_sensor_ * t_robot_sensor_;

    // current_prev
    R_current_prev_ = R_world_sensor_.transpose() * R_world_sensor_prev_;
    t_current_prev_ = R_world_sensor_.transpose() * (t_world_sensor_prev_ - t_world_sensor_);
}

void ProcessorTrackerFeatureCorner::advanceDerived()
{
    ProcessorTrackerFeature::advanceDerived();
    corners_last_ = std::move(corners_incoming_);
}

unsigned int ProcessorTrackerFeatureCorner::trackFeatures(const FeatureBaseList& _features_last_in,
                                                         FeatureBaseList& _features_incoming_out,
                                                         FeatureMatchMap& _feature_correspondences)
{
    std::cout << "tracking " << _features_last_in.size() << " features..." << std::endl;

    Eigen::Vector3s expected_feature_pose;
    for (auto feat_in_ptr : _features_last_in)
    {
        expected_feature_pose = R_current_prev_ * feat_in_ptr->getMeasurement().head<3>() + t_current_prev_;
        
        auto feat_out_next = corners_incoming_.begin();
        auto feat_out_it = feat_out_next++; // next is used to obtain the next iterator after splice
        while (feat_out_it != corners_incoming_.end()) //runs over extracted feature
        {
            if (((*feat_out_it)->getMeasurement().head<3>() - expected_feature_pose).squaredNorm() > params_tracker_feature_corner_->position_error_th*params_tracker_feature_corner_->position_error_th)
            {
                // match
                _feature_correspondences[*feat_out_it] = std::make_shared<FeatureMatch>(FeatureMatch({feat_in_ptr,0}));
                
                // move matched feature to list
                _features_incoming_out.splice(_features_incoming_out.end(), corners_incoming_, feat_out_it);
                
                std::cout << "feature " << feat_in_ptr->id() << " tracked!" << std::endl;
            }
            feat_out_it = feat_out_next++;
        }
    }

    return _features_incoming_out.size();
}

bool ProcessorTrackerFeatureCorner::voteForKeyFrame()
{
    return incoming_ptr_->getFeatureList().size() < params_tracker_feature_corner_->n_corners_th;
}

unsigned int ProcessorTrackerFeatureCorner::detectNewFeatures(const unsigned int& _max_features, FeatureBaseList& _features_incoming_out)
{
    // in corners_last_ remain all not tracked corners
    _features_incoming_out = std::move(corners_last_);
    return _features_incoming_out.size();
}

ConstraintBasePtr ProcessorTrackerFeatureCorner::createConstraint(FeatureBasePtr _feature_ptr,
                                                                FeatureBasePtr _feature_other_ptr)
{
    // Getting landmark ptr
    LandmarkCorner2DPtr landmark_ptr = nullptr;
    for (auto constraint : _feature_other_ptr->getConstraintList())
        if (constraint->getLandmarkOtherPtr() != nullptr && constraint->getLandmarkOtherPtr()->getType() == "CORNER 2D")
            landmark_ptr = std::static_pointer_cast<LandmarkCorner2D>(constraint->getLandmarkOtherPtr());

    if (landmark_ptr == nullptr)
    {
        // Create new landmark
        Eigen::Vector3s feature_global_pose = R_world_sensor_ * _feature_ptr->getMeasurement() + t_world_sensor_;
        landmark_ptr = std::make_shared<LandmarkCorner2D>(std::make_shared<StateBlock>(feature_global_pose.head(2)),
                                                          std::make_shared<StateBlock>(feature_global_pose.tail(1)),
                                                          _feature_ptr->getMeasurement()(3));

        // Add landmark constraint to the other feature
        _feature_other_ptr->addConstraint(std::make_shared<ConstraintCorner2D>(_feature_other_ptr, landmark_ptr, shared_from_this()));
    }

//    std::cout << "creating constraint: last feature " << _feature_ptr->getMeasurement()
//              << " with origin feature " << _feature_other_ptr->getMeasurement() << std::endl
//              << " corresponding to landmark " << landmark_ptr->id() << std::endl;
    return std::make_shared<ConstraintCorner2D>(_feature_ptr, landmark_ptr, shared_from_this());
}

void ProcessorTrackerFeatureCorner::extractCorners(CaptureLaser2DPtr _capture_laser_ptr,
                                                   FeatureBaseList& _corner_list)
{
    // TODO: sort corners by bearing
    std::list<laserscanutils::CornerPoint> corners;

    std::cout << "Extracting corners..." << std::endl;
    corner_finder_.findCorners(_capture_laser_ptr->getScan(),
                               (std::static_pointer_cast<SensorLaser2D>(getSensorPtr()))->getScanParams(),
                               line_finder_,
                               corners);

    Eigen::Vector4s measurement;
    for (auto corner : corners)
    {
        measurement.head<2>() = corner.point_.head<2>();
        measurement(2)=corner.orientation_;
        measurement(3)=corner.aperture_;

        _corner_list.push_back(std::make_shared<FeatureCorner2D>(measurement, corner.covariance_));
    }

/*    //variables
    std::list<laserscanutils::Corner> corners;
    //extract corners from range data
    laserscanutils::extractCorners(scan_params_, corner_alg_params_, _capture_laser_ptr->getRanges(), corners);
    //std::cout << corners.size() << " corners extracted" << std::endl;
    Eigen::Matrix4s measurement_cov;
    Eigen::Matrix3s R = Eigen::Matrix3s::Identity();
    Eigen::Vector4s measurement;
    for (auto corner : corners)
    {
        measurement.head(2) = corner.pt_.head(2);
        measurement(2) = corner.orientation_;
        measurement(3) = corner.aperture_;
        Scalar L1 = corner.line_1_.length();
        Scalar L2 = corner.line_2_.length();
        Scalar cov_angle_line1 = 12 * corner.line_1_.error_
                / (pow(L1, 2) * (pow(corner.line_1_.np_, 3) - pow(corner.line_1_.np_, 2)));
        Scalar cov_angle_line2 = 12 * corner.line_2_.error_
                / (pow(L2, 2) * (pow(corner.line_2_.np_, 3) - pow(corner.line_2_.np_, 2)));
        //init cov in corner coordinates
        measurement_cov << corner.line_1_.error_ + cov_angle_line1 * L1 * L1 / 4, 0, 0, 0, 0, corner.line_2_.error_
                + cov_angle_line2 * L2 * L2 / 4, 0, 0, 0, 0, cov_angle_line1 + cov_angle_line2, 0, 0, 0, 0, cov_angle_line1
                + cov_angle_line2;
        measurement_cov = 10 * measurement_cov;
        //std::cout << "New feature: " << meas.transpose() << std::endl;
        //std::cout << measurement_cov << std::endl;
        // Rotate covariance
        R.topLeftCorner<2, 2>() = Eigen::Rotation2Ds(corner.orientation_).matrix();
        measurement_cov.topLeftCorner<3, 3>() = R.transpose() * measurement_cov.topLeftCorner<3, 3>() * R;
        //std::cout << "rotated covariance: " << std::endl;
        //std::cout << measurement_cov << std::endl;
        _corner_list.push_back(new FeatureCorner2D(measurement, measurement_cov));
    }*/
}

} // namespace wolf
