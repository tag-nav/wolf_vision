/*
 * processor_laser_corners.h
 *
 *  Created on: Apr 4, 2016
 *      Author: jvallve
 */

#ifndef SRC_PROCESSOR_TRACKER_LASER_H_
#define SRC_PROCESSOR_TRACKER_LASER_H_

// Wolf includes
#include "sensor_laser_2D.h"
#include "capture_laser_2D.h"
#include "feature_corner_2D.h"
#include "landmark_corner_2D.h"
#include "constraint_corner_2D.h"
#include "state_block.h"
#include "data_association/association_tree.h"
#include "processor_tracker_landmark.h"

//laser_scan_utils
#include "laser_scan_utils/scan_basics.h"
#include "laser_scan_utils/corner_detector.h"

namespace wolf
{

//some consts.. TODO: this tuning params should be grouped in a struct and passed to the class from ros node, at constructor level
const Scalar aperture_error_th_ = 20.0 * M_PI / 180.; //20 degrees
const Scalar angular_error_th_ = 10.0 * M_PI / 180.; //10 degrees;
const Scalar position_error_th_ = 1;
const Scalar min_features_ratio_th_ = 0.5;

struct ProcessorParamsLaser : public ProcessorParamsBase
{
        laserscanutils::ScanParams scan_params;
        laserscanutils::ExtractCornerParams corner_alg_params;
        unsigned int n_corners_th;

        // These values below are constant and defined within the class -- provide a setter or accept them at construction time if you need to configure them
        //        Scalar aperture_error_th_ = 20.0 * M_PI / 180.; //20 degrees
        //        Scalar angular_error_th_ = 10.0 * M_PI / 180.; //10 degrees;
        //        Scalar position_error_th_ = 1;
        //        Scalar min_features_ratio_th_ = 0.5;
};

class ProcessorTrackerLaser : public ProcessorTrackerLandmark
{
    private:
        laserscanutils::ScanParams scan_params_;
        laserscanutils::ExtractCornerParams corner_alg_params_;
        unsigned int n_corners_th_;

        // These values are constant -- provide a setter or accept them at construction time if you need to configure them
        Scalar aperture_error_th_ = 20.0 * M_PI / 180.; //20 degrees
        Scalar angular_error_th_ = 10.0 * M_PI / 180.; //10 degrees;
        Scalar position_error_th_ = 1;
        Scalar min_features_ratio_th_ = 0.5;

        Eigen::Matrix3s R_sensor_world_, R_world_sensor_;
        Eigen::Matrix3s R_robot_sensor_;
        Eigen::Vector3s t_sensor_world_, t_world_sensor_;
        Eigen::Vector3s t_robot_sensor_;
        Eigen::Vector3s t_world_robot_;
        bool extrinsics_transformation_computed_;

    public:
        ProcessorTrackerLaser(const laserscanutils::ScanParams& _scan_params,
                              const laserscanutils::ExtractCornerParams& _corner_alg_params,
                              const unsigned int& _n_corners_th);

        virtual ~ProcessorTrackerLaser();

    protected:

        virtual void preProcess();
//        virtual void postProcess() { }

        /** \brief Find provided landmarks in the incoming capture
         * \param _landmark_list_in input list of landmarks to be found in incoming
         * \param _feature_list_out returned list of incoming features corresponding to a landmark of _landmark_list_in
         * \param _feature_landmark_correspondences returned map of landmark correspondences: _feature_landmark_correspondences[_feature_out_ptr] = landmark_in_ptr
         */
        virtual unsigned int findLandmarks(LandmarkBaseList& _landmark_list_in, FeatureBaseList& _feature_list_out,
                                           LandmarkMatchMap& _feature_landmark_correspondences);

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame();

        /** \brief Detect new Features
         * \param _capture_ptr Capture for feature detection. Defaults to incoming_ptr_.
         * \param _new_features_list The list of detected Features. Defaults to member new_features_list_.
         * \return The number of detected Features.
         *
         * This function detects Features that do not correspond to known Features/Landmarks in the system.
         *
         * The function sets the member new_features_list_, the list of newly detected features,
         * to be used for landmark initialization.
         */
        virtual unsigned int detectNewFeatures(const unsigned int& _max_features);

        /** \brief Create one landmark
         *
         * Implement in derived classes to build the type of landmark you need for this tracker.
         */
        virtual LandmarkBase* createLandmark(FeatureBase* _feature_ptr);

        /** \brief Create a new constraint
         * \param _feature_ptr pointer to the Feature to constrain
         * \param _landmark_ptr LandmarkBase pointer to the Landmark constrained.
         *
         * Implement this method in derived classes.
         *
         * TODO: Make a general ConstraintFactory, and put it in WolfProblem.
         * This factory only needs to know the two derived pointers to decide on the actual Constraint created.
         */
        virtual ConstraintBase* createConstraint(FeatureBase* _feature_ptr, LandmarkBase* _landmark_ptr);

    private:

        void extractCorners(const CaptureLaser2D* _capture_laser_ptr, FeatureBaseList& _corner_list);

        void expectedFeature(LandmarkBase* _landmark_ptr, Eigen::Vector4s& expected_feature_,
                             Eigen::Matrix3s& expected_feature_cov_);

        Eigen::VectorXs computeSquaredMahalanobisDistances(const FeatureBase* _feature_ptr,
                                                           const Eigen::Vector4s& _expected_feature,
                                                           const Eigen::Matrix3s& _expected_feature_cov,
                                                           const Eigen::MatrixXs& _mu);
    // Factory method
    public:
        static ProcessorBase* create(const std::string& _unique_name, const ProcessorParamsBase* _params);
};

inline ProcessorTrackerLaser::ProcessorTrackerLaser(const laserscanutils::ScanParams& _scan_params,
                                                    const laserscanutils::ExtractCornerParams& _corner_alg_params,
                                                    const unsigned int& _n_corners_th) :
        ProcessorTrackerLandmark(PRC_TRACKER_LIDAR, 0), scan_params_(_scan_params), corner_alg_params_(
                _corner_alg_params), n_corners_th_(_n_corners_th), R_sensor_world_(Eigen::Matrix3s::Identity()), R_world_sensor_(Eigen::Matrix3s::Identity()), R_robot_sensor_(Eigen::Matrix3s::Identity()), extrinsics_transformation_computed_(false)
{
}

inline ProcessorTrackerLaser::~ProcessorTrackerLaser()
{
}

inline unsigned int ProcessorTrackerLaser::detectNewFeatures(const unsigned int& _max_features)
{
    // already computed since each scan is computed in preprocess(), new corners are classified in findLandmarks()
    return new_features_last_.size();
}

inline LandmarkBase* ProcessorTrackerLaser::createLandmark(FeatureBase* _feature_ptr)
{
    // compute feature global pose
    Eigen::Vector3s feature_global_pose = R_world_sensor_ * _feature_ptr->getMeasurement() + t_world_sensor_;
    // Create new landmark
    return new LandmarkCorner2D(new StateBlock(feature_global_pose.head(2)),
                                new StateBlock(feature_global_pose.tail(1)), _feature_ptr->getMeasurement()(3));
}

inline ConstraintBase* ProcessorTrackerLaser::createConstraint(FeatureBase* _feature_ptr, LandmarkBase* _landmark_ptr)
{
    return new ConstraintCorner2D(_feature_ptr, (LandmarkCorner2D*)((_landmark_ptr)));
}

ProcessorBase* ProcessorTrackerLaser::create(const std::string& _unique_name, const ProcessorParamsBase* _params)
{
    ProcessorParamsLaser* params = (ProcessorParamsLaser*)_params;
    ProcessorTrackerLaser* prc_ptr = new ProcessorTrackerLaser(params->scan_params, params->corner_alg_params, params->n_corners_th);
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

} // namespace wolf


// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf {
namespace
{
const bool registered_prc_laser = ProcessorFactory::get()->registerCreator("LASER", ProcessorTrackerLaser::create);
}
} // namespace wolf


#endif /* SRC_PROCESSOR_TRACKER_LASER_H_ */
