/**
 * \file processor_tracker_feature_corner.h
 *
 *  Created on: Apr 11, 2016
 *      \author: jvallve
 */

#ifndef PROCESSOR_TRACKER_FEATURE_CORNER_H_
#define PROCESSOR_TRACKER_FEATURE_CORNER_H_

// Wolf includes
#include "sensor_laser_2D.h"
#include "capture_laser_2D.h"
#include "feature_corner_2D.h"
#include "landmark_corner_2D.h"
#include "constraint_corner_2D.h"
#include "state_block.h"
#include "data_association/association_tree.h"
#include "processor_tracker_feature.h"

//laser_scan_utils
//#include "laser_scan_utils/scan_basics.h"
//#include "laser_scan_utils/corner_detector.h"
#include "laser_scan_utils/laser_scan.h"
#include "laser_scan_utils/line_finder_iterative.h"
#include "laser_scan_utils/corner_finder.h"


namespace wolf
{

//some consts.. TODO: this tuning params should be grouped in a struct and passed to the class from ros node, at constructor level
const Scalar aperture_error_th_ = 20.0 * M_PI / 180.; //20 degrees
const Scalar angular_error_th_ = 10.0 * M_PI / 180.; //10 degrees;
const Scalar position_error_th_ = 1;
const Scalar min_features_ratio_th_ = 0.5;

class ProcessorTrackerFeatureCorner : public wolf::ProcessorTrackerFeature
{
    private:
        //laserscanutils::ScanParams scan_params_;
        //laserscanutils::ExtractCornerParams corner_alg_params_;
        //laserscanutils::LaserScan laser_data_;
        laserscanutils::LineFinderIterative line_finder_;
        laserscanutils::CornerFinder corner_finder_;
        //TODO: add corner_finder_params

        FeatureBaseList corners_incoming_;
        FeatureBaseList corners_last_;
        unsigned int n_tracks_th_;

        Eigen::Matrix3s R_world_sensor_, R_world_sensor_prev_;
        Eigen::Matrix3s R_robot_sensor_;
        Eigen::Matrix3s R_current_prev_;
        Eigen::Vector3s t_world_sensor_, t_world_sensor_prev_;
        Eigen::Vector3s t_robot_sensor_;
        Eigen::Vector3s t_current_prev_;
        Eigen::Vector3s t_world_robot_;
        bool extrinsics_transformation_computed_;

    public:
        ProcessorTrackerFeatureCorner(const laserscanutils::LineFinderIterativeParams& _line_finder_params,
                                      const unsigned int& _n_corners_th);
        virtual ~ProcessorTrackerFeatureCorner();

    protected:

        virtual void preProcess();
        virtual void postProcess();

        void advance()
        {
            ProcessorTrackerFeature::advance();
            corners_last_ = std::move(corners_incoming_);
        }

        /** \brief Track provided features from \b last to \b incoming
         * \param _feature_list_in input list of features in \b last to track
         * \param _feature_list_out returned list of features found in \b incoming
         * \param _feature_correspondences returned map of correspondences: _feature_correspondences[feature_out_ptr] = feature_in_ptr
         */
        virtual unsigned int trackFeatures(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out,
                                           FeatureMatchMap& _feature_correspondences);

        /** \brief Correct the drift in incoming feature by re-comparing against the corresponding feature in origin.
         * \param _last_feature input feature in last capture tracked
         * \param _incoming_feature input/output feature in incoming capture to be corrected
         * \return false if the the process discards the correspondence with origin's feature
         */
        virtual bool correctFeatureDrift(const FeatureBase* _last_feature, FeatureBase* _incoming_feature);

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

        /** \brief Create a new constraint
         * \param _feature_ptr pointer to the Feature to constrain
         * \param _feature_other_ptr FeatureBase pointer to the feature constrained.
         *
         * Implement this method in derived classes.
         * This function only creates the constraint, it doesn't add it to any feature.
         *
         * TODO: Make a general ConstraintFactory, and put it in WolfProblem.
         * This factory only needs to know the two derived pointers to decide on the actual Constraint created.
         */
        virtual ConstraintBase* createConstraint(FeatureBase* _feature_ptr, FeatureBase* _feature_other_ptr);

    private:

        void extractCorners(CaptureLaser2D* _capture_laser_ptr, FeatureBaseList& _corner_list);

};

inline ProcessorTrackerFeatureCorner::ProcessorTrackerFeatureCorner(const laserscanutils::LineFinderIterativeParams& _line_finder_params,
                                                                    const unsigned int& _n_corners_th) :
        ProcessorTrackerFeature(PRC_TRACKER_FEATURE_CORNER, 0), line_finder_(_line_finder_params), n_tracks_th_(_n_corners_th), R_world_sensor_(Eigen::Matrix3s::Identity()), R_robot_sensor_(Eigen::Matrix3s::Identity()), extrinsics_transformation_computed_(false)
{
    // TODO Auto-generated constructor stub
}

inline ProcessorTrackerFeatureCorner::~ProcessorTrackerFeatureCorner()
{
    for (auto corner : corners_last_)
        corner->destruct();
    for (auto corner : corners_incoming_)
        corner->destruct();
}

inline void ProcessorTrackerFeatureCorner::postProcess()
{
}

inline bool ProcessorTrackerFeatureCorner::correctFeatureDrift(const FeatureBase* _last_feature,
                                                              FeatureBase* _incoming_feature)
{
    return true;
}

} // namespace wolf

#endif /* PROCESSOR_TRACKER_FEATURE_CORNER_H_ */
