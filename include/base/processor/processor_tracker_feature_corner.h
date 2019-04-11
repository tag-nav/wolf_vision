/**
 * \file processor_tracker_feature_corner.h
 *
 *  Created on: Apr 11, 2016
 *      \author: jvallve
 */

#ifndef PROCESSOR_TRACKER_FEATURE_CORNER_H_
#define PROCESSOR_TRACKER_FEATURE_CORNER_H_

// Wolf includes
#include "base/sensor/sensor_laser_2D.h"
#include "base/capture/capture_laser_2D.h"
#include "base/feature/feature_corner_2D.h"
#include "base/landmark/landmark_corner_2D.h"
#include "base/factor/factor_corner_2D.h"
#include "base/state_block.h"
#include "base/association/association_tree.h"
#include "base/processor/processor_tracker_feature.h"

//laser_scan_utils
//#include "laser_scan_utils/scan_basics.h"
//#include "laser_scan_utils/corner_detector.h"
#include "laser_scan_utils/laser_scan.h"
#include "laser_scan_utils/line_finder_iterative.h"
#include "laser_scan_utils/corner_finder.h"

namespace wolf
{
WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsTrackerFeatureCorner);

struct ProcessorParamsTrackerFeatureCorner : public ProcessorParamsTrackerFeature
{
        laserscanutils::LineFinderIterativeParams line_finder_params;
        unsigned int n_corners_th;
        const Scalar position_error_th = 1;
};
    
WOLF_PTR_TYPEDEFS(ProcessorTrackerFeatureCorner);
    

//some consts.. TODO: this tuning params should be grouped in a struct and passed to the class from ros node, at constructor level
//const Scalar aperture_error_th_ = 20.0 * M_PI / 180.; //20 degrees
//const Scalar angular_error_th_ = 10.0 * M_PI / 180.; //10 degrees;
//const Scalar position_error_th_ = 1;
//const Scalar min_features_ratio_th_ = 0.5;

class ProcessorTrackerFeatureCorner : public ProcessorTrackerFeature
{
    private:
        ProcessorParamsTrackerFeatureCornerPtr params_tracker_feature_corner_;
        //laserscanutils::ScanParams scan_params_;
        //laserscanutils::ExtractCornerParams corner_alg_params_;
        //laserscanutils::LaserScan laser_data_;
        laserscanutils::LineFinderIterative line_finder_;
        laserscanutils::CornerFinder corner_finder_;
        //TODO: add corner_finder_params

        FeatureBasePtrList corners_incoming_;
        FeatureBasePtrList corners_last_;

        Eigen::Matrix3s R_world_sensor_, R_world_sensor_prev_;
        Eigen::Matrix3s R_robot_sensor_;
        Eigen::Matrix3s R_current_prev_;
        Eigen::Vector3s t_world_sensor_, t_world_sensor_prev_;
        Eigen::Vector3s t_robot_sensor_;
        Eigen::Vector3s t_current_prev_;
        Eigen::Vector3s t_world_robot_;
        bool extrinsics_transformation_computed_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // to guarantee alignment (see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html)

        ProcessorTrackerFeatureCorner(ProcessorParamsTrackerFeatureCornerPtr _params_tracker_feature_corner);
        virtual ~ProcessorTrackerFeatureCorner();
        virtual void configure(SensorBasePtr _sensor) { };

    protected:

        virtual void preProcess();

        void advanceDerived();

        /** \brief Track provided features from \b last to \b incoming
         * \param _features_last_in input list of features in \b last to track
         * \param _features_incoming_out returned list of features found in \b incoming
         * \param _feature_correspondences returned map of correspondences: _feature_correspondences[feature_out_ptr] = feature_in_ptr
         */
        virtual unsigned int trackFeatures(const FeatureBasePtrList& _features_last_in, FeatureBasePtrList& _features_incoming_out,
                                           FeatureMatchMap& _feature_correspondences);

        /** \brief Correct the drift in incoming feature by re-comparing against the corresponding feature in origin.
         * \param _last_feature input feature in last capture tracked
         * \param _incoming_feature input/output feature in incoming capture to be corrected
         * \return false if the the process discards the correspondence with origin's feature
         */
        virtual bool correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature, FeatureBasePtr _incoming_feature);

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame();

        /** \brief Detect new Features
         * \param _max_features maximum number of features detected (-1: unlimited. 0: none)
         * \param _features_last_out The list of detected Features.
         * \return The number of detected Features.
         *
         * This function detects Features that do not correspond to known Features/Landmarks in the system.
         *
         * The function is called in ProcessorTrackerFeature::processNew() to set the member new_features_last_,
         * the list of newly detected features of the capture last_ptr_.
         */
        virtual unsigned int detectNewFeatures(const int& _max_features, FeatureBasePtrList& _features_incoming_out);

        virtual FactorBasePtr createFactor(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr);

    private:

        void extractCorners(CaptureLaser2DPtr _capture_laser_ptr, FeatureBasePtrList& _corner_list);

};

inline bool ProcessorTrackerFeatureCorner::correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature,
                                                              FeatureBasePtr _incoming_feature)
{
    return true;
}

} // namespace wolf

#endif /* PROCESSOR_TRACKER_FEATURE_CORNER_H_ */
