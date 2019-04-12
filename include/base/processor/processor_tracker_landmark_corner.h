/*
 * processor_tracker_landmark_corner.h
 *
 *  Created on: Apr 4, 2016
 *      Author: jvallve
 */

#ifndef SRC_PROCESSOR_TRACKER_LANDMARK_CORNER_H_
#define SRC_PROCESSOR_TRACKER_LANDMARK_CORNER_H_

// Wolf includes
#include "base/sensor/sensor_laser_2D.h"
#include "base/capture/capture_laser_2D.h"
#include "base/feature/feature_corner_2D.h"
#include "base/landmark/landmark_corner_2D.h"
#include "base/factor/factor_corner_2D.h"
#include "base/state_block.h"
#include "base/association/association_tree.h"
#include "base/processor/processor_tracker_landmark.h"

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

//forward declaration to typedef class pointers
struct ProcessorParamsLaser;
typedef std::shared_ptr<ProcessorParamsLaser> ProcessorParamsLaserPtr;

WOLF_PTR_TYPEDEFS(ProcessorTrackerLandmarkCorner);

struct ProcessorParamsLaser : public ProcessorParamsTrackerLandmark
{
        laserscanutils::LineFinderIterativeParams line_finder_params_;
        //TODO: add corner_finder_params
        unsigned int new_corners_th;
        unsigned int loop_frames_th;

        // These values below are constant and defined within the class -- provide a setter or accept them at construction time if you need to configure them
        //        Scalar aperture_error_th_ = 20.0 * M_PI / 180.; //20 degrees
        //        Scalar angular_error_th_ = 10.0 * M_PI / 180.; //10 degrees;
        //        Scalar position_error_th_ = 1;
        //        Scalar min_features_ratio_th_ = 0.5;
};

class ProcessorTrackerLandmarkCorner : public ProcessorTrackerLandmark
{
    private:
        laserscanutils::LineFinderIterative line_finder_;
        laserscanutils::CornerFinder corner_finder_;
        //TODO: add corner_finder_params

        FeatureBasePtrList corners_incoming_;
        FeatureBasePtrList corners_last_;
        unsigned int new_corners_th_;
        unsigned int loop_frames_th_;

        // These values are constant -- provide a setter or accept them at construction time if you need to configure them
        Scalar aperture_error_th_ = 20.0 * M_PI / 180.; //20 degrees
//        Scalar angular_error_th_ = 10.0 * M_PI / 180.; //10 degrees;
//        Scalar position_error_th_ = 1;
//        Scalar min_features_ratio_th_ = 0.5;

        Eigen::Matrix3s R_sensor_world_, R_world_sensor_;
        Eigen::Matrix3s R_robot_sensor_;
        Eigen::Matrix3s R_current_prev_;
        Eigen::Vector3s t_sensor_world_, t_world_sensor_, t_world_sensor_prev_, t_sensor_world_prev_;
        Eigen::Vector3s t_robot_sensor_;
        Eigen::Vector3s t_current_prev_;
        Eigen::Vector3s t_world_robot_;
        bool extrinsics_transformation_computed_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // to guarantee alignment (see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html)

        ProcessorTrackerLandmarkCorner(ProcessorParamsLaserPtr _params_laser);

        virtual ~ProcessorTrackerLandmarkCorner();
        virtual void configure(SensorBasePtr _sensor) { };

    protected:

        virtual void preProcess();
//        virtual void postProcess() { }

        void advanceDerived()
        {
            //std::cout << "\tProcessorTrackerLandmarkCorner::advance:" << std::endl;
            //std::cout << "\t\tcorners_last: " << corners_last_.size() << std::endl;
            //std::cout << "\t\tcorners_incoming_: " << corners_incoming_.size() << std::endl;
            ProcessorTrackerLandmark::advanceDerived();
            while (!corners_last_.empty())
            {
                corners_last_.front()->remove();
                corners_last_.pop_front(); // TODO check if this is needed
            }
            corners_last_ = std::move(corners_incoming_);
        }

        void resetDerived()
        {
            //std::cout << "\tProcessorTrackerLandmarkCorner::reset:" << std::endl;
            //std::cout << "\t\tcorners_last: " << corners_last_.size() << std::endl;
            //std::cout << "\t\tcorners_incoming_: " << corners_incoming_.size() << std::endl;
            ProcessorTrackerLandmark::resetDerived();
            corners_last_ = std::move(corners_incoming_);
        }

        /** \brief Find provided landmarks in the incoming capture
         * \param _landmarks_in input list of landmarks to be found in incoming
         * \param _features_incoming_out returned list of incoming features corresponding to a landmark of _landmarks_in
         * \param _feature_landmark_correspondences returned map of landmark correspondences: _feature_landmark_correspondences[_feature_out_ptr] = landmark_in_ptr
         */
        virtual unsigned int findLandmarks(const LandmarkBasePtrList& _landmarks_in, FeatureBasePtrList& _features_incoming_out,
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
         * \param _max_features maximum number of features detected (-1: unlimited. 0: none)
         * \param _features_last_out The list of detected Features.
         * \return The number of detected Features.
         *
         * This function detects Features that do not correspond to known Features/Landmarks in the system.
         *
         * The function is called in ProcessorTrackerLandmark::processNew() to set the member new_features_last_,
         * the list of newly detected features of the capture last_ptr_.
         */
        virtual unsigned int detectNewFeatures(const int& _max_features, FeatureBasePtrList& _features_incoming_out);

        /** \brief Create one landmark
         *
         * Implement in derived classes to build the type of landmark you need for this tracker.
         */
        virtual LandmarkBasePtr createLandmark(FeatureBasePtr _feature_ptr);

        /** \brief Create a new factor
         * \param _feature_ptr pointer to the Feature to constrain
         * \param _landmark_ptr LandmarkBase pointer to the Landmark constrained.
         *
         * Implement this method in derived classes.
         */
        virtual FactorBasePtr createFactor(FeatureBasePtr _feature_ptr, LandmarkBasePtr _landmark_ptr);

    private:

        void extractCorners(CaptureLaser2DPtr _capture_laser_ptr, FeatureBasePtrList& _corner_list);

        void expectedFeature(LandmarkBasePtr _landmark_ptr, Eigen::Vector4s& expected_feature_,
                             Eigen::Matrix3s& expected_feature_cov_);

        Eigen::VectorXs computeSquaredMahalanobisDistances(const FeatureBasePtr _feature_ptr,
                                                           const Eigen::Vector4s& _expected_feature,
                                                           const Eigen::Matrix3s& _expected_feature_cov,
                                                           const Eigen::MatrixXs& _mu);
    // Factory method
    public:
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr sensor_ptr = nullptr);
};

} // namespace wolf

#endif /* SRC_PROCESSOR_TRACKER_LASER_H_ */
