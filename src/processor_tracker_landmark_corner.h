/*
 * processor_tracker_landmark_corner.h
 *
 *  Created on: Apr 4, 2016
 *      Author: jvallve
 */

#ifndef SRC_PROCESSOR_TRACKER_LANDMARK_CORNER_H_
#define SRC_PROCESSOR_TRACKER_LANDMARK_CORNER_H_

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

struct ProcessorParamsLaser : public ProcessorParamsBase
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


        FeatureBaseList corners_incoming_;
        FeatureBaseList corners_last_;
        unsigned int new_corners_th_;
        unsigned int loop_frames_th_;

        // These values are constant -- provide a setter or accept them at construction time if you need to configure them
        Scalar aperture_error_th_ = 20.0 * M_PI / 180.; //20 degrees
        Scalar angular_error_th_ = 10.0 * M_PI / 180.; //10 degrees;
        Scalar position_error_th_ = 1;
        Scalar min_features_ratio_th_ = 0.5;

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

        ProcessorTrackerLandmarkCorner(const laserscanutils::LineFinderIterativeParams& _line_finder_params,
                                       const unsigned int& _new_corners_th, const unsigned int& _loop_frames_th);

        virtual ~ProcessorTrackerLandmarkCorner();

    protected:

        virtual void preProcess();
//        virtual void postProcess() { }

        void advance()
        {
            //std::cout << "\tProcessorTrackerLandmarkCorner::advance:" << std::endl;
            //std::cout << "\t\tcorners_last: " << corners_last_.size() << std::endl;
            //std::cout << "\t\tcorners_incoming_: " << corners_incoming_.size() << std::endl;
            ProcessorTrackerLandmark::advance();
            while (!corners_last_.empty())
            {
                corners_last_.front()->remove();
                corners_last_.pop_front(); // TODO check if this is needed
            }
            corners_last_ = std::move(corners_incoming_);
        }

        void reset()
        {
            //std::cout << "\tProcessorTrackerLandmarkCorner::reset:" << std::endl;
            //std::cout << "\t\tcorners_last: " << corners_last_.size() << std::endl;
            //std::cout << "\t\tcorners_incoming_: " << corners_incoming_.size() << std::endl;
            ProcessorTrackerLandmark::reset();
            corners_last_ = std::move(corners_incoming_);
        }

        /** \brief Find provided landmarks in the incoming capture
         * \param _landmark_list_in input list of landmarks to be found in incoming
         * \param _feature_list_out returned list of incoming features corresponding to a landmark of _landmark_list_in
         * \param _feature_landmark_correspondences returned map of landmark correspondences: _feature_landmark_correspondences[_feature_out_ptr] = landmark_in_ptr
         */
        virtual unsigned int findLandmarks(const LandmarkBaseList& _landmark_list_in, FeatureBaseList& _feature_list_out,
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
        virtual LandmarkBasePtr createLandmark(FeatureBasePtr _feature_ptr);

        /** \brief Create a new constraint
         * \param _feature_ptr pointer to the Feature to constrain
         * \param _landmark_ptr LandmarkBase pointer to the Landmark constrained.
         *
         * Implement this method in derived classes.
         */
        virtual ConstraintBasePtr createConstraint(FeatureBasePtr _feature_ptr, LandmarkBasePtr _landmark_ptr);

    private:

        void extractCorners(CaptureLaser2DPtr _capture_laser_ptr, FeatureBaseList& _corner_list);

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

inline ProcessorTrackerLandmarkCorner::ProcessorTrackerLandmarkCorner(const laserscanutils::LineFinderIterativeParams& _line_finder_params,
                                                                      const unsigned int& _new_corners_th, const unsigned int& _loop_frames_th) :
        ProcessorTrackerLandmark("TRACKER LANDMARK CORNER", 0), line_finder_(_line_finder_params), new_corners_th_(_new_corners_th), loop_frames_th_(_loop_frames_th), R_sensor_world_(Eigen::Matrix3s::Identity()), R_world_sensor_(Eigen::Matrix3s::Identity()), R_robot_sensor_(Eigen::Matrix3s::Identity()), extrinsics_transformation_computed_(false)
{
}

inline ProcessorTrackerLandmarkCorner::~ProcessorTrackerLandmarkCorner()
{
    while (!corners_last_.empty())
    {
        corners_last_.front()->remove();
        corners_last_.pop_front(); // TODO check if this is needed
    }
    while (!corners_incoming_.empty())
    {
        corners_incoming_.front()->remove();
        corners_incoming_.pop_front(); // TODO check if this is needed
    }
}

inline unsigned int ProcessorTrackerLandmarkCorner::detectNewFeatures(const unsigned int& _max_features)
{
    // already computed since each scan is computed in preprocess()
    new_features_last_ = std::move(corners_last_);
    return new_features_last_.size();
}

inline LandmarkBasePtr ProcessorTrackerLandmarkCorner::createLandmark(FeatureBasePtr _feature_ptr)
{
    //std::cout << "ProcessorTrackerLandmarkCorner::createLandmark" << std::endl;

    // compute feature global pose
    Eigen::Vector3s feature_global_pose = R_world_sensor_ * _feature_ptr->getMeasurement().head<3>() + t_world_sensor_;
    // Create new landmark
    return std::make_shared<LandmarkCorner2D>(std::make_shared<StateBlock>(feature_global_pose.head(2)),
                                              std::make_shared<StateBlock>(feature_global_pose.tail(1)),
                                              _feature_ptr->getMeasurement()(3));
}

inline ConstraintBasePtr ProcessorTrackerLandmarkCorner::createConstraint(FeatureBasePtr _feature_ptr, LandmarkBasePtr _landmark_ptr)
{
    assert(_feature_ptr != nullptr && _landmark_ptr != nullptr && "ProcessorTrackerLandmarkCorner::createConstraint: feature and landmark pointers can not be nullptr!");

    return std::make_shared<ConstraintCorner2D>(shared_from_this(), _feature_ptr, std::static_pointer_cast<LandmarkCorner2D>((_landmark_ptr)));
}

} // namespace wolf

#endif /* SRC_PROCESSOR_TRACKER_LASER_H_ */
