/*
 * processor_tracker_landmark_polyline.h
 *
 *  Created on: May 26, 2016
 *      Author: jvallve
 */

#ifndef SRC_PROCESSOR_TRACKER_LANDMARK_POLYLINE_H_
#define SRC_PROCESSOR_TRACKER_LANDMARK_POLYLINE_H_

// Wolf includes
#include "sensor_laser_2D.h"
#include "capture_laser_2D.h"
#include "feature_polyline_2D.h"
#include "landmark_polyline_2D.h"
#include "constraint_point_2D.h"
#include "constraint_point_to_line_2D.h"
#include "state_block.h"
#include "data_association/association_tree.h"
#include "processor_tracker_landmark.h"

//laser_scan_utils
#include "laser_scan_utils/laser_scan.h"
#include "laser_scan_utils/line_finder_iterative.h"
#include "laser_scan_utils/polyline.h"

namespace wolf
{

//some consts.. TODO: this tuning params should be grouped in a struct and passed to the class from ros node, at constructor level
const Scalar position_error_th_ = 1;
const Scalar min_features_ratio_th_ = 0.5;


// Match Feature - Landmark
struct LandmarkPolylineMatch : public LandmarkMatch
{
         int landmark_match_from_id_;
         int feature_match_from_id_;
         int landmark_match_to_id_;
         int feature_match_to_id_;
//    std::vector<unsigned int> landmark_points_match_;
//    std::vector<unsigned int> feature_points_match_;
//    std::vector<unsigned int> feature_points_add_front_;
//    std::vector<unsigned int> feature_points_add_back_;
};

struct ProcessorParamsPolyline : public ProcessorParamsBase
{
        laserscanutils::LineFinderIterativeParams line_finder_params_;
        //TODO: add corner_finder_params
        unsigned int new_features_th;
        unsigned int loop_frames_th;
        Scalar time_tolerance;

        // These values below are constant and defined within the class -- provide a setter or accept them at construction time if you need to configure them
        //        Scalar aperture_error_th_ = 20.0 * M_PI / 180.; //20 degrees
        //        Scalar angular_error_th_ = 10.0 * M_PI / 180.; //10 degrees;
        //        Scalar position_error_th_ = 1;
        //        Scalar min_features_ratio_th_ = 0.5;
};

class ProcessorTrackerLandmarkPolyline : public ProcessorTrackerLandmark
{
    private:
        laserscanutils::LineFinderIterative line_finder_;
        //TODO: add corner_finder_params

        FeatureBaseList polylines_incoming_;
        FeatureBaseList polylines_last_;
        unsigned int new_features_th_;
        unsigned int loop_frames_th_;

        // These values are constant -- provide a setter or accept them at construction time if you need to configure them
        Scalar aperture_error_th_ = 20.0 * M_PI / 180.; //20 degrees
        Scalar angular_error_th_ = 10.0 * M_PI / 180.; //10 degrees;
        Scalar position_error_th_ = 1;
        Scalar min_features_ratio_th_ = 0.5;

        Eigen::Matrix2s R_sensor_world_, R_world_sensor_;
        Eigen::Matrix2s R_robot_sensor_;
        Eigen::Matrix2s R_current_prev_;
        Eigen::Vector2s t_sensor_world_, t_world_sensor_, t_world_sensor_prev_, t_sensor_world_prev_;
        Eigen::Vector2s t_robot_sensor_;
        Eigen::Vector2s t_current_prev_;
        Eigen::Vector2s t_world_robot_;
        bool extrinsics_transformation_computed_;

    public:
        ProcessorTrackerLandmarkPolyline(const laserscanutils::LineFinderIterativeParams& _line_finder_params,
                                       const unsigned int& _new_features_th, const unsigned int& _loop_frames_th, const Scalar& _time_tolerance = 0.1);

        virtual ~ProcessorTrackerLandmarkPolyline();

        const FeatureBaseList& getLastPolylines() const;

    protected:

        virtual void preProcess();
        void computeTransformations(const TimeStamp& _ts);
        virtual void postProcess()
        {
//            std::cout << "postProcess: " << std::endl;
//            std::cout << "New Last features: " << getNewFeaturesListLast().size() << std::endl;
//            std::cout << "Last features: " << last_ptr_->getFeatureListPtr()->size() << std::endl;
        }

        void advance();

        void reset();

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

        /** \brief Creates a landmark for each of new_features_last_
         **/
        virtual void createNewLandmarks(LandmarkBaseList& _new_landmarks);

        /** \brief Create one landmark
         *
         * Implement in derived classes to build the type of landmark you need for this tracker.
         */
        virtual LandmarkBase* createLandmark(FeatureBase* _feature_ptr);

        /** \brief Establish constraints between features in Captures \b last and \b origin
         */
        virtual void establishConstraints();

        /** \brief Create a new constraint
         * \param _feature_ptr pointer to the Feature to constrain
         * \param _landmark_ptr LandmarkBase pointer to the Landmark constrained.
         *
         * Implement this method in derived classes.
         *
         * TODO: Make a general ConstraintFactory, and put it in WolfProblem. JV: I disagree..
         * This factory only needs to know the two derived pointers to decide on the actual Constraint created.
         */
        virtual ConstraintBase* createConstraint(FeatureBase* _feature_ptr, LandmarkBase* _landmark_ptr);

    private:

        void extractPolylines(CaptureLaser2D* _capture_laser_ptr, FeatureBaseList& _polyline_list);

        void expectedFeature(LandmarkBase* _landmark_ptr, Eigen::MatrixXs& expected_feature_,
                             Eigen::MatrixXs& expected_feature_cov_);

        Eigen::VectorXs computeSquaredMahalanobisDistances(const Eigen::Vector2s& _feature,
                                                           const Eigen::Matrix2s& _feature_cov,
                                                           const Eigen::Vector2s& _expected_feature,
                                                           const Eigen::Matrix2s& _expected_feature_cov,
                                                           const Eigen::MatrixXs& _mu);
        Scalar sqDistPointToLine(const Eigen::Vector3s& _A, const Eigen::Vector3s& _A_aux, const Eigen::Vector3s& _B,
                               bool _A_extreme, bool _B_extreme);
    // Factory method
    public:
        static ProcessorBase* create(const std::string& _unique_name, const ProcessorParamsBase* _params);
};

inline ProcessorTrackerLandmarkPolyline::ProcessorTrackerLandmarkPolyline(const laserscanutils::LineFinderIterativeParams& _line_finder_params,
                                                                      const unsigned int& _new_corners_th, const unsigned int& _loop_frames_th, const Scalar& _time_tolerance) :
        ProcessorTrackerLandmark(PRC_TRACKER_LANDMARK_CORNER, 0, _time_tolerance), line_finder_(_line_finder_params), new_features_th_(_new_corners_th), loop_frames_th_(_loop_frames_th), R_sensor_world_(Eigen::Matrix2s::Identity()), R_world_sensor_(Eigen::Matrix2s::Identity()), R_robot_sensor_(Eigen::Matrix2s::Identity()), extrinsics_transformation_computed_(false)
{
}

inline unsigned int ProcessorTrackerLandmarkPolyline::detectNewFeatures(const unsigned int& _max_features)
{
    // already computed since each scan is computed in preprocess()
    new_features_last_ = std::move(polylines_last_);
    return new_features_last_.size();
}

inline void ProcessorTrackerLandmarkPolyline::advance()
{
    //std::cout << "\tProcessorTrackerLandmarkPolyline::advance:" << std::endl;
    //std::cout << "\t\tcorners_last: " << polylines_last_.size() << std::endl;
    //std::cout << "\t\tcorners_incoming_: " << polylines_incoming_.size() << std::endl;
    ProcessorTrackerLandmark::advance();
    for (auto polyline : polylines_last_)
        polyline->destruct();
    polylines_last_ = std::move(polylines_incoming_);
    //std::cout << "advanced" << std::endl;
}

inline void ProcessorTrackerLandmarkPolyline::reset()
{
    //std::cout << "\tProcessorTrackerLandmarkPolyline::reset:" << std::endl;
    //std::cout << "\t\tcorners_last: " << corners_last_.size() << std::endl;
    //std::cout << "\t\tcorners_incoming_: " << polylines_incoming_.size() << std::endl;
    ProcessorTrackerLandmark::reset();
    polylines_last_ = std::move(polylines_incoming_);
}

inline const FeatureBaseList& ProcessorTrackerLandmarkPolyline::getLastPolylines() const
{
    return polylines_last_;
}

} // namespace wolf

#endif /* SRC_PROCESSOR_TRACKER_LASER_H_ */
