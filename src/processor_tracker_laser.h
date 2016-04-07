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
#include "state_block.h"

//laser_scan_utils
#include "laser_scan_utils/scan_basics.h"
#include "laser_scan_utils/corner_detector.h"
#include "processor_tracker_feature.h"

//some consts.. TODO: this tuning params should be grouped in a struct and passed to the class from ros node, at constructor level
const WolfScalar aperture_error_th_ = 20.0*M_PI/180.; //20 degrees
const WolfScalar angular_error_th_ = 10.0*M_PI/180.; //10 degrees;
const WolfScalar position_error_th_ = 1;
const WolfScalar min_features_ratio_th_ = 0.5;
// TODO: replace this consts
const WolfScalar CONTAINER_WIDTH = 2.44;
const WolfScalar CONTAINER_LENGTH = 12.20;

class ProcessorTrackerLaser : public ProcessorTrackerFeature
{
    private:
        laserscanutils::ScanParams scan_params_;
        laserscanutils::ExtractCornerParams corner_alg_params_;
        CaptureLaser2D* scan_last_;
        CaptureLaser2D* scan_incoming_;
        std::list<FeatureCorner2D*> all_corners_incoming_;

    public:
        ProcessorTrackerLaser();

        //Destructor
        virtual ~ProcessorTrackerLaser();

        /** \brief Tracker function
         * \return The number of successful tracks.
         */
        virtual unsigned int processKnown();

    protected:

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
        virtual unsigned int detectNewFeatures();

        /** \brief Track provided features from \b last to \b incoming
         * \param _feature_list_in input list of features in \b last to track
         * \param _feature_list_out returned list of features found in \b incoming
         */
        virtual unsigned int track(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out);

        /** \brief Create one landmark
         *
         * Implement in derived classes to build the type of landmark you need for this tracker.
         */
        virtual LandmarkBase* createLandmark(FeatureBase* _feature_ptr);

        /** \brief Create a new constraint
         * \param _feature_ptr pointer to the Feature to constrain
         * \param _node_ptr NodeBase pointer to the other entity constrained. It can only be of the types FeatureBase and LandmarkBase.
         *
         * This function will be called with one of these options (and hence the second parameter NodeBase *):
         *  - createConstraint(FeatureBase *, FeatureBase *)
         *  - createConstraint(FeatureBase *, LandmarkBase *)
         *
         * Implement this method in derived classes to build the type of constraint
         * appropriate for the pair feature-feature or feature-landmark used by this tracker.
         *
         * TODO: Make a general ConstraintFactory, and put it in WolfProblem.
         * This factory only needs to know the two derived pointers to decide on the actual Constraint created.
         */
        virtual ConstraintBase* createConstraint(FeatureBase* _feature_ptr, NodeBase* _node_ptr);

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame();
};
#endif /* SRC_PROCESSOR_TRACKER_LASER_H_ */
