/**
 * \file processor_tracker_landmark_dummy.h
 *
 *  Created on: Apr 12, 2016
 *      \author: jvallve
 */

#ifndef PROCESSOR_TRACKER_LANDMARK_DUMMY_H_
#define PROCESSOR_TRACKER_LANDMARK_DUMMY_H_

#include "processor_tracker_landmark.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(ProcessorTrackerLandmarkDummy);

class ProcessorTrackerLandmarkDummy : public ProcessorTrackerLandmark
{
    public:
        ProcessorTrackerLandmarkDummy(const Scalar _time_tolerance, const unsigned int& _max_new_features);
        virtual ~ProcessorTrackerLandmarkDummy();
        virtual void configure(SensorBasePtr _sensor) { };

    protected:

        unsigned int n_feature_;
        unsigned int landmark_idx_non_visible_;

//        virtual void preProcess() { }
        virtual void postProcess(); // implemented

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
         * \param _max_features maximum number of features to detect.
         * \return The number of detected Features.
         *
         * This function detects Features that do not correspond to known Features/Landmarks in the system.
         *
         * The function sets the member new_features_list_, the list of newly detected features,
         * to be used for landmark initialization.
         */
        virtual unsigned int detectNewFeatures(const unsigned int& _max_features, FeatureBaseList& _feature_list_out);

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
};

inline void ProcessorTrackerLandmarkDummy::postProcess()
{
    landmark_idx_non_visible_++;
    std::cout << "------- Landmarks until " << landmark_idx_non_visible_ << " are now out of scope" << std::endl
            << std::endl;
}

} // namespace wolf

// IMPLEMENTATION

namespace wolf
{

} // namespace wolf

#endif /* PROCESSOR_TRACKER_LANDMARK_DUMMY_H_ */
