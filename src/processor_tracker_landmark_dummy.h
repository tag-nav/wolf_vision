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

class ProcessorTrackerLandmarkDummy : public wolf::ProcessorTrackerLandmark
{
    public:
        ProcessorTrackerLandmarkDummy(const unsigned int& _max_new_features);
        virtual ~ProcessorTrackerLandmarkDummy();

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
