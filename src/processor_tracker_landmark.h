/**
 * \file processor_tracker_landmark.h
 *
 *  Created on: Apr 7, 2016
 *      \author: jvallve
 */

#ifndef PROCESSOR_TRACKER_LANDMARK_H_
#define PROCESSOR_TRACKER_LANDMARK_H_

#include "processor_tracker.h"
#include "capture_base.h"

// Correspondence Feature incoming - Landmark
struct LandmarkMatch
{
        LandmarkBase* landmark_ptr_;
        WolfScalar normalized_score_;

        LandmarkMatch() :
                landmark_ptr_(nullptr), normalized_score_(0.0)
        {
        }
        LandmarkMatch(LandmarkBase* _landmark_ptr, const WolfScalar& _normalized_score) :
                landmark_ptr_(_landmark_ptr), normalized_score_(_normalized_score)
        {

        }
};

// Correspondence Landmark - Feature
typedef std::map<FeatureBase*, LandmarkMatch> FeatureLandmarkMap;

class ProcessorTrackerLandmark : public ProcessorTracker
{
    public:
        ProcessorTrackerLandmark(ProcessorType _tp);
        virtual ~ProcessorTrackerLandmark();

    protected:

        FeatureLandmarkMap incoming_2_landmark_;
        FeatureLandmarkMap last_2_landmark_;

        /** \brief Tracker function
         * \return The number of successful tracks.
         *
         * This is the tracker function to be implemented in derived classes.
         * It operates on the \b incoming capture pointed by incoming_ptr_.
         *
         * This should do one of the following, depending on the design of the tracker:
         *   - Track Features against other Features in the \b origin Capture. Tips:
         *     - An intermediary step of matching against Features in the \b last Capture makes tracking easier.
         *     - Once tracked against last, then the link to Features in \b origin is provided by the Features' Constraints in \b last.
         *     - If required, correct the drift by re-comparing against the Features in origin.
         *     - The Constraints in \b last need to be transferred to \b incoming (moved, not copied).
         *
         * The function must generate the necessary Features in the \b incoming Capture,
         * of the correct type, derived from FeatureBase.
         *
         * It must also generate the constraints, of the correct type, derived from ConstraintBase
         * (through ConstraintAnalytic or ConstraintSparse).
         */
        virtual unsigned int processKnown();

        /** \brief Find provided landmarks in the incoming capture
         * \param _landmark_list_in input list of landmarks to be found in incoming
         * \param _feature_list_out returned list of incoming features corresponding to a landmark of _landmark_list_in
         * \param _feature_landmark_correspondences returned map of landmark correspondences: _feature_landmark_correspondences[_feature_out_ptr] = landmark_in_ptr
         */
        virtual unsigned int findLandmarks(LandmarkBaseList& _landmark_list_in, FeatureBaseList& _feature_list_out,
                                           FeatureLandmarkMap _feature_landmark_correspondences) = 0;

        /** \brief Create one landmark
         *
         * Implement in derived classes to build the type of landmark you need for this tracker.
         */
        virtual LandmarkBase* createLandmark(FeatureBase* _feature_ptr) = 0;

        /** \brief Create a new constraint
         * \param _feature_ptr pointer to the Feature to constrain
         * \param _landmark_ptr LandmarkBase pointer to the Landmark constrained.
         *
         * Implement this method in derived classes.
         *
         * TODO: Make a general ConstraintFactory, and put it in WolfProblem.
         * This factory only needs to know the two derived pointers to decide on the actual Constraint created.
         */
        virtual ConstraintBase* createConstraint(FeatureBase* _feature_ptr, LandmarkBase* _landmark_ptr) = 0;

        unsigned int processNew();

        virtual void establishConstraints();

};

inline void ProcessorTrackerLandmark::establishConstraints()
{
    for (auto last_feature : *(last_ptr_->getFeatureListPtr()))
        last_feature->addConstraint(createConstraint(last_feature, last_2_landmark_[last_feature].landmark_ptr_));
}

#endif /* PROCESSOR_TRACKER_LANDMARK_H_ */
