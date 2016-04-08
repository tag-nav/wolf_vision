/*
 * \processor_tracker_feature.h
 *
 *  Created on: 27/02/2016
 *      \author: jsola
 */

#ifndef PROCESSOR_TRACKER_FEATURE_H_
#define PROCESSOR_TRACKER_FEATURE_H_

#include "processor_tracker.h"
#include "capture_base.h"

// Correspondence Feature last - Feature incoming
struct FeatureCorrespondence
{
        FeatureBase* feature_ptr_;
        WolfScalar normalized_score_;

        FeatureCorrespondence() :
                feature_ptr_(nullptr), normalized_score_(0.0)
        {

        }
        FeatureCorrespondence(FeatureBase* _last_feature_ptr, const WolfScalar& _normalized_score) :
                feature_ptr_(_last_feature_ptr), normalized_score_(_normalized_score)
        {

        }
};

typedef std::map<FeatureBase*, FeatureCorrespondence> FeatureCorrespondenceMap;

/** \brief Feature tracker processor
 *
 * This class implements the incremental tracker. It contains three pointers to three Captures of type CaptureBase, named \b origin, \b last and \b incoming:
 *   - \b origin: this points to a Capture where all Feature tracks start.
 *   - \b last: the last Capture tracked by the tracker. A sufficient subset of the Features in \b origin is still alive in \b last.
 *   - \b incoming: the capture being received. The tracker operates on this Capture,
 *     establishing correspondences between the features here and the features in \b origin. Each successful correspondence
 *     results in an extension of the track of the Feature up to the \b incoming Capture.
 *
 * It establishes constraints Feature-Landmark;
 *     it uses Landmarks for tracking, in an active-search approach,
 *     and it creates Landmarks with each new Feature detected.
 *
 * The pipeline of actions for an autonomous tracker can be resumed as follows:
 *   - Init the tracker with an \b origin Capture: init();
 *   - On each incoming Capture,
 *     - Track known features in the \b incoming Capture: processKnownFeatures();
 *     - Check if enough Features are still tracked, and vote for a new KeyFrame if this number is too low:
 *     - if voteForKeyFrame()
 *       - Look for new Features and make Landmarks with them:
 *       - detectNewFeatures()
 *       - For each detected Feature:
 *          - create landmarks: createOneLandmark()
 *          - create constraints Feature-Landmark: createConstraint()
 *       - Make a KeyFrame with the \b last Capture: makeKeyFrame();
 *       - Reset the tracker with the \b last Capture as the new \b origin: reset();
 *     - else
 *       - Advance the tracker one Capture ahead: advance()
 *
 * This functionality exists by default in the virtual method process(). You can overload it at your convenience.
 *
 * This is an abstract class. The following pure virtual methods have to be implemented in derived classes:
 *   - processKnownFeatures()
 *   - voteForKeyFrame()
 *   - detectNewFeatures()
 *   - createLandmark()
 *   - createConstraint()
 */
class ProcessorTrackerFeature : public ProcessorTracker
{
    public:

        /** \brief Constructor with type
         */
        ProcessorTrackerFeature(ProcessorType _tp);
        virtual ~ProcessorTrackerFeature();

    protected:

        FeatureCorrespondenceMap incoming_2_last_;
        FeatureCorrespondenceMap last_2_origin_;

        /** \brief Advance the incoming Capture to become the last.
         *
         * Call this when the tracking and keyframe policy work is done and
         * we need to get ready to accept a new incoming Capture.
         */
        virtual void advance();

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

        /** \brief Track provided features from \b last to \b incoming
         * \param _feature_list_in input list of features in \b last to track
         * \param _feature_list_out returned list of features found in \b incoming
         * \param _feature_correspondences returned map of correspondences: _feature_correspondences[feature_out_ptr] = feature_in_ptr
         */
        virtual unsigned int trackFeatures(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out,
                                           FeatureCorrespondenceMap& _feature_correspondences) = 0;

        /** \brief Correct the drift in incoming feature by re-comparing against the corresponding feature in origin.
         * \param _last_feature input feature in last capture tracked
         * \param _incoming_feature input/output feature in incoming capture to be corrected
         * \return false if the the process discards the correspondence with origin's feature
         */
        virtual bool correctFeatureDrift(const FeatureBase* _last_feature, FeatureBase* _incoming_feature) = 0;

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
        virtual unsigned int detectNewFeatures() = 0;

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame() = 0;

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
        virtual ConstraintBase* createConstraint(FeatureBase* _feature_ptr, FeatureBase* _feature_other_ptr) = 0;

        virtual void establishConstraints();

    protected:

        /**\brief Process new Features
         *
         */
        virtual unsigned int processNew();
};

inline void ProcessorTrackerFeature::establishConstraints()
{
    for (auto last_feature : *(last_ptr_->getFeatureListPtr()))
        last_feature->addConstraint(createConstraint(last_feature, last_2_origin_[last_feature].feature_ptr_));
}

#endif /* PROCESSOR_TRACKER_FEATURE_H_ */
