/*
 * \processor_tracker_feature.h
 *
 *  Created on: 27/02/2016
 *      \author: jsola
 */

#ifndef PROCESSOR_TRACKER_FEATURE_H_
#define PROCESSOR_TRACKER_FEATURE_H_

//wolf includes
#include "processor_tracker.h"
#include "capture_base.h"
#include "feature_match.h"
#include "track_matrix.h"
#include "wolf.h"

namespace wolf
{

WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsTrackerFeature);

struct ProcessorParamsTrackerFeature : public ProcessorParamsTracker
{
    //
};
    
WOLF_PTR_TYPEDEFS(ProcessorTrackerFeature);

/** \brief Feature tracker processor
 *
 * This is an abstract class.
 *
 * This class implements the incremental feature tracker.
 *
 * The incremental tracker contains three pointers to three Captures of type CaptureBase,
 * named \b origin, \b last and \b incoming:
 *   - \b origin: this points to a Capture where all Feature tracks start.
 *   - \b last: the last Capture tracked by the tracker.
 *     A sufficient subset of the Features in \b origin is still alive in \b last.
 *   - \b incoming: the capture being received. The tracker operates on this Capture,
 *     establishing correspondences between the features here and the features in \b origin.
 *     Each successful correspondence
 *     results in an extension of the track of the Feature up to the \b incoming Capture.
 *
 * It establishes constraints Feature-Feature or Feature-Landmark.
 *
 * This tracker builds on top of the ProcessorTracker by implementing some of its pure virtual functions.
 * As a reminder, we sketch here the pipeline of the parent ProcessorTracker process() function.
 * We highlight the functions implemented here with a sign  '<--- IMPLEMENTED', and the ones to be implemented by derived classes with '<=== IMPLEMENT'
 *
 *   - On each incoming Capture,
 *     - Track known features in the \b incoming Capture: processKnown()            <--- IMPLEMENTED
 *     - Check if enough Features are still tracked, and vote for a new KeyFrame if this number is too low:
 *     - if voteForKeyFrame()                                                       <=== IMPLEMENT
 *       - Populate the tracker with new Features : processNew()                    <--- IMPLEMENTED
 *       - Make a KeyFrame with the \b last Capture: makeFrame(), setKey()
 *       - Establish constraints of the new Features: establishConstraints()        <--- IMPLEMENTED
 *       - Reset the tracker with the \b last Capture as the new \b origin: reset() <--- IMPLEMENTED
 *     - else
 *       - Advance the tracker one Capture ahead: advance()                         <--- IMPLEMENTED
 *
 * The most important implemented methods are:
 *   - processKnown() : which calls the pure virtuals, to be implemented in derived classes:
 *     - trackFeatures() : track Features from \b last to \b incoming               <=== IMPLEMENT
 *     - correctFeatureDrift() : correct the drift by re-matching from \b origin to \b incoming
 *   - processNew() : which calls the pure virtuals:
 *     - detectNewFeatures() : detects new Features in \b last                      <=== IMPLEMENT
 *     - trackFeatures() : track these new Features again in \b incoming            <=== IMPLEMENT
 *   - establishConstraints() : which calls the pure virtual:
 *     - createConstraint() : create constraint of the correct derived type         <=== IMPLEMENT
 *
 * Should you need extra functionality for your derived types, you can override these two methods,
 *
 *   -  preProcess() { }
 *   -  postProcess() { }
 *
 * which are called at the beginning and at the end of process() respectively.
 * See the doc of these functions for more info.
 */
class ProcessorTrackerFeature : public ProcessorTracker
{
    public:

        /** \brief Constructor with type
         */
        ProcessorTrackerFeature(const std::string& _type,
                                ProcessorParamsTrackerFeaturePtr _params_tracker_feature);
        virtual ~ProcessorTrackerFeature();

    protected:
        ProcessorParamsTrackerFeaturePtr params_tracker_feature_;
        TrackMatrix track_matrix_;

        FeatureBaseList known_features_incoming_;
        FeatureMatchMap matches_last_from_incoming_;

        /** \brief Process known Features
         * \return The number of successful matches.
         *
         * This function operates on the \b incoming capture pointed by incoming_ptr_.
         *
         * This function does:
         *   - Track Features against other Features in the \b origin Capture. Tips:
         *     - An intermediary step of matching against Features in the \b last Capture makes tracking easier.
         *     - Once tracked against last, then the link to Features in \b origin is provided by the Features' Constraints in \b last.
         *     - If required, correct the drift by re-comparing against the Features in origin.
         *     - The Constraints in \b last need to be transferred to \b incoming (moved, not copied).
         *   - Create the necessary Features in the \b incoming Capture,
         *     of the correct type, derived from FeatureBase.
         *   - Create the constraints, of the correct type, derived from ConstraintBase
         *     (through ConstraintAnalytic or ConstraintSparse).
         */
        virtual unsigned int processKnown();

        /** \brief Track provided features from \b last to \b incoming
         * \param _features_last_in input list of features in \b last to track
         * \param _features_incoming_out returned list of features found in \b incoming
         * \param _feature_correspondences returned map of correspondences: _feature_correspondences[feature_out_ptr] = feature_in_ptr
         */
        virtual unsigned int trackFeatures(const FeatureBaseList& _features_last_in, FeatureBaseList& _features_incoming_out, FeatureMatchMap& _feature_correspondences) = 0;

        /** \brief Correct the drift in incoming feature by re-comparing against the corresponding feature in origin.
         * \param _origin_feature input feature in origin capture tracked
         * \param _incoming_feature input/output feature in incoming capture to be corrected
         * \return false if the the process discards the correspondence with origin's feature
         */
        virtual bool correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature, FeatureBasePtr _incoming_feature) = 0;

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame() = 0;

        // We overload the advance and reset functions to update the lists of matches
        void advanceDerived();
        void resetDerived();

        /**\brief Process new Features
         *
         */
        virtual unsigned int processNew(const unsigned int& _max_features);

        /** \brief Detect new Features
         * \param _max_features maximum number of features detected
         * \return The number of detected Features.
         *
         * This function detects Features that do not correspond to known Features/Landmarks in the system.
         *
         * The function sets the member new_features_last_, the list of newly detected features.
         */
        virtual unsigned int detectNewFeatures(const unsigned int& _max_new_features, FeatureBaseList& _features_incoming_out) = 0;

        /** \brief Create a new constraint and link it to the wolf tree
         * \param _feature_ptr pointer to the parent Feature
         * \param _feature_other_ptr pointer to the other feature constrained.
         *
         * Implement this method in derived classes.
         *
         * This function creates a constraint of the appropriate type for the derived processor.
         */
        virtual ConstraintBasePtr createConstraint(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr) = 0;

        /** \brief Establish constraints between features in Captures \b last and \b origin
         */
        virtual void establishConstraints();
};

} // namespace wolf

#endif /* PROCESSOR_TRACKER_FEATURE_H_ */
