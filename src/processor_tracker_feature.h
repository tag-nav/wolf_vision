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

/** \brief General tracker processor
 *
 * This class implements the incremental tracker. It contains three pointers to three Captures of type CaptureBase, named \b origin, \b last and \b incoming:
 *   - \b origin: this points to a Capture where all Feature tracks start.
 *   - \b last: the last Capture tracked by the tracker. A sufficient subset of the Features in \b origin is still alive in \b last.
 *   - \b incoming: the capture being received. The tracker operates on this Capture,
 *     establishing correspondences between the features here and the features in \b origin. Each successful correspondence
 *     results in an extension of the track of the Feature up to the \b incoming Capture.
 *
 * A tracker can be designed to track either Features or Landmarks. This property must be set at construction time.
 *   - If tracking Features, it establishes constraints Feature-Feature;
 *     it does not use Landmarks, nor it creates Landmarks.
 *   - If tracking Landmarks, it establishes constraints Feature-Landmark;
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
 *       - if we use landmarks, do for each detected Feature:
 *         - create landmarks: createOneLandmark()
 *       - create constraints Feature-Landmark or Feature-Feature: createConstraint()
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

        /** \brief Constructor with options
         * \param _autonomous Set to make the tracker autonomous to create KeyFrames and/or Landmarks.
         * \param _uses_landmarks Set to make the tracker work with Landmarks. Un-set to work only with Features.
         */
        ProcessorTrackerFeature(ProcessorType _tp);
        virtual ~ProcessorTrackerFeature();

        // We start with all the pure-virtual functions

        /** \brief Tracker function
         * \return The number of successful tracks.
         *
         * This is the tracker function to be implemented in derived classes.
         * It operates on the \b incoming capture pointed by incoming_ptr_.
         *
         * This should do one of the following, depending on the design of the tracker -- see use_landmarks_:
         *   - Track Features against other Features in the \b origin Capture. Tips:
         *     - An intermediary step of matching against Features in the \b last Capture makes tracking easier.
         *     - Once tracked against last, then the link to Features in \b origin is provided by the Features' Constraints in \b last.
         *     - If required, correct the drift by re-comparing against the Features in origin.
         *     - The Constraints in \b last need to be transferred to \b incoming (moved, not copied).
         *   - Track Features against Landmarks in the Map. Tips:
         *     - The links to the Landmarks are in the Features' Constraints in last.
         *     - The Constraints in \b last need to be transferred to \b incoming (moved, not copied).
         *
         * The function must generate the necessary Features in the \b incoming Capture,
         * of the correct type, derived from FeatureBase.
         *
         * It must also generate the constraints, of the correct type, derived from ConstraintBase
         * (through ConstraintAnalytic or ConstraintSparse).
         */
        virtual unsigned int processKnown() = 0;


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
        virtual unsigned int detectNewFeatures() = 0;

        /** \brief Track provided features from \b last to \b incoming
         * \param _feature_list_in input list of features in \b last to track
         * \param _feature_list_out returned list of features found in \b incoming
         */
        virtual unsigned int track(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out) = 0;

        /** \brief Create one landmark
         *
         * Implement in derived classes to build the type of landmark you need for this tracker.
         */
        virtual LandmarkBase* createLandmark(FeatureBase* _feature_ptr) = 0;

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
        virtual ConstraintBase* createConstraint(FeatureBase* _feature_ptr, NodeBase* _node_ptr) = 0;



        // From now on, all functions are already implemented.

    public:

        /** \brief Full processing of an incoming Capture.
         *
         * Usually you do not need to overload this method in derived classes.
         * Overload it only if you want to alter this algorithm.
         */
        virtual void process(CaptureBase* const _incoming_ptr);

        /** \brief Initialize tracker.
         */
        void init(CaptureBase* _origin_ptr);

        virtual bool voteForKeyFrame();

//        // getters and setters
//        CaptureBase* getOriginPtr() const;
//        CaptureBase* getLastPtr() const;
//        CaptureBase* getIncomingPtr() const;
//        void setOriginPtr(CaptureBase* const _origin_ptr);
//        void setLastPtr(CaptureBase* const _last_ptr);
//        void setIncomingPtr(CaptureBase* const _incoming_ptr);
        FeatureBaseList& getNewFeaturesList();
        void addNewFeature(FeatureBase* _feature_ptr);
        //
        FeatureBaseList& getNewFeaturesListIncoming();
        void addNewFeatureIncoming(FeatureBase* _feature_ptr);

    protected:

        /**\brief Process new Features
         *
         */
        virtual unsigned int processNew();

        /**\brief Make a KeyFrame using the provided Capture.
         */
        virtual void makeKeyFrame();


};

inline FeatureBaseList& ProcessorTrackerFeature::getNewFeaturesList()
{
    return new_features_list_last_;
}

inline void ProcessorTrackerFeature::addNewFeature(FeatureBase* _feature_ptr)
{
    new_features_list_last_.push_back(_feature_ptr);
}

inline FeatureBaseList& ProcessorTrackerFeature::getNewFeaturesListIncoming()
{
    return new_features_list_incoming_;
}

inline void ProcessorTrackerFeature::addNewFeatureIncoming(FeatureBase* _feature_ptr)
{
    new_features_list_incoming_.push_back(_feature_ptr);
}

inline void ProcessorTrackerFeature::init(CaptureBase* _origin_ptr)
{
    origin_ptr_ = _origin_ptr;
    last_ptr_ = _origin_ptr;
}

inline bool ProcessorTrackerFeature::voteForKeyFrame()
{
    return false;
}

//inline CaptureBase* ProcessorTrackerFeature::getOriginPtr() const
//{
//    return origin_ptr_;
//}
//
//inline CaptureBase* ProcessorTrackerFeature::getLastPtr() const
//{
//    return last_ptr_;
//}
//
//inline CaptureBase* ProcessorTrackerFeature::getIncomingPtr() const
//{
//    return incoming_ptr_;
//}
//
//inline void ProcessorTrackerFeature::setOriginPtr(CaptureBase* const _origin_ptr)
//{
//    origin_ptr_ = _origin_ptr;
//}
//
//inline void ProcessorTrackerFeature::setLastPtr(CaptureBase* const _last_ptr)
//{
//    last_ptr_ = _last_ptr;
//}
//
//inline void ProcessorTrackerFeature::setIncomingPtr(CaptureBase* const _incoming_ptr)
//{
//    incoming_ptr_ = _incoming_ptr;
//}

#endif /* PROCESSOR_TRACKER_FEATURE_H_ */
