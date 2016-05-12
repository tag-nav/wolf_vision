/*
 * processor_tracker.h
 *
 *  Created on: Apr 7, 2016
 *      Author: jvallve
 */

#ifndef SRC_PROCESSOR_TRACKER_H_
#define SRC_PROCESSOR_TRACKER_H_

#include "processor_base.h"
#include "capture_base.h"

namespace wolf {

struct ProcessorParamsTracker : public ProcessorParamsBase
{
        unsigned int max_new_features;
};

/** \brief General tracker processor
 *
 * This is an abstract class.
 *
 * This class implements the incremental tracker, that can be used to track either Features in
 * other Captures, or Landmarks in a Map, through two derived classes,
 *   - ProcessorTrackerFeature,
 *   - ProcessorTrackerLandmark.
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
 * It establishes constraints Feature-Feature or Feature-Landmark;
 * Implement these options in two separate derived classes:
 *   - ProcessorTrackerFeature : for Feature-Feature correspondences (no landmarks)
 *   - ProcessorTrackerLandmark : for Feature-Landmark correspondences (with landmarks)
 *
 * The pipeline of actions for an autonomous tracker can be resumed as follows (see process() for full detail).
 * We highlight the functions to be implemented by derived classes with the sign '<=== IMPLEMENT'
 *   - On each incoming Capture,
 *     - Track known features in the \b incoming Capture: processKnown()            <=== IMPLEMENT
 *     - Check if enough Features are still tracked, and vote for a new KeyFrame if this number is too low:
 *     - if voteForKeyFrame()                                                       <=== IMPLEMENT
 *       - Populate the tracker with new Features : processNew()                    <=== IMPLEMENT
 *       - Make a KeyFrame with the \b last Capture: makeFrame(), setKey()
 *       - Establish constraints of the new Features: establishConstraints()        <=== IMPLEMENT
 *       - Reset the tracker with the \b last Capture as the new \b origin: reset() <=== IMPLEMENT
 *     - else
 *       - Advance the tracker one Capture ahead: advance()                         <=== IMPLEMENT
 *
 * This functionality exists by default in the virtual method process().
 * Should you need extra functionality for your derived types, you can overload the two methods,
 *
 *   -  preProcess() { }
 *   -  postProcess() { }
 *
 * which are called at the beginning and at the end of process(). See the doc of these functions for more info.
 */
class ProcessorTracker : public ProcessorBase
{
    protected:
        CaptureBase* origin_ptr_;    ///< Pointer to the origin of the tracker.
        CaptureBase* last_ptr_;      ///< Pointer to the last tracked capture.
        CaptureBase* incoming_ptr_;  ///< Pointer to the incoming capture being processed.
        FeatureBaseList new_features_last_; ///< List of new features in \b last for landmark initialization and new key-frame creation.
        FeatureBaseList new_features_incoming_; ///< list of the new features of \b last successfully tracked in \b incoming
        unsigned int max_new_features_; ///< max features alowed to detect in one iteration. 0 = no limit
        Scalar time_tolerance_;         ///< self time tolerance for adding a capture into a frame

    public:
        ProcessorTracker(ProcessorType _tp, const unsigned int _max_new_features = 0, const Scalar& _time_tolerance = 0);
        virtual ~ProcessorTracker();

        /** \brief Full processing of an incoming Capture.
         *
         * Usually you do not need to overload this method in derived classes.
         * Overload it only if you want to alter this algorithm.
         */
        virtual void process(CaptureBase* const _incoming_ptr);

        void setMaxNewFeatures(const unsigned int& _max_new_features);
        const unsigned int getMaxNewFeatures();

        virtual bool keyFrameCallback(FrameBase* _keyframe_ptr, const Scalar& _dt);

        virtual CaptureBase* getLastPtr();

    protected:
        /** Pre-process incoming Capture
         *
         * This is called by process() just after assigning incoming_ptr_ to a valid Capture.
         *
         * Overload this function to prepare stuff on derived classes.
         *
         * Typical uses of prePrecess() are:
         *   - casting base types to derived types
         *   - initializing counters, flags, or any derived variables
         *   - initializing algorithms needed for processing the derived data
         */
        virtual void preProcess() { };

        /** Post-process
         *
         * This is called by process() after finishing the processing algorithm.
         *
         * Overload this function to post-process stuff on derived classes.
         *
         * Typical uses of postPrecess() are:
         *   - resetting and/or clearing variables and/or algorithms at the end of processing
         *   - drawing / printing / logging the results of the processing
         */
        virtual void postProcess() { };

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

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame() = 0;

        /** \brief Advance the incoming Capture to become the last.
         *
         * Call this when the tracking and keyframe policy work is done and
         * we need to get ready to accept a new incoming Capture.
         */
        virtual void advance() = 0;

        /**\brief Process new Features or Landmarks
         *
         */
        virtual unsigned int processNew(const unsigned int& _max_features) = 0;

        /**\brief Creates and adds constraints from last_ to origin_
         *
         */
        virtual void establishConstraints()=0;

        /**\brief make a non-key Frame with the provided Capture
         */
        void makeFrame(CaptureBase* _capture_ptr, FrameKeyType _type = NON_KEY_FRAME);

        /** \brief Reset the tracker using the \b last Capture as the new \b origin.
         */
        virtual void reset() = 0;

    protected:

        FeatureBaseList& getNewFeaturesListLast();

        void addNewFeatureLast(FeatureBase* _feature_ptr);

        FeatureBaseList& getNewFeaturesListIncoming();

        void addNewFeatureIncoming(FeatureBase* _feature_ptr);
};

inline void ProcessorTracker::setMaxNewFeatures(const unsigned int& _max_new_features)
{
    max_new_features_ = _max_new_features;
}

inline const unsigned int ProcessorTracker::getMaxNewFeatures()
{
    return max_new_features_;
}

inline void ProcessorTracker::makeFrame(CaptureBase* _capture_ptr, FrameKeyType _type)
{
    // We need to create the new free Frame to hold what will become the last Capture
    FrameBase* new_frame_ptr = getProblem()->createFrame(_type, _capture_ptr->getTimeStamp());
    new_frame_ptr->addCapture(_capture_ptr); // Add incoming Capture to the new Frame
}

inline FeatureBaseList& ProcessorTracker::getNewFeaturesListLast()
{
    return new_features_last_;
}

inline void ProcessorTracker::addNewFeatureLast(FeatureBase* _feature_ptr)
{
    new_features_last_.push_back(_feature_ptr);
}

inline FeatureBaseList& ProcessorTracker::getNewFeaturesListIncoming()
{
    return new_features_incoming_;
}

inline void ProcessorTracker::addNewFeatureIncoming(FeatureBase* _feature_ptr)
{
    new_features_incoming_.push_back(_feature_ptr);
}

inline CaptureBase* ProcessorTracker::getLastPtr()
{
    return last_ptr_;
}

} // namespace wolf

#endif /* SRC_PROCESSOR_TRACKER_H_ */
