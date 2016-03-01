/*
 * \processor_tracker.h
 *
 *  Created on: 27/02/2016
 *      \author: jsola
 */

#ifndef PROCESSOR_TRACKER_H_
#define PROCESSOR_TRACKER_H_

#include "processor_base.h"
#include "capture_base.h"

/** \brief General tracker processor
 *
 * This class implements the incremental tracker. It contains three pointers to three Captures of type CaptureBase, named \b origin, \b last and \b incoming:
 *   - \b origin: this points to a Capture where all Feature tracks start.
 *   - \b last: the last Capture tracked by the tracker. A sufficient subset of the Features in \b origin is still alive in \b last.
 *   - \b incoming: the capture being received. The tracker operates on this Capture,
 *   establishing correspondences between the features here and the features in \b origin. Each successful correspondence
 *   results in an extension of the track of the Feature up to the \b incoming Capture.
 *
 * The pipeline of actions for a tracker like this can be resumed as follows:
 *   - Reset the tracker with an \b origin Capture: reset(CaptureBase* origin_ptr);
 *   - On each incoming Capture,
 *     - Track features in an \b incoming Capture: track(CaptureBase* incoming_ptr);
 *     - Check if enough Features are still tracked, and vote for a new KeyFrame if this number is too low: voteForKeyFrame();
 *     - if voteForKeyFrame()
 *       - Mark the Frame owning the \b last Capture as KeyFrame: markKeyFrame();
 *       - Reset the tracker with the \b last Capture as the new \b origin: reset();
 *     - else
 *       - Advance the tracker one Capture ahead: advance()
 */
class ProcessorTracker : public ProcessorBase
{
    public:
        ProcessorTracker(unsigned int _min_nbr_of_tracks_for_keyframe);
        virtual ~ProcessorTracker();

        /** \brief Reset the tracker to a new origin Capture
         */
        void reset(CaptureBase* _origin_ptr);

        /** \brief Reset the tracker using the \b last Capture
         */
        void reset();

        /** \brief Tracker function
         *
         * This is the tracker function to be implemented in derived classes. It operates on the incoming capture.
         *
         * This should do one of the following, depending on the design of the tracker:
         *   - Track Features against other Features in another Capture.
         *   - Track Features against Landmarks in the Map.
         *
         * It should also generate the necessary Features in the incoming Capture, of a type derived from FeatureBase,
         * and the constraints, of a type derived from ConstraintBase.
         */
        virtual void track(CaptureBase* _incoming_ptr) = 0;

        /**\brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * The criterion is evaluated on the \b incoming Capture, if the number of active tracks
         * falls below the #min_tracks_th_ threshold.
         * The Keyframe will be generated using the \b last Capture.
         * This is so because usually the \b incoming Capture is a bad KeyFrame, and this is detectable.
         * Then, the \b last Capture was still a good KeyFrame, and so it is used for KeyFrame generation.
         *
         * You can create other voting policies by overloading this function in derived classes.
         *
         * WARNING! This function only votes!
         * An external agent decides then if the keyframe is effectively created.
         * New keyframe generation should be notified to the tracker via markNewKeyFrame().
         */
        bool voteForKeyFrame();

        /**\brief Mark the frame of the last Capture as KeyFrame.
         *
         * This function only marks the KeyFrame property of the Frame.
         * It does not do anything else with the frame or with the Tracker.
         */
        virtual void markKeyFrame();

        /** \brief Advance the incoming Capture to become the last.
         *
         * Call this when the tracking and keyframe policy work is done and
         * we need to get ready to accept a new incoming Capture.
         */
        void advance();


        // TODO see what to do with this prototype from ProcessBase
        virtual void extractFeatures(CaptureBase* _capture_ptr);
        // TODO see what to do with this prototype from ProcessBase
        virtual void establishConstraints(CaptureBase* _capture_ptr);


        // getters and setters // TODO hide some of these in protected or private
        CaptureBase* getOriginPtr() const;
        CaptureBase* getLastPtr() const;
        CaptureBase* getIncomingPtr() const;
        void setOriginPtr(CaptureBase* const _origin_ptr);
        void setLastPtr(CaptureBase* const _last_ptr);
        void setIncomingPtr(CaptureBase* const _incoming_ptr);

    protected:

    private:
        CaptureBase* origin_ptr_;    ///< Pointer to the origin of the tracker.
        CaptureBase* last_ptr_;      ///< Pointer to the last tracked capture.
        CaptureBase* incoming_ptr_;  ///< Pointer to the incoming capture being processed.
        unsigned int min_tracks_th_; ///< Threshold on the number of active tracks to vote for keyframe generation.

};

// IMPLEMENTATION //

inline void ProcessorTracker::reset(CaptureBase* _origin_ptr)
{
    origin_ptr_ = _origin_ptr;
    last_ptr_ = origin_ptr_;
    incoming_ptr_ = nullptr;
}

inline void ProcessorTracker::reset()
{
    reset(last_ptr_);
}

inline bool ProcessorTracker::voteForKeyFrame()
{
    unsigned int n = getIncomingPtr()->getFeatureListPtr()->size(); // Number of features in Incoming Capture
    return (n < min_tracks_th_);
    return false;
}

inline void ProcessorTracker::markKeyFrame()
{
    getLastPtr()->getFramePtr()->setType(KEY_FRAME);
    //TODO Check all that needs to be done: To the Frame, to the Capture, to the Constraints
    reset(last_ptr_); // last Capture becomes origin Capture.
}

inline void ProcessorTracker::advance()
{
    // TODO: check for memory leaks if last does not get deleted.
    // TODO: check for destruction policy.
    last_ptr_->getFramePtr()->destruct();
    last_ptr_ = incoming_ptr_; // incoming Capture becomes last Capture
    incoming_ptr_ = nullptr;
}

inline CaptureBase* ProcessorTracker::getOriginPtr() const
{
    return origin_ptr_;
}

inline CaptureBase* ProcessorTracker::getLastPtr() const
{
    return last_ptr_;
}

inline CaptureBase* ProcessorTracker::getIncomingPtr() const
{
    return incoming_ptr_;
}

inline void ProcessorTracker::setOriginPtr(CaptureBase* const _origin_ptr)
{
    origin_ptr_ = _origin_ptr;
}

inline void ProcessorTracker::setLastPtr(CaptureBase* const _last_ptr)
{
    last_ptr_ = _last_ptr;
}

inline void ProcessorTracker::setIncomingPtr(CaptureBase* const _incoming_ptr)
{
    incoming_ptr_ = _incoming_ptr;
}

#endif /* PROCESSOR_TRACKER_H_ */
