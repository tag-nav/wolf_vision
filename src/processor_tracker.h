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
 *   - Init the tracker with an \b origin Capture: init(CaptureBase* origin_ptr);
 *   - On each incoming Capture,
 *     - Track known features in the \b incoming Capture: track(CaptureBase* incoming_ptr);
 *     - Check if enough Features are still tracked, and vote for a new KeyFrame if this number is too low:
 *     - if voteForKeyFrame()
 *       - Make a KeyFrame with the \b last Capture: makeKeyFrame();
 *       - Look for new Features and make Landmarks with them,
 *       - if detectNewFeatures(last)
 *         - initNewLandmarks(last)
 *       - Reset the tracker with the \b last Capture as the new \b origin: reset();
 *     - else
 *       - Advance the tracker one Capture ahead: advance()
 *
 * This functionality exists by default in the virtual method process(). You can overload it at your convenience.
 */
class ProcessorTracker : public ProcessorBase
{
    public:

        ProcessorTracker(bool _autonomous = true);
        virtual ~ProcessorTracker();

        /** \brief Initialize tracker.
         */
        void init(CaptureBase* _origin_ptr);

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
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame() = 0;

        /**\brief Mark the frame of the last Capture as KeyFrame.
         *
         * This function only marks the KeyFrame property of the Frame.
         * It does not do anything else with the frame or with the Tracker.
         */
        virtual void markKeyFrame();

        /** \brief Reset the tracker to a new \b origin and \b last Captures
         */
        void reset(CaptureBase* _origin_ptr, CaptureBase* _last_ptr);

        /** \brief Reset the tracker using the \b last Capture as the new \b origin.
         */
        void reset();

        /** \brief Advance the incoming Capture to become the last.
         *
         * Call this when the tracking and keyframe policy work is done and
         * we need to get ready to accept a new incoming Capture.
         */
        void advance();

        /** \brief Full processing of an incoming Capture.
         *
         * Overload this function in derived trackers if desired.
         */
        virtual void process(CaptureBase* const _incoming_ptr);


//        // TODO see what to do with this prototype from ProcessBase
//        virtual void extractFeatures(CaptureBase* _capture_ptr)
//        {
//        }
//        // TODO see what to do with this prototype from ProcessBase
//        virtual void establishConstraints(CaptureBase* _capture_ptr)
//        {
//            track(_capture_ptr);
//        }


        // getters and setters // TODO hide some of these in protected or private
        bool isAutonomous() const;
        CaptureBase* getOriginPtr() const;
        CaptureBase* getLastPtr() const;
        CaptureBase* getIncomingPtr() const;
        void setOriginPtr(CaptureBase* const _origin_ptr);
        void setLastPtr(CaptureBase* const _last_ptr);
        void setIncomingPtr(CaptureBase* const _incoming_ptr);

    protected:

    private:
        bool autonomous_;    ///< Sets whether the tracker is autonomous to make decisions that affect the WolfProblem, like creating new KeyFrames and/or Landmarks.
        CaptureBase* origin_ptr_;    ///< Pointer to the origin of the tracker.
        CaptureBase* last_ptr_;      ///< Pointer to the last tracked capture.
        CaptureBase* incoming_ptr_;  ///< Pointer to the incoming capture being processed.
};

// IMPLEMENTATION //

inline void ProcessorTracker::init(CaptureBase* _origin_ptr)
{
    origin_ptr_ = _origin_ptr;
    last_ptr_ = _origin_ptr;
}

inline void ProcessorTracker::markKeyFrame()
{
    // TODO: check how Frames are managed from Tracker, and where are they kept (in Trajectory, or in Tracker, or nowhere)
    // TODO Check all that needs to be done: To the Frame, to the Capture, to the Constraints.
    // Here we opt for marking the owner frame of the last Capture as KEY_FRAME.
    // Someone has to take care of creating a new Frame for the new incoming Captures that will start to arrive...
    // Maybe here we could already create a new Frame to store this KeyFrame, and keep the Tracker's Frame always alive...
    getLastPtr()->getFramePtr()->setType(KEY_FRAME);
    reset(); // last Capture becomes origin Capture.
}

inline void ProcessorTracker::reset(CaptureBase* _origin_ptr, CaptureBase* _last_ptr)
{
    origin_ptr_ = _origin_ptr;
    last_ptr_ = _last_ptr;
    incoming_ptr_ = nullptr;
}

inline void ProcessorTracker::reset()
{
    reset(last_ptr_, incoming_ptr_);
}

inline void ProcessorTracker::advance()
{
    // TODO: check how Frames are managed from Tracker, and where are they kept (in Trajectory, or in Tracker, or nowhere)
    // Here we opt for keeping the owner frame, adding to it the incoming Capture and destructing the last.
    last_ptr_->getFramePtr()->addCapture(incoming_ptr_); // Add incoming Capture to the tracker's Frame
    last_ptr_->destruct();     // Destruct obsolete last before reassigning a new pointer
    last_ptr_ = incoming_ptr_; // Incoming Capture takes the place of last Capture
    incoming_ptr_ = nullptr;   // This line is not really needed, but it make things clearer.
}

inline bool ProcessorTracker::isAutonomous() const
{
    return autonomous_;
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
