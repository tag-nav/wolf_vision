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

class ProcessorTracker : public ProcessorBase
{
    protected:
        CaptureBase* origin_ptr_;    ///< Pointer to the origin of the tracker.
        CaptureBase* last_ptr_;      ///< Pointer to the last tracked capture.
        CaptureBase* incoming_ptr_;  ///< Pointer to the incoming capture being processed.
        FeatureBaseList new_features_list_last_; ///< List of new features in \b last for landmark initialization and new key-frame creation.
        FeatureBaseList new_features_list_incoming_; ///< list of the new features of \b last tracked in \b incoming

    public:
        ProcessorTracker(ProcessorType _tp);
        virtual ~ProcessorTracker();

        /** \brief Initialize tracker.
         */
        void init(CaptureBase* _origin_ptr);

        /** \brief Full processing of an incoming Capture.
         *
         * Usually you do not need to overload this method in derived classes.
         * Overload it only if you want to alter this algorithm.
         */
        virtual void process(CaptureBase* const _incoming_ptr);

        CaptureBase* getLastPtr();

    protected:

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
        virtual void advance();

        /**\brief Process new Features or Landmarks
         *
         */
        virtual unsigned int processNew() = 0;

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

        /**\brief Creates and adds constraints from last_ to origin_
         *
         */
        virtual void establishConstraints()=0;

        /**\brief Make a KeyFrame using the provided Capture.
         */
        virtual void makeKeyFrame();

        /** \brief Reset the tracker using the \b last Capture as the new \b origin.
         */
        void reset();

    protected:

        FeatureBaseList& getNewFeaturesListLast();

        void addNewFeatureLast(FeatureBase* _feature_ptr);

        FeatureBaseList& getNewFeaturesListIncoming();

        void addNewFeatureIncoming(FeatureBase* _feature_ptr);

    private:
        /** \brief Reset the tracker to a new \b origin and \b last Captures
         */
        void reset(CaptureBase* _origin_ptr, CaptureBase* _last_ptr);

};

inline void ProcessorTracker::init(CaptureBase* _origin_ptr)
{
    origin_ptr_ = _origin_ptr;
    last_ptr_ = _origin_ptr;
}

inline void ProcessorTracker::advance()
{
    last_ptr_->getFramePtr()->addCapture(incoming_ptr_); // Add incoming Capture to the tracker's Frame
    last_ptr_->destruct(); // TODO: JS->JV why this does not work?? Destruct now the obsolete last before reassigning a new pointer
    last_ptr_ = incoming_ptr_; // Incoming Capture takes the place of last Capture
    incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.
}

inline void ProcessorTracker::makeKeyFrame()
{
    // Create a new non-key Frame in the Trajectory with the incoming Capture
    FrameBase* new_frame_ptr = getWolfProblem()->createFrame(NON_KEY_FRAME, incoming_ptr_->getTimeStamp());
    new_frame_ptr->addCapture(incoming_ptr_); // Add incoming Capture to the new Frame
    // Make the last Capture's Frame a KeyFrame so that it gets into the solver
    last_ptr_->getFramePtr()->setKey();

    // Create constraints from last to origin
    establishConstraints();
}

inline void ProcessorTracker::reset()
{
    reset(last_ptr_, incoming_ptr_);
}

inline CaptureBase* ProcessorTracker::getLastPtr()
{
    return last_ptr_;
}

inline FeatureBaseList& ProcessorTracker::getNewFeaturesListLast()
{
    return new_features_list_last_;
}

inline void ProcessorTracker::addNewFeatureLast(FeatureBase* _feature_ptr)
{
    new_features_list_last_.push_back(_feature_ptr);
}

inline FeatureBaseList& ProcessorTracker::getNewFeaturesListIncoming()
{
    return new_features_list_incoming_;
}

inline void ProcessorTracker::addNewFeatureIncoming(FeatureBase* _feature_ptr)
{
    new_features_list_incoming_.push_back(_feature_ptr);
}

inline void ProcessorTracker::reset(CaptureBase* _origin_ptr, CaptureBase* _last_ptr)
{
    origin_ptr_ = _origin_ptr;
    last_ptr_ = _last_ptr;
    incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.
}

#endif /* SRC_PROCESSOR_TRACKER_H_ */
