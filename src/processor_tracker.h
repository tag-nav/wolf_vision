/*
 * ProcessorTracker.h
 *
 *  Created on: Apr 7, 2016
 *      Author: jvallve
 */

#ifndef SRC_PROCESSOR_TRACKER_H_
#define SRC_PROCESSOR_TRACKER_H_

#include "processor_base.h"
#include "capture_base.h"

namespace wolf {

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
        void init(CaptureBase* _origin_ptr)
        {
            origin_ptr_ = _origin_ptr;
            last_ptr_ = _origin_ptr;
        }

        /** \brief Reset the tracker to a new \b origin and \b last Captures
         */
        void reset(CaptureBase* _origin_ptr, CaptureBase* _last_ptr)
        {
            origin_ptr_ = _origin_ptr;
            last_ptr_ = _last_ptr;
            incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.
        }

        /** \brief Reset the tracker using the \b last Capture as the new \b origin.
         */
        void reset()
        {
            reset(last_ptr_, incoming_ptr_);
        }

        FeatureBaseList& getNewFeaturesList()
        {
            return new_features_list_last_;
        }

        void addNewFeature(FeatureBase* _feature_ptr)
        {
            new_features_list_last_.push_back(_feature_ptr);
        }

        FeatureBaseList& getNewFeaturesListIncoming()
        {
            return new_features_list_incoming_;
        }

        void addNewFeatureIncoming(FeatureBase* _feature_ptr)
        {
            new_features_list_incoming_.push_back(_feature_ptr);
        }

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

        /** \brief Advance the incoming Capture to become the last.
         *
         * Call this when the tracking and keyframe policy work is done and
         * we need to get ready to accept a new incoming Capture.
         */
        void advance()
        {
            last_ptr_->getFramePtr()->addCapture(incoming_ptr_); // Add incoming Capture to the tracker's Frame
            //    last_ptr_->destruct(); // TODO: JS->JV why this does not work?? Destruct now the obsolete last before reassigning a new pointer
            last_ptr_ = incoming_ptr_; // Incoming Capture takes the place of last Capture
            incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.
        }

        CaptureBase* getLastPtr(){return last_ptr_;}

        /** \brief Full processing of an incoming Capture.
         *
         * Usually you do not need to overload this method in derived classes.
         * Overload it only if you want to alter this algorithm.
         */
        virtual void process(CaptureBase* const _incoming_ptr)
        {
            // 1. First we track the known Features and create new constraints as needed
            incoming_ptr_ = _incoming_ptr;
            processKnown();
            // 2. Then we see if we want and we are allowed to create a KeyFrame
            if (voteForKeyFrame() && permittedKeyFrame())
            {
                // 2.a. Detect new Features, initialize Landmarks, create Constraints
                processNew();
                // Make KeyFrame
                makeKeyFrame();
                // Reset the Tracker
                reset();
            }
            else
            {
                // We did not create a KeyFrame:
                // 2.b. Update the tracker's last and incoming pointers one step ahead
                advance();
            }
        }

        /**\brief Make a KeyFrame using the provided Capture.
         */
        virtual void makeKeyFrame()
        {
            // Create a new non-key Frame in the Trajectory with the incoming Capture
            getWolfProblem()->createFrame(NON_KEY_FRAME, incoming_ptr_->getTimeStamp());
            getWolfProblem()->getLastFramePtr()->addCapture(incoming_ptr_); // Add incoming Capture to the new Frame
            // Make the last Capture's Frame a KeyFrame so that it gets into the solver
            last_ptr_->getFramePtr()->setKey();
        }

};

} // namespace wolf

#endif /* SRC_PROCESSOR_TRACKER_H_ */
