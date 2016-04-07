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

    protected:
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
};

#endif /* SRC_PROCESSOR_TRACKER_H_ */
