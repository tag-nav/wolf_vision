/*
 * ProcessorTracker.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: jvallve
 */

// wolf
#include "base/processor/processor_tracker.h"

// std
#include <cmath>

namespace wolf
{

ProcessorTracker::ProcessorTracker(const std::string& _type,
                                   ProcessorParamsTrackerPtr _params_tracker) :
        ProcessorBase(_type, _params_tracker),
        params_tracker_(_params_tracker),
        processing_step_(FIRST_TIME_WITHOUT_PACK),
        origin_ptr_(nullptr),
        last_ptr_(nullptr),
        incoming_ptr_(nullptr),
        number_of_tracks_(0)
{
    //
}

ProcessorTracker::~ProcessorTracker()
{
    //
}

void ProcessorTracker::process(CaptureBasePtr const _incoming_ptr)
{
    using std::abs;

    if (_incoming_ptr == nullptr)
    {
        WOLF_ERROR("Received capture is nullptr.");
        return;
    }

    incoming_ptr_ = _incoming_ptr;

    preProcess(); // Derived class operations

    computeProcessingStep();

    switch(processing_step_)
    {
        case FIRST_TIME_WITH_PACK :
        {
            PackKeyFramePtr pack = kf_pack_buffer_.selectPack( incoming_ptr_, params_tracker_->time_tolerance);
            kf_pack_buffer_.removeUpTo( incoming_ptr_->getTimeStamp() );

            WOLF_DEBUG( "PT ", getName(), ": KF" , pack->key_frame->id() , " callback unpacked with ts= " , pack->key_frame->getTimeStamp().get() );

            // Append incoming to KF
            pack->key_frame->addCapture(incoming_ptr_);

            // Process info
            // TrackerFeature:  We only process new features in Last, here last = nullptr, so we do not have anything to do.
            // TrackerLandmark: If we have given a map, all landmarks in the map are know. Process them.
            processKnown();

            // Update pointers
            resetDerived();
            origin_ptr_ = incoming_ptr_;
            last_ptr_   = incoming_ptr_;
            incoming_ptr_ = nullptr;

            break;
        }
        case FIRST_TIME_WITHOUT_PACK :
        {
            FrameBasePtr kfrm = getProblem()->emplaceFrame(KEY_FRAME, incoming_ptr_->getTimeStamp());
            kfrm->addCapture(incoming_ptr_);

            // Process info
            processKnown();
            // We only process new features in Last, here last = nullptr, so we do not have anything to do.

            // Issue KF callback with new KF
            getProblem()->keyFrameCallback(kfrm, shared_from_this(), params_tracker_->time_tolerance);

            // Update pointers
            resetDerived();
            origin_ptr_ = incoming_ptr_;
            last_ptr_   = incoming_ptr_;
            incoming_ptr_ = nullptr;

            break;
        }
        case SECOND_TIME_WITH_PACK :
        {
        	// No-break case only for debug. Next case will be executed too.
            PackKeyFramePtr pack = kf_pack_buffer_.selectPack( incoming_ptr_, params_tracker_->time_tolerance);
            WOLF_DEBUG( "PT ", getName(), ": KF" , pack->key_frame->id() , " callback unpacked with ts= " , pack->key_frame->getTimeStamp().get() );
        } // @suppress("No break at end of case")
        case SECOND_TIME_WITHOUT_PACK :
        {
            FrameBasePtr frm = getProblem()->emplaceFrame(NON_KEY_FRAME, incoming_ptr_->getTimeStamp());
            frm->addCapture(incoming_ptr_);

            // We have a last_ Capture with no features, so we do not process known features, and we do not vote for KF.

            // Process info
            processKnown();
            processNew(params_tracker_->max_new_features);

            // Establish factors
            establishFactors();

            // Update pointers
            resetDerived();
            origin_ptr_ = last_ptr_;
            last_ptr_   = incoming_ptr_;
            incoming_ptr_ = nullptr;

            break;
        }
        case RUNNING_WITH_PACK :
        {
            PackKeyFramePtr pack = kf_pack_buffer_.selectPack( last_ptr_ , params_tracker_->time_tolerance);
            kf_pack_buffer_.removeUpTo( last_ptr_->getTimeStamp() );

            WOLF_DEBUG( "PT ", getName(), ": KF" , pack->key_frame->id() , " callback unpacked with ts= " , pack->key_frame->getTimeStamp().get() );

            processKnown();

            // Capture last_ is added to the new keyframe
            FrameBasePtr last_old_frame = last_ptr_->getFrame();
            last_old_frame->unlinkCapture(last_ptr_);
            last_old_frame->remove();
            pack->key_frame->addCapture(last_ptr_);

            // Create new frame
            FrameBasePtr frm = getProblem()->emplaceFrame(NON_KEY_FRAME, incoming_ptr_->getTimeStamp());
            frm->addCapture(incoming_ptr_);

            // Detect new Features, initialize Landmarks, create Factors, ...
            processNew(params_tracker_->max_new_features);

            // Establish factors
            establishFactors();

            // Update pointers
            resetDerived();
            origin_ptr_ = last_ptr_;
            last_ptr_   = incoming_ptr_;
            incoming_ptr_ = nullptr;

            break;
        }
        case RUNNING_WITHOUT_PACK :
        {
            processKnown();

            // eventually add more features
            if (last_ptr_->getFeatureList().size() < params_tracker_->min_features_for_keyframe)
            {
                WOLF_TRACE("Adding more features...");
                processNew(params_tracker_->max_new_features);
            }

            if (voteForKeyFrame() && permittedKeyFrame())
            {
                // We create a KF

                // set KF on last
                last_ptr_->getFrame()->setState(getProblem()->getState(last_ptr_->getTimeStamp()));
                last_ptr_->getFrame()->setKey();

                // make F; append incoming to new F
                FrameBasePtr frm = getProblem()->emplaceFrame(NON_KEY_FRAME, incoming_ptr_->getTimeStamp());
                frm->addCapture(incoming_ptr_);

                // process
                processNew(params_tracker_->max_new_features);

                //                // Set key
                //                last_ptr_->getFrame()->setKey();
                //
                // Set state to the keyframe
                last_ptr_->getFrame()->setState(getProblem()->getState(last_ptr_->getTimeStamp()));

                // Establish factors
                establishFactors();

                // Call the new keyframe callback in order to let the other processors to establish their factors
                getProblem()->keyFrameCallback(last_ptr_->getFrame(), std::static_pointer_cast<ProcessorBase>(shared_from_this()), params_tracker_->time_tolerance);

                // Update pointers
                resetDerived();
                origin_ptr_ = last_ptr_;
                last_ptr_   = incoming_ptr_;
                incoming_ptr_ = nullptr;

            }
            else
            {
                // We do not create a KF

                // Advance this
                last_ptr_->getFrame()->addCapture(incoming_ptr_); // Add incoming Capture to the tracker's last Frame
                last_ptr_->remove();
                incoming_ptr_->getFrame()->setTimeStamp(incoming_ptr_->getTimeStamp());

                // Update pointers
                advanceDerived();
                last_ptr_   = incoming_ptr_;
                incoming_ptr_ = nullptr;
            }
            break;
        }
        default :
            break;
    }

    number_of_tracks_ = last_ptr_->getFeatureList().size();
    postProcess();
}

void ProcessorTracker::computeProcessingStep()
{
    // First determine the processing phase by checking the status of the tracker pointers
    enum {FIRST_TIME, SECOND_TIME, RUNNING} step;
    if (origin_ptr_ == nullptr && last_ptr_ == nullptr)
        step = FIRST_TIME;
    else if (origin_ptr_ == last_ptr_)
        step = SECOND_TIME;
    else
        step = RUNNING;

    // Then combine with the existence (or not) of a keyframe callback pack
    switch (step)
    {
        case FIRST_TIME :

            if (kf_pack_buffer_.selectPack(incoming_ptr_, params_tracker_->time_tolerance))
                processing_step_ = FIRST_TIME_WITH_PACK;
            else // ! last && ! pack(incoming)
                processing_step_ = FIRST_TIME_WITHOUT_PACK;
        break;

        case SECOND_TIME :

            if (kf_pack_buffer_.selectPack(last_ptr_, params_tracker_->time_tolerance))
                processing_step_ = SECOND_TIME_WITH_PACK;
            else
                processing_step_ = SECOND_TIME_WITHOUT_PACK;
            break;

        case RUNNING :
        default :

            if (kf_pack_buffer_.selectPack(last_ptr_, params_tracker_->time_tolerance))
            {
                if (last_ptr_->getFrame()->isKey())
                {
                    WOLF_WARN("||*||");
                    WOLF_INFO(" ... It seems you missed something!");
                    WOLF_INFO("Pack's KF and last's KF have matching time stamps (i.e. below time tolerances)");
                    WOLF_INFO("Check the following for correctness:");
                    WOLF_INFO("  - You have all processors installed before starting receiving any data");
                    WOLF_INFO("  - You issued a problem->setPrior() after all processors are installed ---> ", (getProblem()->priorIsSet() ? "OK" : "NOK"));
                    WOLF_INFO("  - You have configured all your processors with compatible time tolerances");
                    WOLF_ERROR("Pack's KF and last's KF have matching time stamps (i.e. below time tolerances).");
                }
                processing_step_ = RUNNING_WITH_PACK;
            }
            else
                processing_step_ = RUNNING_WITHOUT_PACK;
            break;
    }
}

} // namespace wolf

