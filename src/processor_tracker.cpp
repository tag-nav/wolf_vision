/*
 * ProcessorTracker.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: jvallve
 */

// wolf
#include "processor_tracker.h"

// std
#include <cmath>

namespace wolf
{

ProcessorTracker::ProcessorTracker(const std::string& _type, const unsigned int _max_new_features, const Scalar& _time_tolerance) :
        ProcessorBase(_type, _time_tolerance),
        processing_step_(FIRST_TIME_WITHOUT_PACK),
        origin_ptr_(nullptr),
        last_ptr_(nullptr),
        incoming_ptr_(nullptr),
        max_new_features_(_max_new_features)
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
            KFPackPtr pack = selectPack( incoming_ptr_);
            kf_pack_buffer_.removeUpTo( incoming_ptr_->getTimeStamp() );

            WOLF_DEBUG( "PT: KF" , pack->key_frame->id() , " callback received with ts= " , pack->key_frame->getTimeStamp().get() );

            // Append incoming to KF
            pack->key_frame->addCapture(incoming_ptr_);

            // Process info
            // We only process new features in Last, here last = nullptr, so we do not have anything to do.

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
            // We only process new features in Last, here last = nullptr, so we do not have anything to do.

            // Issue KF callback with new KF
            getProblem()->keyFrameCallback(kfrm, shared_from_this(), time_tolerance_);

            // Update pointers
            resetDerived();
            origin_ptr_ = incoming_ptr_;
            last_ptr_   = incoming_ptr_;
            incoming_ptr_ = nullptr;

            break;
        }
        case SECOND_TIME_WITH_PACK :
        case SECOND_TIME_WITHOUT_PACK :
        {
            FrameBasePtr frm = getProblem()->emplaceFrame(NON_KEY_FRAME, incoming_ptr_->getTimeStamp());
            frm->addCapture(incoming_ptr_);

            // We have a last_ Capture with no features, so we do not process known features, and we do not vote for KF.

            // Process info
            processNew(max_new_features_);

            // Update pointers
            resetDerived();
            origin_ptr_ = last_ptr_;
            last_ptr_   = incoming_ptr_;
            incoming_ptr_ = nullptr;

            break;
        }
        case RUNNING_WITH_PACK :
        {
            KFPackPtr pack = selectPack( last_ptr_ );
            kf_pack_buffer_.removeUpTo( last_ptr_->getTimeStamp() );

            WOLF_DEBUG( "PT: KF" , pack->key_frame->id() , " callback received at ts= " , pack->key_frame->getTimeStamp().get() );

            processKnown();

            // Capture last_ is added to the new keyframe
            FrameBasePtr last_old_frame = last_ptr_->getFramePtr();
            last_old_frame->unlinkCapture(last_ptr_);
            last_old_frame->remove();
            pack->key_frame->addCapture(last_ptr_);

            // Create new frame
            FrameBasePtr frm = getProblem()->emplaceFrame(NON_KEY_FRAME, incoming_ptr_->getTimeStamp());
            frm->addCapture(incoming_ptr_);

            // Detect new Features, initialize Landmarks, create Constraints, ...
            processNew(max_new_features_);

            // Establish constraints
            establishConstraints();

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

            if (voteForKeyFrame() && permittedKeyFrame())
            {
                // We create a KF

                // set KF on last
                last_ptr_->getFramePtr()->setKey();

                // make F; append incoming to new F
                FrameBasePtr frm = getProblem()->emplaceFrame(NON_KEY_FRAME, incoming_ptr_->getTimeStamp());
                frm->addCapture(incoming_ptr_);

                // process
                processNew(max_new_features_);

                // Set key
                last_ptr_->getFramePtr()->setKey();

                // Set state to the keyframe
                last_ptr_->getFramePtr()->setState(getProblem()->getState(last_ptr_->getTimeStamp()));

                // Establish constraints
                establishConstraints();

                // Call the new keyframe callback in order to let the other processors to establish their constraints
                getProblem()->keyFrameCallback(last_ptr_->getFramePtr(), std::static_pointer_cast<ProcessorBase>(shared_from_this()), time_tolerance_);

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
                last_ptr_->getFramePtr()->addCapture(incoming_ptr_); // Add incoming Capture to the tracker's last Frame
                last_ptr_->remove();
                incoming_ptr_->getFramePtr()->setTimeStamp(incoming_ptr_->getTimeStamp());

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

            if (selectPack(incoming_ptr_))
                processing_step_ = FIRST_TIME_WITH_PACK;
            else // ! last && ! pack(incoming)
                processing_step_ = FIRST_TIME_WITHOUT_PACK;
        break;

        case SECOND_TIME :

            if (selectPack(last_ptr_))
                processing_step_ = SECOND_TIME_WITH_PACK;
            else
                processing_step_ = SECOND_TIME_WITHOUT_PACK;
            break;

        case RUNNING :
        default :

            if (selectPack(last_ptr_))
            {
                if (last_ptr_->getFramePtr()->isKey())
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

