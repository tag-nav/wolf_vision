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
            // (1)
            // pack.KF.addCapture(incoming)
            // makeF; F.addCapture(incoming)
            // o <- i
            // l <- i

            KFPackPtr pack = selectPack( incoming_ptr_);
            kf_pack_buffer_.removeUpTo( incoming_ptr_->getTimeStamp() );

            // Append incoming to KF
            pack->key_frame->addCapture(incoming_ptr_);

            // TODO process info
            processNew(max_new_features_); // TODO not sure. Check code inside.

            origin_ptr_ = incoming_ptr_;
            last_ptr_   = incoming_ptr_;
            incoming_ptr_ = nullptr;
            // KF_O, F_L, -

            break;
        }
        case FIRST_TIME_WITHOUT_PACK :
        {
            // (4)  WARNING No pack on first incoming!
            // makeKF; KF.addCapture(incoming)
            // makeF; F.addCapture(incoming)
            // o <- i
            // l <- i

            FrameBasePtr kfrm = getProblem()->emplaceFrame(KEY_FRAME, incoming_ptr_->getTimeStamp());
            kfrm->addCapture(incoming_ptr_);
            FrameBasePtr frm = getProblem()->emplaceFrame(NON_KEY_FRAME, incoming_ptr_->getTimeStamp());
            frm->addCapture(incoming_ptr_);

            // TODO process info

            // Issue KF callback with new KF
            getProblem()->keyFrameCallback(kfrm, shared_from_this(), time_tolerance_);


            origin_ptr_ = incoming_ptr_;
            last_ptr_   = incoming_ptr_;
            incoming_ptr_ = nullptr;

            break;
        }
        case SECOND_TIME_WITHOUT_PACK :
        {
            // (2)
            // origin.F.unlink(last)
            // makeF; F.addCapture(incoming)
            // o <- l
            // l <- i

            origin_ptr_->getFramePtr()->unlinkCapture(last_ptr_);

            FrameBasePtr frm = getProblem()->emplaceFrame(NON_KEY_FRAME, incoming_ptr_->getTimeStamp());
            frm->addCapture(incoming_ptr_);

            // TODO process info

            origin_ptr_ = last_ptr_;
            last_ptr_   = incoming_ptr_;
            incoming_ptr_ = nullptr;

            break;
        }
        case RUNNING_WITH_PACK :
        {
            // (1)
            // pack.KF.addCapture(last)
            // makeF; F.addCapture(incoming)
            // o <- l
            // l <- i

            KFPackPtr pack = selectPack( last_ptr_ );
            kf_pack_buffer_.removeUpTo( last_ptr_->getTimeStamp() );

            WOLF_DEBUG( "PT: KF" , pack->key_frame->id() , " callback received at ts= " , pack->key_frame->getTimeStamp().get() );

            // Capture last_ is added to the new keyframe
            FrameBasePtr last_old_frame = last_ptr_->getFramePtr();
            last_old_frame->unlinkCapture(last_ptr_);
            last_old_frame->remove();
            pack->key_frame->addCapture(last_ptr_);
            WOLF_DEBUG( " --> appended last capture" );

            // Create new frame
            FrameBasePtr frm = getProblem()->emplaceFrame(NON_KEY_FRAME, incoming_ptr_->getTimeStamp());
            frm->addCapture(incoming_ptr_);

            // Detect new Features, initialize Landmarks, create Constraints, ...
            processNew(max_new_features_);

            // Establish constraints between last and origin
            // TODO: revise this fcn: code inside, and placement in here
            establishConstraints();

            // reset pointers
            origin_ptr_ = last_ptr_;
            last_ptr_   = incoming_ptr_;
            incoming_ptr_ = nullptr;

            break;
        }
        case RUNNING_WITHOUT_PACK :
        {
            // (3)
            // o <- o   // verbose
            // l <- i

            // TODO process info

            last_ptr_   = incoming_ptr_;
            incoming_ptr_ = nullptr;

            break;
        }
        default :
            break;
    }

    switch (processing_step_)
    {
        case FIRST_TIME_WITHOUT_PACK :
        case FIRST_TIME_WITH_PACK :
        {
            WOLF_DEBUG( "FIRST TIME" );

            // advance
            advance();

            // advance this
            last_ptr_ = incoming_ptr_;
            incoming_ptr_ = nullptr;

            // keyframe creation on last
            FrameBasePtr closest_key_frm = getProblem()->getTrajectoryPtr()->closestKeyFrameToTimeStamp(last_ptr_->getTimeStamp());
            if (closest_key_frm && checkTimeTolerance(closest_key_frm, last_ptr_))
            {
                // Set KF
                closest_key_frm->addCapture(last_ptr_);
                closest_key_frm->setKey();
                WOLF_DEBUG( "Last appended to existing F",  closest_key_frm->id(), " ==> set KF" , closest_key_frm->id() );
            }
            else
            {
                // Make KF
                FrameBasePtr new_frame_ptr = getProblem()->emplaceFrame(KEY_FRAME,
                                                                        getProblem()->getState(last_ptr_->getTimeStamp()),
                                                                        last_ptr_->getTimeStamp());
                new_frame_ptr->addCapture(last_ptr_); // Add incoming Capture to the new Frame
                WOLF_DEBUG( "Last appended to new KF" , new_frame_ptr->id() );

                getProblem()->keyFrameCallback(new_frame_ptr, shared_from_this(), time_tolerance_);
            }

            // Detect new Features, initialize Landmarks, create Constraints, ...
            processNew(max_new_features_);

            // Establish constraints from last
            establishConstraints();

            break;
        }


        case SECOND_TIME_WITHOUT_PACK :
        {
            WOLF_DEBUG("SECOND TIME or after KEY FRAME CALLBACK");

            // First we track the known Features between last and incoming
            processKnown();

            // Create a new non-key Frame in the Trajectory with the incoming Capture
            FrameBasePtr closest_key_frm = getProblem()->getTrajectoryPtr()->closestKeyFrameToTimeStamp(incoming_ptr_->getTimeStamp());
            if (closest_key_frm && checkTimeTolerance(closest_key_frm, incoming_ptr_))
            {
                // Just append the Capture to the existing keyframe
                closest_key_frm->addCapture(incoming_ptr_);
                WOLF_DEBUG("Incoming appended to F" , closest_key_frm->id() );
            }
            else
            {
                // Create a frame to hold what will become the last Capture
                FrameBasePtr new_frame_ptr = getProblem()->emplaceFrame(NON_KEY_FRAME, incoming_ptr_->getTimeStamp());
                new_frame_ptr->addCapture(incoming_ptr_); // Add incoming Capture to the new Frame
                WOLF_DEBUG("Incoming in new F" , new_frame_ptr->id() );
            }

            // Reset the derived Tracker
            reset();

            // Reset this
            origin_ptr_     = last_ptr_;
            last_ptr_       = incoming_ptr_;
            incoming_ptr_   = nullptr; // This line is not really needed, but it makes things clearer.

            // Establish constraints from last
            establishConstraints();

            break;
        }

        case RUNNING_WITH_PACK :
        case RUNNING_WITHOUT_PACK :
        default:
        {
            WOLF_DEBUG("OTHER TIMES");

            // 1. First we track the known Features and create new constraints as needed

            processKnown();

            // 2. Then we see if we want and we are allowed to create a KeyFrame
            // Three conditions to make a KF:
            //   - We vote for KF
            //   - Problem allows us to make keyframe
            //   - There is no existing KF very close to our Time Stamp <--- NOT SURE OF THIS

            FrameBasePtr closest_key_frm_to_last = last_ptr_->getFramePtr(); // start with the same last's frame
            if ( ! ( closest_key_frm_to_last && closest_key_frm_to_last->isKey() ) ) // last F is not KF
                closest_key_frm_to_last = getProblem()->getTrajectoryPtr()->closestKeyFrameToTimeStamp(last_ptr_->getTimeStamp());

            if (closest_key_frm_to_last && !checkTimeTolerance(closest_key_frm_to_last, last_ptr_)) // closest KF is not close enough
                closest_key_frm_to_last = nullptr;

            if ( !( (voteForKeyFrame() && permittedKeyFrame() ) || closest_key_frm_to_last ) )
            {

                // 2.a. We did not create a KeyFrame:

                // advance the derived tracker
                advance();

                // Advance this
                last_ptr_->getFramePtr()->addCapture(incoming_ptr_); // Add incoming Capture to the tracker's last Frame
                last_ptr_->remove();
                incoming_ptr_->getFramePtr()->setTimeStamp(incoming_ptr_->getTimeStamp());
                last_ptr_ = incoming_ptr_; // Incoming Capture takes the place of last Capture
                incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.

                WOLF_DEBUG("last <-- incoming");

            }
            else
            {

                // 2.b. We create a KF

                // Detect new Features, initialize Landmarks, create Constraints, ...
                processNew(max_new_features_);

                FrameBasePtr key_frm = getProblem()->getTrajectoryPtr()->closestKeyFrameToTimeStamp(incoming_ptr_->getTimeStamp());
                if ( abs(key_frm->getTimeStamp() - incoming_ptr_->getTimeStamp() ) < time_tolerance_)
                {
                    // Append incoming to existing key-frame
                    key_frm->addCapture(incoming_ptr_);
                    WOLF_DEBUG("Incoming adhered to existing KF" , key_frm->id());
                }
                else
                {
                    // Create a new non-key Frame in the Trajectory with the incoming Capture
                    // Make a non-key-frame to hold incoming
                    FrameBasePtr new_frame_ptr = getProblem()->emplaceFrame(NON_KEY_FRAME, incoming_ptr_->getTimeStamp());
                    new_frame_ptr->addCapture(incoming_ptr_); // Add incoming Capture to the new Frame
                    WOLF_DEBUG( "Incoming adhered to new F" , key_frm->id() );

                    // Make the last Capture's Frame a KeyFrame
                    setKeyFrame(last_ptr_);
                    WOLF_DEBUG( "Set KEY to last F" , key_frm->id() );
                }

                // Reset the derived Tracker
                reset();

                // Reset this
                origin_ptr_ = last_ptr_;
                last_ptr_ = incoming_ptr_;
                incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.

                // Establish constraints between last and origin
                establishConstraints();

            }

            break;
        }
    }

    postProcess();

    //std::cout << "-----End of process():" << std::endl;
}

//bool ProcessorTracker::keyFrameCallback(FrameBasePtr _keyframe_ptr, const Scalar& _time_tol_other)
//{
//    WOLF_TRACE("This callback doing nothing!!! ");
//    WOLF_DEBUG( "PT: KF" , _keyframe_ptr->id() , " callback received at ts= " , _keyframe_ptr->getTimeStamp().get() );
//
//    assert((last_ptr_ == nullptr || last_ptr_->getFramePtr() != nullptr) && "ProcessorTracker::keyFrameCallback: last_ptr_ must have a frame always");
//
//    Scalar time_tol = std::min(time_tolerance_, _time_tol_other);
//
//
//    // Nothing to do if:
//    //   - there is no last
//    //   - last frame is already a key frame
//    //   - last frame is too far in time from keyframe
//    if (last_ptr_ == nullptr || last_ptr_->getFramePtr()->isKey() || std::abs(last_ptr_->getTimeStamp() - _keyframe_ptr->getTimeStamp()) > time_tol)
//    {
//        WOLF_DEBUG( " --> nothing done" );
//        return false;
//    }
//
//    WOLF_DEBUG( " --> appended last capture" );
//    //std::cout << "ProcessorTracker::keyFrameCallback in sensor " << getSensorPtr()->id() << std::endl;
//
//    // Capture last_ is added to the new keyframe
//    FrameBasePtr last_old_frame = last_ptr_->getFramePtr();
//    last_old_frame->unlinkCapture(last_ptr_);
//    last_old_frame->remove();
//    _keyframe_ptr->addCapture(last_ptr_);
//
//    // Detect new Features, initialize Landmarks, create Constraints, ...
//    processNew(max_new_features_);
//
//    // Establish constraints between last and origin
//    establishConstraints();
//
//    // Set ready to go to 2nd case in process()
//    origin_ptr_ = nullptr;

    return true;

}

void ProcessorTracker::setKeyFrame(CaptureBasePtr _capture_ptr)
{

    assert(_capture_ptr != nullptr && _capture_ptr->getFramePtr() != nullptr && "ProcessorTracker::setKeyFrame: null capture or capture without frame");
    if (!_capture_ptr->getFramePtr()->isKey())
    {
        // Set key
        _capture_ptr->getFramePtr()->setKey();
        // Set state to the keyframe
        _capture_ptr->getFramePtr()->setState(getProblem()->getState(_capture_ptr->getTimeStamp()));
        // Call the new keyframe callback in order to let the other processors to establish their constraints
        getProblem()->keyFrameCallback(_capture_ptr->getFramePtr(), std::static_pointer_cast<ProcessorBase>(shared_from_this()), time_tolerance_);
    }
}

KFPackPtr ProcessorTracker::selectPack(const CaptureBasePtr & _cap)
{
    if (_cap)
        return kf_pack_buffer_.selectPack(_cap->getTimeStamp(), time_tolerance_);

    return nullptr;
}

void ProcessorTracker::computeProcessingStep()
{

    enum {FIRST_TIME, SECOND_TIME, RUNNING} step;
    if (origin_ptr_ == nullptr && last_ptr_ == nullptr)
        step = FIRST_TIME;
    else if (origin_ptr_ == last_ptr_)
        step = SECOND_TIME;
    else
        step = RUNNING;


    switch (step)
    {
        case FIRST_TIME :

            if (selectPack(incoming_ptr_))
                processing_step_ = FIRST_TIME_WITH_PACK;
            else // ! last && ! pack(incoming)
            {
                WOLF_WARN("\n||*||");
                WOLF_INFO("\n ... It seems you missed something!");
                WOLF_INFO("\nReceived first Capture without KF pack to associate to");
                WOLF_INFO("Creating a KF for the Capture. But...")
                WOLF_INFO("Check the following:");
                WOLF_INFO("  - You have all processors installed before starting receiving any data");
                WOLF_INFO("  - You issued a problem->setPrior() after all processors are installed");
                WOLF_INFO("  - You have configured all your processors with compatible time tolerances");
                processing_step_ = FIRST_TIME_WITHOUT_PACK;
            }
        break;

        case SECOND_TIME :

            if (selectPack(last_ptr_))
            {
                WOLF_WARN("\n||*||");
                WOLF_INFO("\n ... It seems you missed something!");
                WOLF_INFO("\nPack's KF and origin's KF have matching time stamps (i.e. below time tolerances)");
                WOLF_INFO("Check the following:");
                WOLF_INFO("  - You have all processors installed before starting receiving any data");
                WOLF_INFO("  - You issued a problem->setPrior() after all processors are installed");
                WOLF_INFO("  - You have configured all your processors with compatible time tolerances");
                WOLF_ERROR("Pack's KF and origin's KF have matching time stamps (i.e. below time tolerances). Check time tolerances!");
            }
            else
                processing_step_ = SECOND_TIME_WITHOUT_PACK;
            break;

        case RUNNING :
        default :

            if (selectPack(last_ptr_))
            {
                if (last_ptr_->getFramePtr()->isKey())
                {
                    WOLF_WARN("\n||*||");
                    WOLF_INFO("\n ... It seems you missed something!");
                    WOLF_INFO("\nPack's KF and last's KF have matching time stamps (i.e. below time tolerances)");
                    WOLF_INFO("Check the following:");
                    WOLF_INFO("  - You have all processors installed before starting receiving any data");
                    WOLF_INFO("  - You issued a problem->setPrior() after all processors are installed");
                    WOLF_INFO("  - You have configured all your processors with compatible time tolerances");
                    WOLF_ERROR("Pack's KF and last's KF have matching time stamps (i.e. below time tolerances). Check time tolerances!");
                }
                processing_step_ = RUNNING_WITH_PACK;
            }
            else
                processing_step_ = RUNNING_WITHOUT_PACK;
            break;
    }
}




} // namespace wolf

