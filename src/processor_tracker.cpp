/*
 * ProcessorTracker.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: jvallve
 */

#include "processor_tracker.h"

namespace wolf
{

ProcessorTracker::ProcessorTracker(ProcessorType _tp, const unsigned int _max_new_features) :
        ProcessorBase(_tp), origin_ptr_(nullptr), last_ptr_(nullptr), incoming_ptr_(nullptr),
        max_new_features_(_max_new_features)
{
    //
}

ProcessorTracker::~ProcessorTracker()
{
    // FIXME: This test with nullptr is not fail safe. Only the class design can make it safe, by ensuring
    // at all times that whenever incoming_ptr_ is not used, it points to nullptr.
    // See both flavors of reset(), and advance().
    if (incoming_ptr_ != nullptr)
        delete incoming_ptr_;
}

void ProcessorTracker::process(CaptureBase* const _incoming_ptr)
{
    std::cout << "\nProcess..." << std::endl;

    incoming_ptr_ = _incoming_ptr;

    preProcess();

    // FIRST TIME
    if (origin_ptr_ == nullptr)
    {
        std::cout << "FIRST TIME" << std::endl;
        std::cout << "Features in origin: " << 0 << "; in last: " << 0 << std::endl;

        last_ptr_ = _incoming_ptr;
        origin_ptr_ = _incoming_ptr;

        if (last_ptr_->getFramePtr() == nullptr)
            makeFrame(last_ptr_);

        // Detect new Features, initialize Landmarks, create Constraints, ...
        processNew(max_new_features_);

        // Make the last Capture's Frame a KeyFrame so that it gets into the solver
        last_ptr_->getFramePtr()->setKey();

        // Establish constraints from last
        establishConstraints();

        // advance the derived tracker
        reset();

        // Clear incoming ptr. origin and last are OK.
        incoming_ptr_ = nullptr;

        std::cout << "Features in origin (should be 5): " << origin_ptr_->getFeatureListPtr()->size() << "; in last (should be 5): " << last_ptr_->getFeatureListPtr()->size() << std::endl;
    }
    // SECOND TIME
    else if (origin_ptr_ == last_ptr_)
    {
        std::cout << "SECOND TIME" << std::endl;
        std::cout << "Features in origin: " << origin_ptr_->getFeatureListPtr()->size() << "; in last: " << last_ptr_->getFeatureListPtr()->size() << std::endl;


        // 1. First we track the known Features and create new constraints as needed
        processKnown();

        // Make frame in incoming
        makeFrame(incoming_ptr_);

        // reset the derived tracker
        reset();

        // Update the tracker's last and incoming pointers one step ahead
        last_ptr_ = incoming_ptr_; // Incoming Capture takes the place of last Capture
        incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.

        std::cout << "Features in origin: " << origin_ptr_->getFeatureListPtr()->size() << "; in last: " << last_ptr_->getFeatureListPtr()->size() << std::endl;
    }
    // OTHER TIMES
    else
    {
        std::cout << "OTHER TIMES" << std::endl;

        std::cout << "Features in origin: " << origin_ptr_->getFeatureListPtr()->size() << "; in last: " << last_ptr_->getFeatureListPtr()->size() << std::endl;

        // 1. First we track the known Features and create new constraints as needed

        processKnown();

        // 2. Then we see if we want and we are allowed to create a KeyFrame
        if (!(voteForKeyFrame() && permittedKeyFrame()))
        {
            // We did not create a KeyFrame:

            // advance the derived tracker
            advance();

            // Advance this
            last_ptr_->getFramePtr()->addCapture(incoming_ptr_); // Add incoming Capture to the tracker's Frame
            last_ptr_->destruct(); // TODO: JS->JV why this does not work?? Destruct now the obsolete last before reassigning a new pointer
            incoming_ptr_->getFramePtr()->setTimeStamp(incoming_ptr_->getTimeStamp());
            last_ptr_ = incoming_ptr_; // Incoming Capture takes the place of last Capture
            incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.
        }
        else
        {
            // 2.b. Detect new Features, initialize Landmarks, create Constraints, ...
            processNew();

            // Create a new non-key Frame in the Trajectory with the incoming Capture
            makeFrame(incoming_ptr_);

            // Make the last Capture's Frame a KeyFrame so that it gets into the solver
            last_ptr_->getFramePtr()->setKey();

            // Establish constraints between last and origin
            establishConstraints();

            // Reset the derived Tracker
            reset();
            // Reset this
            origin_ptr_ = last_ptr_;
            last_ptr_ = incoming_ptr_;
            incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.
        }

        std::cout << "Features in origin: " << origin_ptr_->getFeatureListPtr()->size() << "; in last: " << last_ptr_->getFeatureListPtr()->size() << std::endl;

    }

    postProcess();
}

} // namespace wolf

