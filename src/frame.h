/**
 * \file frame.h
 *
 *  Created on: 12/08/2014
 *     \author: jsola
 */

#ifndef FRAME_H_
#define FRAME_H_

#include "node_constrainer.h"
#include "time_stamp.h"

// Forward declarations for node templates
//class VehicleBase;
//class Capture;
// Forward declarations for member pointers
class StatePose;

class Frame : public NodeConstrainer<VehicleBase, Capture>
{

    private:
        TimeStamp time_stamp_; ///< Time stamp
        FrameType frame_type_; ///< indicates whether the frame is KEY_FRAME or REGULAR_FRAME
        StateShPtr state_ptr_; ///< Pointer to state. //TODO see if we can put the object and template the class as initially.

    public:
        /** \brief Constructor
         * 
         * \param _vehicle_ptr a shared pointer to the Vehicle up node
         * \param _state_ptr a shared pointer to the state of the vehicle at the time of data capture
         * \param _ts The time stamp
         * \param _ft the frame type (KEY_FRAME or REGULAR_FRAME, it defaults to KEY_FRAME).
         * \param _loc the location in the Wolf tree (TOP, MID or BOTTOM, it defaults to MID)
         * 
         **/
        Frame(const VehicleShPtr& _vehicle_ptr, const StateShPtr & _state_ptr, const TimeStamp& _ts, FrameType _ft = KEY_FRAME, const NodeLocation _loc = MID);

        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~Frame();

        void addCapture(const CaptureShPtr& _sc_ptr);

        const CaptureList& captureList() const;

        const StateShPtr& stateShPtr() const;
        const StatePtr statePtr() const;
        const StatePose& state() const;

        virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;

};

//////////////////////////////////////
// IMPLEMENTATION
//////////////////////////////////////

#include "state_pose.h"  //TODO: Why this include is here and not on the top of the file ??


Frame::Frame(const VehicleShPtr& _vehicle_ptr, const StateShPtr & _state, const TimeStamp& _ts, FrameType _ft, const NodeLocation _loc) :
        NodeConstrainer(_loc, "FRAME", _vehicle_ptr), //
        time_stamp_(_ts), //
        frame_type_(_ft), //
        state_ptr_(_state)
{
    //
}

Frame::~Frame()
{
    //
}

inline void Frame::addCapture(const CaptureShPtr& _sc_ptr)
{
    addDownNode(_sc_ptr);
}

inline const CaptureList& Frame::captureList() const
{
    return downNodeList();
}

inline const StateShPtr& Frame::stateShPtr() const
{
    return state_ptr_;
}

inline const StatePtr Frame::statePtr() const
{
    return state_ptr_.get();
}

inline const StatePose& Frame::state() const
{
    return *state_ptr_;
}

void Frame::printSelf(unsigned int _ntabs, std::ostream& _ost) const
{
    NodeConstrainer::printSelf(_ntabs, _ost);
    printNTabs(_ntabs);
    _ost << "\tPose : ( " << state_ptr_->x().transpose() << " )" << std::endl;
}

#endif /* FRAME_H_ */
