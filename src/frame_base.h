
#ifndef FRAME_BASE_H_
#define FRAME_BASE_H_

class TrajectoryBase;
class CaptureBase;

//std includes
#include <iostream>
#include <vector>
#include <list>
#include <random>
#include <cmath>

//Wolf includes
#include "wolf.h"
#include "time_stamp.h"
#include "node_linked.h"
#include "trajectory_base.h"
#include "capture_base.h"
#include "state_base.h"

//class FrameBase
class FrameBase : public NodeLinked<TrajectoryBase,CaptureBase>
{
    protected:
        FrameType type_; //type of frame. Either REGULAR_FRAME or KEY_FRAME. (types defined at wolf.h)
        TimeStamp time_stamp_; //frame time stamp
        //Eigen::Vector3s state_; //TBD: Instead , It could be a vector/list/map of pointers to state units
		StateBaseShPtr p_ptr_; // Position state unit pointer
		StateBaseShPtr o_ptr_; // Orientation state unit pointer
		StateBaseShPtr v_ptr_; // Velocity state unit pointer
		StateBaseShPtr w_ptr_; // Angular velocity state unit pointer
		//TBD: accelerations?
        
    public:
        /** \brief Constructor with only time stamp
         *
         * Constructor with only time stamp
         * \param _traj_ptr a pointer to the Capture up node
         * \param _tp indicates frame type. Generally either REGULAR_FRAME or KEY_FRAME. (types defined at wolf.h)
         * \param _p_ptr StateBase pointer to the position (default: nullptr)
         * \param _o_ptr StateBase pointer to the orientation (default: nullptr)
         * \param _v_ptr StateBase pointer to the velocity (default: nullptr)
         * \param _w_ptr StateBase pointer to the angular velocity (default: nullptr)
         *
         **/
        FrameBase(const TrajectoryBasePtr & _traj_ptr, const TimeStamp& _ts, const StateBaseShPtr& _p_ptr = {}, const StateBaseShPtr& _o_ptr = {}, const StateBaseShPtr& _v_ptr = {}, const StateBaseShPtr& _w_ptr = {});

        /** \brief Constructor with type, time stamp and state pointer
         * 
         * Constructor with type, time stamp and state pointer
         * \param _traj_ptr a pointer to the Capture up node
         * \param _tp indicates frame type. Generally either REGULAR_FRAME or KEY_FRAME. (types defined at wolf.h)
         * \param _ts is the time stamp associated to this frame, provided in seconds
         * \param _p_ptr StateBase pointer to the position (default: nullptr)
         * \param _o_ptr StateBase pointer to the orientation (default: nullptr)
         * \param _v_ptr StateBase pointer to the velocity (default: nullptr)
         * \param _w_ptr StateBase pointer to the angular velocity (default: nullptr)
         * 
         **/        
        FrameBase(const TrajectoryBasePtr & _traj_ptr, const FrameType & _tp, const TimeStamp& _ts, const StateBaseShPtr& _p_ptr = {}, const StateBaseShPtr& _o_ptr = {}, const StateBaseShPtr& _v_ptr = {}, const StateBaseShPtr& _w_ptr = {});
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~FrameBase();
        
        /** \brief Checks if this frame is KEY_FRAME 
         * 
         * Returns true if type_ is KEY_FRAME. Oterwise returns false.
         * 
         **/
        bool isKey() const;
        
        void setType(FrameType _ft);
        
        void setTimeStamp(const TimeStamp& _ts);
        
        TimeStamp getTimeStamp() const;
        
        void getTimeStamp(TimeStamp & _ts) const;

        void setState(const Eigen::VectorXs& _st);

        void addCapture(CaptureBaseShPtr& _capt_ptr);
        
        const TrajectoryBasePtr getTrajectoryPtr() const;

        //const CaptureBaseList & captureList() const;
        
        CaptureBaseList* getCaptureListPtr();
        
        FrameBasePtr getPreviousFrame() const;

        StateBaseShPtr getPPtr() const;

        StateBaseShPtr getOPtr() const;

        StateBaseShPtr getVPtr() const;

        StateBaseShPtr getWPtr() const;

        virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;
        
};
#endif
