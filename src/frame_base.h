
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
        StateStatus status_; // status of the estimation of the frame state
		StateBase* p_ptr_; // Position state unit pointer
		StateOrientation* o_ptr_; // Orientation state unit pointer
		StateBase* v_ptr_; // Velocity state unit pointer
		StateBase* w_ptr_; // Angular velocity state unit pointer
		//TODO: accelerations?
        
    public:
        /** \brief Constructor with only time stamp
         *
         * Constructor with only time stamp
         * \param _traj_ptr a pointer to the Trajectory up node
         * \param _tp indicates frame type. Generally either REGULAR_FRAME or KEY_FRAME. (types defined at wolf.h)
         * \param _p_ptr StateBase pointer to the position (default: nullptr)
         * \param _o_ptr StateBase pointer to the orientation (default: nullptr)
         * \param _v_ptr StateBase pointer to the velocity (default: nullptr)
         * \param _w_ptr StateBase pointer to the angular velocity (default: nullptr)
         *
         **/
        FrameBase(const TimeStamp& _ts, StateBase* _p_ptr, StateOrientation* _o_ptr = nullptr, StateBase* _v_ptr = nullptr, StateBase* _w_ptr = nullptr); //ACM: Either remove all pointer arguments from this header, or merge both constructors in a single one
        
        /** \brief Constructor with type, time stamp and state pointer
         * 
         * Constructor with type, time stamp and state pointer
         * \param _traj_ptr a pointer to the Trajectory up node
         * \param _tp indicates frame type. Generally either REGULAR_FRAME or KEY_FRAME. (types defined at wolf.h)
         * \param _ts is the time stamp associated to this frame, provided in seconds
         * \param _p_ptr StateBase pointer to the position (default: nullptr)
         * \param _o_ptr StateBase pointer to the orientation (default: nullptr)
         * \param _v_ptr StateBase pointer to the velocity (default: nullptr)
         * \param _w_ptr StateBase pointer to the angular velocity (default: nullptr)
         * 
         **/        
        FrameBase(const FrameType & _tp, const TimeStamp& _ts, StateBase* _p_ptr, StateOrientation* _o_ptr = nullptr, StateBase* _v_ptr = nullptr, StateBase* _w_ptr = nullptr);
        
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
        
        void fix();

        void unfix();

        void setTimeStamp(const TimeStamp& _ts); //ACM: if all constructors require a timestamp, do we need a set? Should we allow to change TS?
        
        TimeStamp getTimeStamp() const;
        
        void getTimeStamp(TimeStamp & _ts) const;

        void setStatus(const StateStatus& _status);

        StateStatus getStatus() const;

        void setState(const Eigen::VectorXs& _st);

        void addCapture(CaptureBase* _capt_ptr);

        Eigen::Map<Eigen::VectorXs> getState() const;

        void removeCapture(CaptureBaseIter& _capt_ptr);
        
        TrajectoryBase* getTrajectoryPtr() const;
        
        CaptureBaseList* getCaptureListPtr();
        
        void getConstraintList(ConstraintBaseList & _ctr_list);

        FrameBase* getPreviousFrame() const;

        FrameBase* getNextFrame() const;

        StateBase* getPPtr() const;

        StateOrientation* getOPtr() const;

        StateBase* getVPtr() const;

        StateBase* getWPtr() const;

        const Eigen::Matrix4s * getTransformationMatrix() const; //ACM: Who owns this return matrix ?

        CaptureBaseIter hasCaptureOf(const SensorBase* _sensor_ptr);

        virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;
        
};
#endif
