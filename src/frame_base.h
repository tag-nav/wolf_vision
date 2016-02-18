
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
#include "state_block.h"

//class FrameBase
class FrameBase : public NodeLinked<TrajectoryBase,CaptureBase>
{
    protected:
        FrameType type_;         ///< type of frame. Either REGULAR_FRAME or KEY_FRAME. (types defined at wolf.h)
        TimeStamp time_stamp_;   ///< frame time stamp
        StateStatus status_;     ///< status of the estimation of the frame state
		StateBlock* p_ptr_;      ///< Position state block pointer
		StateBlock* o_ptr_;      ///< Orientation state block pointer
		std::list<ConstraintBase*> constraint_to_list_; ///> List of constraints TO this frame
        
    public:
        /** \brief Constructor with only time stamp
         *
         * Constructor with only time stamp
         * \param _ts is the time stamp associated to this frame, provided in seconds
         * \param _p_ptr StateBlock pointer to the position (default: nullptr)
         * \param _o_ptr StateBlock pointer to the orientation (default: nullptr)
         *
         **/
        FrameBase(const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _o_ptr = nullptr);
        
        /** \brief Constructor with type, time stamp and state pointer
         * 
         * Constructor with type, time stamp and state pointer
         * \param _tp indicates frame type. Generally either REGULAR_FRAME or KEY_FRAME. (types defined at wolf.h)
         * \param _ts is the time stamp associated to this frame, provided in seconds
         * \param _p_ptr StateBlock pointer to the position (default: nullptr)
         * \param _o_ptr StateBlock pointer to the orientation (default: nullptr)
         * 
         **/        
        FrameBase(const FrameType & _tp, const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _o_ptr = nullptr);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         * 
         **/
        virtual ~FrameBase();

        /** \brief Link with a constraint
         *
         * Link with a constraint
         *
         **/
        void addConstraintTo(ConstraintBase* _ctr_ptr);

        /** \brief Remove a constraint to this frame
         *
         * Remove a constraint to this frame
         *
         **/
        void removeConstraintTo(ConstraintBase* _ctr_ptr);

        /** \brief Gets the number of constraints linked with this frame
         *
         * Gets the number of constraints linked with this frame
         *
         **/
        unsigned int getHits() const;

        /** \brief Gets the list of constraints linked with this frame
         *
         * Gets the list of constraints linked with this frame
         *
         **/
        std::list<ConstraintBase*>* getConstraintToListPtr();

        /** \brief Sets the Frame status
         *
         * Sets the Frame status (see wolf.h)
         *
         **/
        void setStatus(StateStatus _st);

        /** \brief Sets all the states status to fixed
         *
         * Sets all the states status to fixed
         *
         **/
        void fix();

        /** \brief Sets all the states status to estimated
         *
         * Sets all the states status to estimated
         *
         **/
        void unfix();
        
        /** \brief Checks if this frame is KEY_FRAME 
         * 
         * Returns true if type_ is KEY_FRAME. Oterwise returns false.
         * 
         **/
        bool isKey() const;

        /** \brief Sets the Frame type
         *
         * Sets the frame type (see wolf.h)
         *
         **/
        
        void setType(FrameType _ft);

        void setTimeStamp(const TimeStamp& _ts); //ACM: if all constructors require a timestamp, do we need a set? Should we allow to change TS?
        
        TimeStamp getTimeStamp() const;
        
        void getTimeStamp(TimeStamp & _ts) const;

        StateStatus getStatus() const;

        void setState(const Eigen::VectorXs& _st);

        Eigen::VectorXs getState() const;

        void addCapture(CaptureBase* _capt_ptr);

        void removeCapture(CaptureBaseIter& _capt_ptr);
        
        TrajectoryBase* getTrajectoryPtr() const;
        
        CaptureBaseList* getCaptureListPtr();
        
        void getConstraintList(ConstraintBaseList & _ctr_list);

        FrameBase* getPreviousFrame() const;

        FrameBase* getNextFrame() const;

        StateBlock* getPPtr() const;

        StateBlock* getOPtr() const;

        const Eigen::Matrix4s * getTransformationMatrix() const; //ACM: Who owns this return matrix ?

        CaptureBaseIter hasCaptureOf(const SensorBase* _sensor_ptr);

//        virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;
        
};
#endif
