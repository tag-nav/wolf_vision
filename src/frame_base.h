
#ifndef FRAME_BASE_H_
#define FRAME_BASE_H_

// Fwd refs
class TrajectoryBase;
class CaptureBase;
class StateBlock;

//Wolf includes
#include "wolf.h"
#include "time_stamp.h"
#include "node_linked.h"

//std includes

//class FrameBase
class FrameBase : public NodeLinked<TrajectoryBase,CaptureBase>
{
    protected:
        FrameType type_;         ///< type of frame. Either NON_KEY_FRAME or KEY_FRAME. (types defined at wolf.h)
        TimeStamp time_stamp_;   ///< frame time stamp
        StateStatus status_;     ///< status of the estimation of the frame state
        StateBlock* p_ptr_;      ///< Position state block pointer
        StateBlock* o_ptr_;      ///< Orientation state block pointer
        StateBlock* v_ptr_;      ///< Linear velocity state block pointer
        std::list<ConstraintBase*> constraint_to_list_; ///> List of constraints TO this frame
        
    public:
        /** \brief Constructor of non-key Frame with only time stamp
         *
         * Constructor with only time stamp
         * \param _ts is the time stamp associated to this frame, provided in seconds
         * \param _p_ptr StateBlock pointer to the position (default: nullptr)
         * \param _o_ptr StateBlock pointer to the orientation (default: nullptr)
         *
         **/
        FrameBase(const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _o_ptr = nullptr, StateBlock* _v_ptr = nullptr);
        
        /** \brief Constructor with type, time stamp and state pointer
         * 
         * Constructor with type, time stamp and state pointer
         * \param _tp indicates frame type. Generally either NON_KEY_FRAME or KEY_FRAME. (types defined at wolf.h)
         * \param _ts is the time stamp associated to this frame, provided in seconds
         * \param _p_ptr StateBlock pointer to the position (default: nullptr)
         * \param _o_ptr StateBlock pointer to the orientation (default: nullptr)
         * 
         **/        
        FrameBase(const FrameType & _tp, const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _o_ptr = nullptr, StateBlock* _v_ptr = nullptr);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         * 
         **/
        virtual ~FrameBase();



        // Frame properties -----------------------------------------------

        // KeyFrame / NonKeyFrame
        bool isKey() const;
        void setKey();

        // Fixed / Estimated
        void fix();
        void unfix();
        bool isFixed();



        // Frame values ------------------------------------------------

        void setTimeStamp(const TimeStamp& _ts);
        TimeStamp getTimeStamp() const;
        void getTimeStamp(TimeStamp& _ts) const;

        StateBlock* getPPtr() const;
        StateBlock* getOPtr() const;
        StateBlock* getVPtr() const;

        void setState(const Eigen::VectorXs& _st);
        Eigen::VectorXs getState() const;



        // Wolf tree access ---------------------------------------------------

        TrajectoryBase* getTrajectoryPtr() const;

        FrameBase* getPreviousFrame() const;
        FrameBase* getNextFrame() const;

        CaptureBaseList* getCaptureListPtr();
        void addCapture(CaptureBase* _capt_ptr);
        void removeCapture(CaptureBaseIter& _capt_iter);
        CaptureBaseIter hasCaptureOf(const SensorBase* _sensor_ptr);

        void getConstraintList(ConstraintBaseList & _ctr_list);




        // Other constraints pointing to this frame ------------------------------------------

        /** \brief Gets the list of constraints linked with this frame
         **/
        std::list<ConstraintBase*>* getConstraintToListPtr();

        /** \brief Link with a constraint
         **/
        void addConstraintTo(ConstraintBase* _ctr_ptr);

        /** \brief Remove a constraint to this frame
         **/
        void removeConstraintTo(ConstraintBase* _ctr_ptr);

        /** \brief Gets the number of constraints linked with this frame
         **/
        unsigned int getHits() const;



    private:
        /** \brief Gets the Frame status (see wolf.h for Frame status)
         **/
        StateStatus getStatus() const;
        /** \brief Sets the Frame status (see wolf.h for Frame status)
         **/
        void setStatus(StateStatus _st);


};

// IMPLEMENTATION //

inline bool FrameBase::isKey() const
{
    if (type_ == KEY_FRAME)
        return true;
    else
        return false;
}

inline void FrameBase::fix()
{
    this->setStatus(ST_FIXED);
}

inline void FrameBase::unfix()
{
    //std::cout << "Unfixing frame " << nodeId() << std::endl;
    this->setStatus(ST_ESTIMATED);
}

inline bool FrameBase::isFixed()
{
    return status_ == ST_FIXED;
}

inline void FrameBase::setTimeStamp(const TimeStamp& _ts)
{
    time_stamp_ = _ts;
}

inline void FrameBase::getTimeStamp(TimeStamp& _ts) const
{
    _ts = time_stamp_.get();
}

inline TimeStamp FrameBase::getTimeStamp() const
{
    return time_stamp_.get();
}

inline StateBlock* FrameBase::getPPtr() const
{
    return p_ptr_;
}

inline StateBlock* FrameBase::getOPtr() const
{
    return o_ptr_;
}

inline StateBlock* FrameBase::getVPtr() const
{
    return v_ptr_;
}

inline TrajectoryBase* FrameBase::getTrajectoryPtr() const
{
    return upperNodePtr();
}

inline CaptureBaseList* FrameBase::getCaptureListPtr()
{
    return getDownNodeListPtr();
}

inline void FrameBase::addCapture(CaptureBase* _capt_ptr)
{
    addDownNode(_capt_ptr);
}

inline void FrameBase::removeCapture(CaptureBaseIter& _capt_iter)
{
    //std::cout << "removing capture " << (*_capt_iter)->nodeId() << " from Frame " << nodeId() << std::endl;
    removeDownNode(_capt_iter);
}

inline std::list<ConstraintBase*>* FrameBase::getConstraintToListPtr()
{
    return &constraint_to_list_;
}

inline void FrameBase::addConstraintTo(ConstraintBase* _ctr_ptr)
{
    constraint_to_list_.push_back(_ctr_ptr);
}

inline unsigned int FrameBase::getHits() const
{
    return constraint_to_list_.size();
}

inline StateStatus FrameBase::getStatus() const
{
    return status_;
}

#endif
