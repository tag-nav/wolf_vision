#ifndef CAPTURE_BASE_H_
#define CAPTURE_BASE_H_

// Forward declarations for node templates
namespace wolf{
class FrameBase;
class FeatureBase;
class SensorBase;
}

//Wolf includes
#include "wolf.h"
#include "time_stamp.h"
#include "node_linked.h"

//std includes
//

namespace wolf{


//class CaptureBase
class CaptureBase : public NodeLinked<FrameBase, FeatureBase>
{
    private:
        static unsigned int capture_id_count_;
    protected:
        unsigned int capture_id_;
        TimeStamp time_stamp_; ///< Time stamp
        SensorBase* sensor_ptr_; ///< Pointer to sensor

        // Allow precomputing global frames for accelerating code.
        //Eigen::Vector3s sensor_pose_global_; ///< Sensor pose in world frame: composition of the frame pose and the sensor pose. TODO: use state units
        //Eigen::Vector3s inverse_sensor_pose_; ///< World pose in the sensor frame: inverse of the global_pose_. TODO: use state units

        // Deal with sensors with dynamic extrinsics (check dynamic_extrinsic_ in SensorBase)
        StateBlock* sensor_p_ptr_; //TODO: initialize this at construction time; delete it at destruction time
        StateBlock* sensor_o_ptr_; //TODO: initialize this at construction time; delete it at destruction time

    public:

        CaptureBase(const TimeStamp& _ts, SensorBase* _sensor_ptr);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~CaptureBase();

        unsigned int id();

        /** \brief Adds a Feature to the down node list
         **/
        FeatureBase* addFeature(FeatureBase* _ft_ptr);

        /** \brief Gets Frame pointer
         **/
        FrameBase* getFramePtr() const;

        /** \brief Gets a pointer to feature list
         **/
        FeatureBaseList* getFeatureListPtr();

        /** \brief Fills the provided list with all constraints related to this capture
         **/
        //TODO: Check if it could be removed. THen remove it also at every wolf tree level.
        void getConstraintList(ConstraintBaseList & _ctr_list);

        TimeStamp getTimeStamp() const;

        SensorBase* getSensorPtr() const;

        StateBlock* getSensorPPtr() const;

        StateBlock* getSensorOPtr() const;

        void setTimeStamp(const TimeStamp& _ts);

        void setTimeStampToNow();

        /** \brief Call all the processors for this Capture
         */
        virtual void process(); 

        // TODO Move it to ProcessorX class()
        //      Rename to computeFrameInitialGuess() ... for instance
        //      Another name could be provideFrameInitialGuess();
        //Should be virtual in ProcessorBase with an empty/error message
        virtual Eigen::VectorXs computeFramePose(const TimeStamp& _now) const {return Eigen::VectorXs::Zero(0);};

		
//        virtual void printSelf(unsigned int _ntabs = 0, std::ostream & _ost = std::cout) const;
};

inline unsigned int CaptureBase::id()
{
    return capture_id_;
}

inline FeatureBase* CaptureBase::addFeature(FeatureBase* _ft_ptr)
{
    //std::cout << "Adding feature" << std::endl;
    addDownNode(_ft_ptr);
    return _ft_ptr;
}

inline FrameBase* CaptureBase::getFramePtr() const
{
    return upperNodePtr();
}

inline FeatureBaseList* CaptureBase::getFeatureListPtr()
{
    return getDownNodeListPtr();
}

inline TimeStamp CaptureBase::getTimeStamp() const
{
    return time_stamp_;
}

inline SensorBase* CaptureBase::getSensorPtr() const
{
    return sensor_ptr_;
}

inline void CaptureBase::setTimeStamp(const TimeStamp& _ts)
{
    time_stamp_ = _ts;
}

inline void CaptureBase::setTimeStampToNow()
{
    time_stamp_.setToNow();
}

} // namespace wolf

#endif
