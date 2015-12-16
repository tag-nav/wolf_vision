#ifndef CAPTURE_BASE_H_
#define CAPTURE_BASE_H_

// Forward declarations for node templates
class FrameBase;
class FeatureBase;

//std includes
//

//Wolf includes
#include "wolf.h"
#include "time_stamp.h"
#include "node_linked.h"
#include "frame_base.h"
#include "feature_base.h"
#include "sensor_base.h"

//class CaptureBase
class CaptureBase : public NodeLinked<FrameBase, FeatureBase>
{
    protected:
        TimeStamp time_stamp_; ///< Time stamp
        SensorBase* sensor_ptr_; ///< Pointer to sensor
        Eigen::VectorXs data_; ///< //TODO: to be removed. raw data. Ignore this member if you need other data structures.
        Eigen::MatrixXs data_covariance_; /////TODO: to be removed < Noise of the capture. // TODO Covariance should belong to Feature measurement, to Sensor, or to Processor.

        // Allow precomputing global frames for accelerating code.
        //Eigen::Vector3s sensor_pose_global_; ///< Sensor pose in world frame: composition of the frame pose and the sensor pose. TODO: use state units
        //Eigen::Vector3s inverse_sensor_pose_; ///< World pose in the sensor frame: inverse of the global_pose_. TODO: use state units

        // Deal with sensors with dynamic extrinsics (check dynamic_extrinsic_ in SensorBase)
        StateBlock* sensor_p_ptr_; //TODO: initialize this at construction time; delete it at destruction time
        StateBlock* sensor_o_ptr_; //TODO: initialize this at construction time; delete it at destruction time

    public:
        CaptureBase(const TimeStamp& _ts, SensorBase* _sensor_ptr);

		//TODO: to be removed
        CaptureBase(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data);

		//TODO: to be removed
        CaptureBase(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance);


        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~CaptureBase();

        /** \brief Set link to Frame
         *
         * Set link to Frame
         *
         **/
		//TODO: to be removed
        void linkToFrame(FrameBase* _frm_ptr);

        /** \brief Adds a Feature to the down node list
         *
         * Adds a Feature to the down node list
         *
         **/
        void addFeature(FeatureBase* _ft_ptr);

        /** \brief Gets up_node_ptr_
         *
         * Gets up_node_ptr_, which is a pointer to the Frame owning of this Capture
         *
         **/
        FrameBase* getFramePtr() const;

        /** \brief Gets a pointer to feature list
         *
         * Gets a pointer to feature list
         *
         **/
        FeatureBaseList* getFeatureListPtr();

        /** \brief Fills the provided list with all constraints related to this capture
         *
         * Fills the provided list with all constraints related to this capture
         *
         **/
		//TODO: Check if it could be removed. THen remove it also at every wolf tree level. 
        void getConstraintList(ConstraintBaseList & _ctr_list);

        TimeStamp getTimeStamp() const;

        SensorBase* getSensorPtr() const;

        StateBlock* getSensorPPtr() const;

        StateBlock* getSensorOPtr() const;

        void setTimeStamp(const TimeStamp & _ts);

        void setTimeStampToNow();

		//TODO: to be removed
        Eigen::VectorXs getData();

		//TODO: to be removed
        Eigen::MatrixXs getDataCovariance();

		//TODO: to be removed
        void setData(unsigned int _size, const WolfScalar *_data);

		//TODO: to be removed
        void setData(const Eigen::VectorXs& _data);

		//TODO: to be removed
        void setDataCovariance(const Eigen::MatrixXs& _data_cov);

		// TODO rename to process()
        virtual void processCapture(); 

		// TODO Rename to computeFrameInitialGuess() ... for instance
		//      Another name could be provideFrameInitialGuess(); 
		//      Move it to ProcessorX class() 
		//      Should be virtual in ProcessorBase with an empty/error message
        virtual Eigen::VectorXs computePrior(const TimeStamp& _now) const = 0;

		
        virtual void printSelf(unsigned int _ntabs = 0, std::ostream & _ost = std::cout) const;
};
#endif
