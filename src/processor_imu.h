#ifndef PROCESSOR_IMU_H
#define PROCESSOR_IMU_H

// Wolf
#include "processor_motion.h"
#include "capture_imu.h"
#include "feature_imu.h"


namespace wolf {
WOLF_STRUCT_PTR_TYPEDEFS(ProcessorIMUParams);

struct ProcessorIMUParams : public ProcessorParamsBase
{
        Scalar max_time_span;
        Size   max_buff_length;
        Scalar dist_traveled;
        Scalar angle_turned;
        bool voting_active; //IMU will not vote for key Frames to be created


        ProcessorIMUParams() :
            max_time_span(0.5),
            max_buff_length(10),
            dist_traveled(5),
            angle_turned(.5),
            voting_active(false)
        {
            type = "IMU";
            name = "";
        }
};

WOLF_PTR_TYPEDEFS(ProcessorIMU);
    
//class
class ProcessorIMU : public ProcessorMotion{
    public:
        ProcessorIMU(ProcessorIMUParamsPtr _params = nullptr);
        virtual ~ProcessorIMU();

    protected:
        virtual void computeCurrentDelta(const Eigen::VectorXs& _data,
                                         const Eigen::MatrixXs& _data_cov,
                                         const Eigen::VectorXs& _calib,
                                         const Scalar _dt,
                                         Eigen::VectorXs& _delta,
                                         Eigen::MatrixXs& _delta_cov,
                                         Eigen::MatrixXs& _jacobian_calib) override;
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta_preint,
                                    const Eigen::VectorXs& _delta,
                                    const Scalar _dt,
                                    Eigen::VectorXs& _delta_preint_plus_delta) override;
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta_preint,
                                    const Eigen::VectorXs& _delta,
                                    const Scalar _dt,
                                    Eigen::VectorXs& _delta_preint_plus_delta,
                                    Eigen::MatrixXs& _jacobian_delta_preint,
                                    Eigen::MatrixXs& _jacobian_delta) override;
        virtual void statePlusDelta(const Eigen::VectorXs& _x,
                                const Eigen::VectorXs& _delta,
                                const Scalar _dt,
                                Eigen::VectorXs& _x_plus_delta ) override;
        virtual Eigen::VectorXs deltaZero() const override;
        virtual Motion interpolate(const Motion& _motion_ref,
                                   Motion& _motion,
                                   TimeStamp& _ts) override;
        virtual bool voteForKeyFrame() override;
        virtual CaptureMotionPtr createCapture(const TimeStamp& _ts,
                                               const SensorBasePtr& _sensor,
                                               const VectorXs& _data,
                                               const MatrixXs& _data_cov,
                                               const FrameBasePtr& _frame_origin) override;
        virtual FeatureBasePtr createFeature(CaptureMotionPtr _capture_motion) override;
        virtual ConstraintBasePtr emplaceConstraint(FeatureBasePtr _feature_motion,
                                                    CaptureBasePtr _capture_origin) override;

    protected:

        // keyframe voting parameters
        Scalar max_time_span_;  // maximum time between keyframes
        Size   max_buff_length_;// maximum buffer size before keyframe
        Scalar dist_traveled_;  // maximum linear motion between keyframes
        Scalar angle_turned_;   // maximum rotation between keyframes
        bool voting_active_;    // IMU will be voting for KeyFrame only if this is true


    public:
        //getters
        Scalar getMaxTimeSpan() const;
        Scalar getMaxBuffLength() const;
        Scalar getDistTraveled() const;
        Scalar getAngleTurned() const;

        //for factory
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr sensor_ptr = nullptr);
};

}

/////////////////////////////////////////////////////////
// IMPLEMENTATION. Put your implementation includes here
/////////////////////////////////////////////////////////

// Wolf
#include "constraint_imu.h"
#include "state_block.h"
#include "rotations.h"
#include "imu_tools.h"


namespace wolf{

inline Eigen::VectorXs ProcessorIMU::deltaZero() const
{
    return (Eigen::VectorXs(10) << 0,0,0,  0,0,0,1,  0,0,0 ).finished(); // p, q, v
}

inline Scalar ProcessorIMU::getMaxTimeSpan() const
{
    return max_time_span_;
}

inline Scalar ProcessorIMU::getMaxBuffLength() const
{
    return max_buff_length_;
}

inline Scalar ProcessorIMU::getDistTraveled() const
{
    return dist_traveled_;
}

inline Scalar ProcessorIMU::getAngleTurned() const
{
    return angle_turned_;
}

} // namespace wolf

#endif // PROCESSOR_IMU_H
