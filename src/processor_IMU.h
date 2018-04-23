#ifndef PROCESSOR_IMU_H
#define PROCESSOR_IMU_H

// Wolf
#include "capture_IMU.h"
#include "feature_IMU.h"
#include "processor_motion.h"


namespace wolf {
WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsIMU);

struct ProcessorParamsIMU : public ProcessorParamsMotion
{
        //
};

WOLF_PTR_TYPEDEFS(ProcessorIMU);
    
//class
class ProcessorIMU : public ProcessorMotion{
    public:
        ProcessorIMU(ProcessorParamsIMUPtr _params_motion_IMU);
        virtual ~ProcessorIMU();
        virtual void configure(SensorBasePtr _sensor) override { };

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
        ProcessorParamsIMUPtr params_motion_IMU_;

        // keyframe voting parameters
//        Scalar max_time_span_;  // maximum time between keyframes
//        Size   max_buff_length_;// maximum buffer size before keyframe
//        Scalar dist_traveled_;  // maximum linear motion between keyframes
//        Scalar angle_turned_;   // maximum rotation between keyframes


    public:
        //for factory
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr sensor_ptr = nullptr);
};

}

/////////////////////////////////////////////////////////
// IMPLEMENTATION. Put your implementation includes here
/////////////////////////////////////////////////////////

// Wolf
#include "constraint_IMU.h"
#include "state_block.h"
#include "rotations.h"
#include "IMU_tools.h"


namespace wolf{

inline Eigen::VectorXs ProcessorIMU::deltaZero() const
{
    return (Eigen::VectorXs(10) << 0,0,0,  0,0,0,1,  0,0,0 ).finished(); // p, q, v
}

} // namespace wolf

#endif // PROCESSOR_IMU_H
