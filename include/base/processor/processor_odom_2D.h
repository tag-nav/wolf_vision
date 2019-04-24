/**
 * \file processor_odom_2D.h
 *
 *  Created on: Apr 15, 2016
 *      \author: jvallve
 */

#ifndef SRC_PROCESSOR_ODOM_2D_H_
#define SRC_PROCESSOR_ODOM_2D_H_

#include "base/processor/processor_motion.h"
#include "base/capture/capture_odom_2D.h"
#include "base/factor/factor_odom_2D.h"
#include "base/math/rotations.h"

namespace wolf {
    
WOLF_PTR_TYPEDEFS(ProcessorOdom2D);
WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsOdom2D);

struct ProcessorParamsOdom2D : public ProcessorParamsMotion
{
    Scalar cov_det                  = 1.0;      // 1 rad^2
    Scalar unmeasured_perturbation_std = 0.001;    // no particular dimension: the same for displacement and angle
};

class ProcessorOdom2D : public ProcessorMotion
{
    public:
        ProcessorOdom2D(ProcessorParamsOdom2DPtr _params);
        virtual ~ProcessorOdom2D();
        virtual void configure(SensorBasePtr _sensor) override { };

        virtual bool voteForKeyFrame() override;

    protected:
        virtual void computeCurrentDelta(const Eigen::VectorXs& _data,
                                         const Eigen::MatrixXs& _data_cov,
                                         const Eigen::VectorXs& _calib,
                                         const Scalar _dt,
                                         Eigen::VectorXs& _delta,
                                         Eigen::MatrixXs& _delta_cov,
                                         Eigen::MatrixXs& _jacobian_calib) override;
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1,
                                    const Eigen::VectorXs& _delta2,
                                    const Scalar _Dt2,
                                    Eigen::VectorXs& _delta1_plus_delta2) override;
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1,
                                    const Eigen::VectorXs& _delta2,
                                    const Scalar _Dt2,
                                    Eigen::VectorXs& _delta1_plus_delta2,
                                    Eigen::MatrixXs& _jacobian1,
                                    Eigen::MatrixXs& _jacobian2) override;
        virtual void statePlusDelta(const Eigen::VectorXs& _x,
                                    const Eigen::VectorXs& _delta,
                                    const Scalar _Dt,
                                    Eigen::VectorXs& _x_plus_delta) override;
        virtual Eigen::VectorXs deltaZero() const override;
        virtual Motion interpolate(const Motion& _ref,
                                   Motion& _second,
                                   TimeStamp& _ts) override;
        virtual Motion interpolate(const Motion& _ref1,
                                   const Motion& _ref2,
                                   const TimeStamp& _ts,
                                   Motion& _second) override;

        virtual CaptureMotionPtr createCapture(const TimeStamp& _ts,
                                               const SensorBasePtr& _sensor,
                                               const VectorXs& _data,
                                               const MatrixXs& _data_cov,
                                               const FrameBasePtr& _frame_origin) override;
        virtual FeatureBasePtr createFeature(CaptureMotionPtr _capture_motion) override;
        virtual FactorBasePtr emplaceFactor(FeatureBasePtr _feature,
                                                    CaptureBasePtr _capture_origin) override;

    protected:
        ProcessorParamsOdom2DPtr params_odom_2D_;
        MatrixXs unmeasured_perturbation_cov_;

        // Factory method
    public:
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr sensor_ptr = nullptr);
};

inline Eigen::VectorXs ProcessorOdom2D::deltaZero() const
{
    return Eigen::VectorXs::Zero(delta_size_);
}

} // namespace wolf

#endif /* SRC_PROCESSOR_ODOM_2D_H_ */
