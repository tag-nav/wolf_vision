#ifndef FACTOR_EPIPOLAR_H
#define FACTOR_EPIPOLAR_H

#include "vision/sensor/sensor_camera.h"

#include <core/factor/factor_autodiff.h>
#include <core/math/rotations.h>

namespace wolf {

WOLF_PTR_TYPEDEFS(FactorEpipolar);

class FactorEpipolar : public FactorAutodiff<FactorEpipolar, 1, 3, 4, 3, 4, 3, 4>
{
    public:
        FactorEpipolar(const FeatureBasePtr& _feature_ptr,
                       const FeatureBasePtr& _feature_other_ptr,
                       const ProcessorBasePtr& _processor_ptr,
                       bool _apply_loss_function,
                       FactorStatus _status = FAC_ACTIVE);

        virtual ~FactorEpipolar() = default;

        virtual std::string getTopology() const override
        {
            return std::string("GEOM");
        }

        template<typename T>
        bool operator ()(const T* const _frame_own_p,
                         const T* const _frame_own_o,
                         const T* const _frame_other_p,
                         const T* const _frame_other_o,
                         const T* const _sensor_p,
                         const T* const _sensor_o,
                               T*       _residuals) const;

    private:
        SensorCameraPtr camera_;
        Eigen::Matrix3d K_inv_; ///< Intrinsic matrix and its inverse

};

inline FactorEpipolar::FactorEpipolar(const FeatureBasePtr& _feature_ptr,
                                      const FeatureBasePtr& _feature_other_ptr,
                                      const ProcessorBasePtr& _processor_ptr,
                                      bool _apply_loss_function,
                                      FactorStatus _status) :
        FactorAutodiff<FactorEpipolar, 1, 3, 4, 3, 4, 3, 4>("FEATURE EPIPOLAR",
                                                            nullptr,
                                                            nullptr,
                                                            _feature_other_ptr,
                                                            nullptr,
                                                            _processor_ptr,
                                                            _apply_loss_function,
                                                            _status,
                                                            _feature_ptr->getCapture()->getFrame()->getP(),
                                                            _feature_ptr->getCapture()->getFrame()->getO(),
                                                            _feature_other_ptr->getCapture()->getFrame()->getP(),
                                                            _feature_other_ptr->getCapture()->getFrame()->getO(),
                                                            _feature_ptr->getCapture()->getSensorP(),
                                                            _feature_ptr->getCapture()->getSensorO() )
{
    camera_ = std::static_pointer_cast<SensorCamera>(_feature_ptr->getCapture()->getSensor());

    K_inv_  = camera_->getIntrinsicMatrix().inverse();
}

template<typename T>
inline bool FactorEpipolar::operator ()(const T* const _frame_own_p,
                                        const T* const _frame_own_o,
                                        const T* const _frame_other_p,
                                        const T* const _frame_other_o,
                                        const T* const _sensor_p,
                                        const T* const _sensor_o,
                                              T*       _residuals) const
{
    using namespace Eigen;

    // Map input and output
    Map<const Matrix<T, 3, 1> > p_own(_frame_own_p);
    Map<const Quaternion<T> >   q_own(_frame_own_o);
    Map<const Matrix<T, 3, 1> > p_other(_frame_other_p);
    Map<const Quaternion<T> >   q_other(_frame_other_o);
    Map<const Matrix<T, 3, 1> > p_sen(_sensor_p);
    Map<const Quaternion<T> >   q_sen(_sensor_o);
    Map<Matrix<T, 1, 1> >       residual(_residuals);

    // Compose frames get to camera frames in absolute reference
    Matrix<T, 3, 1> cam_own_p   = p_own + q_own * p_sen;
    Quaternion<T>   cam_own_q   = q_own * q_sen;
    Matrix<T, 3, 1> cam_other_p = p_other + q_own * p_sen;
    Quaternion<T>   cam_other_q = q_other * q_sen;

    // Compute essential matrix and fundamental matrix
    /* Essential matrix convention disambiguation -- beware of literature existing conventions are a mess!
     *
     * C1 is a camera at the origin frame or reference
     * C2 is another camera
     * C2 is specified by R and t wrt C1 so that
     *   t is the position    of C2 wrt C1
     *   R is the orientation of C2 wrt C1
     * There is a 3D point P, noted P1 when expressed in C1 and P2 when expressed in C2:
     *   P1 = t + R * P2
     *
     * Co-planarity condition: a' * (b x c) = 0 with {a,b,c} three co-planar vectors.
     *
     * The three vectors are the ones defined by the triangle C1-C2-P at positions 0, t and P1 respectively:
     *
     *        b
     *     0 --- t
     *       \   |
     *     r1 \  | r2
     *         \ |
     *          P1
     *
     *   baseline: b  = t  - 0 = t
     *   ray 1   : r1 = P1 - 0 = P1
     *   ray 2   : r2 = P1 - t = R*P2;
     *
     * so,
     *
     *   (r1)' * (b x r2) = 0 , which develops as:
     *
     *   P1' * (t x (R*P2))  = 0
     *   P1' * [t]x * R * P2 = 0
     *   P1' * c1Ec2 * P2    = 0 <--- Epipolar factor
     *
     * therefore:
     *   c1Ec2 = [t]x * R        <--- Essential matrix
     *
     * If the two cameras are the same with matrix K, and observed pixels are u1 and u2, then
     *   P1 = K.inv * u1
     *   P2 = K.inv * u2
     *
     * and therefore
     *   u1 * K.inv.tr * c1Ec2 * K.inv * u2 = 0
     *   u1 * c1Fc2 * u2 = 0
     *
     * where c1Fc2 = K.inv.tr * c1Ec2 * K.inv is the fundamental matrix.
     */

    Matrix<T, 3, 1> t = cam_own_q.conjugate() * (cam_other_p - cam_own_p); // other with respect to own
    Matrix<T, 3, 3> R = (cam_own_q.conjugate() * cam_other_q).matrix(); // other with respect to own

    // Fundamental matrix
    auto K_inv = K_inv_.cast<T>();
    Matrix<T, 3, 3> own_F_other = K_inv.transpose() * wolf::skew(t) * R * K_inv;

    // own and other pixels
    Matrix<T, 3, 1> u_own, u_other;
    u_own   << getFeature()     ->getMeasurement().cast<T>(), (T)(1.0);
    u_other << getFeatureOther()->getMeasurement().cast<T>(), (T)(1.0);

    // Jacobians of error e = u_own.tr * own_F_other * u_other
    Matrix<T, 1, 3> J_e_uown    = (own_F_other * u_other).transpose();
    Matrix<T, 1, 3> J_e_uother  = u_own.transpose() * own_F_other;

    // Covariance of error
    Matrix<T, 1, 1> Q_e =
            J_e_uown  .template block<1, 2>(0, 0) * getFeature()     ->getMeasurementCovariance().cast<T>() * J_e_uown  .template block<1, 2>(0, 0).template transpose()
          + J_e_uother.template block<1, 2>(0, 0) * getFeatureOther()->getMeasurementCovariance().cast<T>() * J_e_uother.template block<1, 2>(0, 0).template transpose();

    // Sqrt of inverse of the covariance is sigma^-1
    T sigma_inv = 1.0 / sqrt(Q_e(0, 0));

    // Final residual
    // residual = sigma_inv * u_own.transpose() * own_F_other * u_other; // use expression below which is the same but reuses computations
    residual = sigma_inv * J_e_uother * u_other;
    return true;
}

} // namespace wolf

#endif // FACTOR_EPIPOLAR_H
