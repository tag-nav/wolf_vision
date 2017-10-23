#ifndef CONSTRAINT_IMU_THETA_H_
#define CONSTRAINT_IMU_THETA_H_

//Wolf includes
#include "constraint_autodiff.h"
#include "feature_imu.h"
#include "frame_imu.h"
#include "sensor_imu.h"
#include "rotations.h"

//Eigen include


namespace wolf {
    
WOLF_PTR_TYPEDEFS(ConstraintIMU);

//class
class ConstraintIMU : public ConstraintAutodiff<ConstraintIMU, 15, 3, 4, 3, 6, 3, 4, 3, 6>
{
    public:
        ConstraintIMU(const FeatureIMUPtr& _ftr_ptr,
                      const CaptureIMUPtr& _capture_origin_ptr,
                      const ProcessorBasePtr& _processor_ptr = nullptr,
                      bool _apply_loss_function = false,
                      ConstraintStatus _status = CTR_ACTIVE);

        virtual ~ConstraintIMU() = default;

        /* \brief : compute the residual from the state blocks being iterated by the solver.
            -> computes the expected measurement
            -> compares the actual measurement with the expected one
            -> weights the result with the covariance of the noise (residual = sqrt_info_matrix * err;)
        */
        template<typename T>
        bool operator ()(const T* const _p1,
                         const T* const _o1,
                         const T* const _v1,
                         const T* const _b1,
                         const T* const _p2,
                         const T* const _o2,
                         const T* const _v2,
                         const T* const _b2,
                         T* _residuals) const;
        
        /* \brief : compute the residual from the state blocks being iterated by the solver. (same as operator())
            -> computes the expected measurement
            -> compares the actual measurement with the expected one
            -> weights the result with the covariance of the noise (residual = sqrt_info_matrix * err;)
         * params :
         * Vector3s _p1 : position in imu frame
         * Vector4s _q1 : orientation quaternion in imu frame
         * Vector3s _v1 : velocity in imu frame
         * Vector3s _ab : accelerometer bias in imu frame
         * Vector3s _wb : gyroscope bias in imu frame
         * Vector3s _p2 : position in current frame
         * Vector4s _q2 : orientation quaternion in current frame
         * Vector3s _v2 : velocity in current frame
         * Matrix<9,1, wolf::Scalar> _residuals : to retrieve residuals (POV) O is rotation vector... NOT A QUATERNION
        */
        template<typename D1, typename D2, typename D3>
        bool getResiduals(const Eigen::MatrixBase<D1> & _p1,
                          const Eigen::QuaternionBase<D2> & _q1,
                          const Eigen::MatrixBase<D1> & _v1,
                          const Eigen::MatrixBase<D1> & _ab1,
                          const Eigen::MatrixBase<D1> & _wb1,
                          const Eigen::MatrixBase<D1> & _p2,
                          const Eigen::QuaternionBase<D2> & _q2,
                          const Eigen::MatrixBase<D1> & _v2,
                          const Eigen::MatrixBase<D1> & _ab2,
                          const Eigen::MatrixBase<D1> & _wb2,
                          Eigen::MatrixBase<D3> & _residuals) const;

        virtual JacobianMethod getJacobianMethod() const override;

        /* Function expectation(...)
         * params :
         * Vector3s _p1 : position in imu frame
         * Vector4s _q1 : orientation quaternion in imu frame
         * Vector3s _v1 : velocity in imu frame
         * Vector3s _ab : accelerometer bias in imu frame
         * Vector3s _wb : gyroscope bias in imu frame
         * Vector3s _p2 : position in current frame
         * Vector4s _q2 : orientation quaternion in current frame
         * Vector3s _v2 : velocity in current frame
         * Matrix<10,1, wolf::Scalar> _expectation : to retrieve resulting expectation (PVQ)
        */
        template<typename D1, typename D2, typename D3, typename D4>
        void expectation(const Eigen::MatrixBase<D1> & _p1,
                         const Eigen::QuaternionBase<D2> & _q1,
                         const Eigen::MatrixBase<D1> & _v1,
                         const Eigen::MatrixBase<D1> & _p2,
                         const Eigen::QuaternionBase<D2> & _q2,
                         const Eigen::MatrixBase<D1> & _v2,
                         typename D1::Scalar _dt,
                         Eigen::MatrixBase<D3> & _pe,
                         Eigen::QuaternionBase<D4> & _qe,
                         Eigen::MatrixBase<D3> & _ve) const;

        /* \brief : return the expected value given the state blocks in the wolf tree
            current frame data is taken from constraintIMU object.
            IMU frame is taken from wolf tree
        */
        Eigen::VectorXs expectation() const;


    private:
        /// Preintegrated delta
        Eigen::Vector3s dp_preint_;
        Eigen::Quaternions dq_preint_;
        Eigen::Vector3s dv_preint_;

        // Biases used during preintegration
        Eigen::Vector3s acc_bias_preint_;
        Eigen::Vector3s gyro_bias_preint_;

        // Jacobians of preintegrated deltas wrt biases
        Eigen::Matrix3s dDp_dab_;
        Eigen::Matrix3s dDv_dab_;
        Eigen::Matrix3s dDp_dwb_;
        Eigen::Matrix3s dDv_dwb_;
        Eigen::Matrix3s dDq_dwb_;

        /// Metrics
        const wolf::Scalar dt_; ///< delta-time and delta-time-squared between keyframes
        const Eigen::Vector3s g_; ///< acceleration of gravity in World frame
        const wolf::Scalar ab_rate_stdev_, wb_rate_stdev_; //stdev for standard_deviation (= sqrt(variance))
        
        /* bias covariance and bias residuals
         *
         * continuous bias covariance matrix for accelerometer would then be A_r = diag(ab_stdev_^2, ab_stdev_^2, ab_stdev_^2)
         * To compute bias residuals, we will need to do (sqrt(A_r)).inverse() * ab_error
         *
         * In our case, we introduce time 'distance' in the computation of this residual (SEE FORSTER17), thus we have to use the discret covariance matrix
         * discrete bias covariance matrix for accelerometer : A_r_dt = dt_ * A_r
         * taking the square root for bias residuals : sqrt_A_r_dt = sqrt(dt_ * A_r) = sqrt(dt_) * sqrt(A_r)
         * then with the inverse : sqrt_A_r_dt_inv = (sqrt(dt_ * A_r)).inverse() = (1/sqrt(dt_)) * sqrt(A_r).inverse()
         *
         * same logic for gyroscope bias
         */ 
        const Eigen::Matrix3s sqrt_A_r_dt_inv;
        const Eigen::Matrix3s sqrt_W_r_dt_inv;
};

inline ConstraintIMU::ConstraintIMU(const FeatureIMUPtr& _ftr_ptr,
                                    const CaptureIMUPtr& _cap_origin_ptr,
                                    const ProcessorBasePtr& _processor_ptr,
                                    bool _apply_loss_function,
                                    ConstraintStatus _status) :
                ConstraintAutodiff<ConstraintIMU, 15, 3, 4, 3, 6, 3, 4, 3, 6>( // ...
                        CTR_IMU,
                        _cap_origin_ptr->getFramePtr(),
                        _cap_origin_ptr,
                        nullptr,
                        nullptr,
                        _processor_ptr,
                        _apply_loss_function,
                        _status,
                        _cap_origin_ptr->getFramePtr()->getPPtr(),
                        _cap_origin_ptr->getFramePtr()->getOPtr(),
                        _cap_origin_ptr->getFramePtr()->getVPtr(),
                        _cap_origin_ptr->getSensorIntrinsicPtr(),
                        _ftr_ptr->getFramePtr()->getPPtr(),
                        _ftr_ptr->getFramePtr()->getOPtr(),
                        _ftr_ptr->getFramePtr()->getVPtr(),
                        _ftr_ptr->getCapturePtr()->getSensorIntrinsicPtr()),
        dp_preint_(_ftr_ptr->dp_preint_), // dp, dv, dq at preintegration time
        dq_preint_(_ftr_ptr->dq_preint_),
        dv_preint_(_ftr_ptr->dv_preint_),
        acc_bias_preint_(_ftr_ptr->acc_bias_preint_), // state biases at preintegration time
        gyro_bias_preint_(_ftr_ptr->gyro_bias_preint_),
        dDp_dab_(_ftr_ptr->jacobian_bias_.block(0,0,3,3)), // Jacs of dp dv dq wrt biases
        dDv_dab_(_ftr_ptr->jacobian_bias_.block(6,0,3,3)),
        dDp_dwb_(_ftr_ptr->jacobian_bias_.block(0,3,3,3)),
        dDv_dwb_(_ftr_ptr->jacobian_bias_.block(6,3,3,3)),
        dDq_dwb_(_ftr_ptr->jacobian_bias_.block(3,3,3,3)),
        dt_(_ftr_ptr->getFramePtr()->getTimeStamp() - _cap_origin_ptr->getTimeStamp()),
        g_(wolf::gravity()),
        ab_rate_stdev_(std::static_pointer_cast<SensorIMU>(_ftr_ptr->getCapturePtr()->getSensorPtr())->getAbRateStdev()),
        wb_rate_stdev_(std::static_pointer_cast<SensorIMU>(_ftr_ptr->getCapturePtr()->getSensorPtr())->getWbRateStdev()),
        sqrt_A_r_dt_inv((Eigen::Matrix3s::Identity() * ab_rate_stdev_ * sqrt(dt_)).inverse()),
        sqrt_W_r_dt_inv((Eigen::Matrix3s::Identity() * wb_rate_stdev_ * sqrt(dt_)).inverse())
{
    setType("IMU");
}

template<typename T>
inline bool ConstraintIMU::operator ()(const T* const _p1,
                                       const T* const _q1,
                                       const T* const _v1,
                                       const T* const _b1,
                                       const T* const _p2,
                                       const T* const _q2,
                                       const T* const _v2,
                                       const T* const _b2,
                                       T* _residuals) const
{
    using namespace Eigen;

    // MAPS
    Map<const Matrix<T,3,1> > p1(_p1);
    Map<const Quaternion<T> > q1(_q1);
    Map<const Matrix<T,3,1> > v1(_v1);
    Map<const Matrix<T,3,1> > ab1(_b1);
    Map<const Matrix<T,3,1> > wb1(_b1 + 3);

    Map<const Matrix<T,3,1> > p2(_p2);
    Map<const Quaternion<T> > q2(_q2);
    Map<const Matrix<T,3,1> > v2(_v2);
    Map<const Matrix<T,3,1> > ab2(_b2);
    Map<const Matrix<T,3,1> > wb2(_b2 + 3);

    Map<Matrix<T,15,1> > residuals(_residuals);

    getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, residuals);

    return true;
}

template<typename D1, typename D2, typename D3>
inline bool ConstraintIMU::getResiduals(const Eigen::MatrixBase<D1> & _p1,
                                        const Eigen::QuaternionBase<D2> & _q1,
                                        const Eigen::MatrixBase<D1> & _v1,
                                        const Eigen::MatrixBase<D1> & _ab1,
                                        const Eigen::MatrixBase<D1> & _wb1,
                                        const Eigen::MatrixBase<D1> & _p2,
                                        const Eigen::QuaternionBase<D2> & _q2,
                                        const Eigen::MatrixBase<D1> & _v2,
                                        const Eigen::MatrixBase<D1> & _ab2,
                                        const Eigen::MatrixBase<D1> & _wb2,
                                        Eigen::MatrixBase<D3> & _residuals) const
{
    //needed typedefs
    typedef typename D2::Scalar T;

    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D3, 15);

    // Expected delta from states
    Eigen::Matrix<T, 3, 1 >   dp_exp;
    Eigen::Quaternion<T>      dq_exp;
    Eigen::Matrix<T, 3, 1>    dv_exp;

    expectation(_p1, _q1, _v1, _p2, _q2, _v2, (T)dt_, dp_exp, dq_exp, dv_exp);

    // Corrected integrated delta: delta_corr = delta_preint (+) J_bias * (bias_current - bias_preint)
    Eigen::Matrix<T,3,1> dp_correct = dp_preint_.cast<T>() + dDp_dab_.cast<T>() * (_ab1 - acc_bias_preint_.cast<T>()) + dDp_dwb_.cast<T>() * (_wb1 - gyro_bias_preint_.cast<T>());
    Eigen::Matrix<T,3,1> dv_correct = dv_preint_.cast<T>() + dDv_dab_.cast<T>() * (_ab1 - acc_bias_preint_.cast<T>()) + dDv_dwb_.cast<T>() * (_wb1 - gyro_bias_preint_.cast<T>());
    Eigen::Matrix<T,3,1> do_step    = dDq_dwb_  .cast<T>() * (_wb1 - gyro_bias_preint_.cast<T>());
    Eigen::Quaternion<T> dq_correct = dq_preint_.cast<T>() * v2q(do_step);

    // Delta error in minimal form: d_min = log(delta_pred (-) delta_corr)
    // Note the Dt here is zero because it's the delta-time between the same time stamps!
    Eigen::Matrix<T, 9, 1> dpov_error;
    Eigen::Map<Eigen::Matrix<T, 3, 1> >   dp_error(dpov_error.data()    );
    Eigen::Map<Eigen::Matrix<T, 3, 1> >   do_error(dpov_error.data() + 3);
    Eigen::Map<Eigen::Matrix<T, 3, 1> >   dv_error(dpov_error.data() + 6);

    imu::diff(dp_exp, dq_exp, dv_exp, dp_correct, dq_correct, dv_correct, dp_error, do_error, dv_error);

    // Errors between biases
    Eigen::Matrix<T,3,1> ab_error(_ab1 - _ab2); // KF1.bias - KF2.bias
    Eigen::Matrix<T,3,1> wb_error(_wb1 - _wb2);

    // Residuals are the weighted errors
    Eigen::Matrix<T,9,1> dpov_residuals(getMeasurementSquareRootInformationTransposed().cast<T>()  * dpov_error);

    // Assign to residuals vector
    _residuals.head(3)       = dpov_residuals.head(3);
    _residuals.segment(3,3)  = dpov_residuals.segment(3,3);
    _residuals.segment(6,3)  = dpov_residuals.tail(3);
    _residuals.segment(9,3)  = sqrt_A_r_dt_inv.cast<T>() * ab_error;
    _residuals.tail(3)       = sqrt_A_r_dt_inv.cast<T>() * wb_error;

    return true;
}

inline Eigen::VectorXs ConstraintIMU::expectation() const
{
    FrameBasePtr frm_current = getFeaturePtr()->getFramePtr();
    FrameBasePtr frm_previous = getFrameOtherPtr();

    //get information on current_frame in the ConstraintIMU
    const Eigen::Vector3s frame_current_pos = frm_current->getPPtr()->getState();
    const Eigen::Quaternions frame_current_ori(frm_current->getOPtr()->getState().data());
    const Eigen::Vector3s frame_current_vel = frm_current->getVPtr()->getState();

    // get info on previous frame in the ConstraintIMU
    const Eigen::Vector3s frame_previous_pos = (frm_previous->getPPtr()->getState());
    const Eigen::Quaternions frame_previous_ori  (frm_previous->getOPtr()->getState().data());
    const Eigen::Vector3s frame_previous_vel = (frm_previous->getVPtr()->getState());

    // Define results vector and Map bits over it
    Eigen::Matrix<wolf::Scalar, 10, 1> exp;
    Eigen::Map<Eigen::Matrix<wolf::Scalar, 3, 1> >   pe(exp.data()    );
    Eigen::Map<Eigen::Quaternion<wolf::Scalar> >     qe(exp.data() + 3);
    Eigen::Map<Eigen::Matrix<wolf::Scalar, 3, 1> >   ve(exp.data() + 7);

    expectation(frame_previous_pos, frame_previous_ori, frame_previous_vel,
                frame_current_pos, frame_current_ori, frame_current_vel,
                dt_,
                pe, qe, ve);

    return exp;
}

template<typename D1, typename D2, typename D3, typename D4>
inline void ConstraintIMU::expectation(const Eigen::MatrixBase<D1> &        _p1,
                                       const Eigen::QuaternionBase<D2> &    _q1,
                                       const Eigen::MatrixBase<D1> &        _v1,
                                       const Eigen::MatrixBase<D1> &        _p2,
                                       const Eigen::QuaternionBase<D2> &    _q2,
                                       const Eigen::MatrixBase<D1> &        _v2,
                                       typename D1::Scalar                  _dt,
                                       Eigen::MatrixBase<D3> &              _pe,
                                       Eigen::QuaternionBase<D4> &          _qe,
                                       Eigen::MatrixBase<D3> &              _ve) const
{
    imu::betweenStates(_p1, _q1, _v1, _p2, _q2, _v2, _dt, _pe, _qe, _ve);
}

inline JacobianMethod ConstraintIMU::getJacobianMethod() const
{
    return JAC_AUTO;
}


} // namespace wolf

#endif
