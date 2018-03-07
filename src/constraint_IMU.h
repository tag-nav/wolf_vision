#ifndef CONSTRAINT_IMU_THETA_H_
#define CONSTRAINT_IMU_THETA_H_

//Wolf includes
#include <feature_IMU.h>
#include <sensor_IMU.h>
#include "constraint_autodiff.h"
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
            -> corrects actual measurement with new bias
            -> compares the corrected measurement with the expected one
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
                         T* _res) const;
        
        /* \brief : compute the residual from the state blocks being iterated by the solver. (same as operator())
            -> computes the expected measurement
            -> corrects actual measurement with new bias
            -> compares the corrected measurement with the expected one
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
         * Matrix<9,1, wolf::Scalar> _res : to retrieve residuals (POV) O is rotation vector... NOT A QUATERNION
        */
        template<typename D1, typename D2, typename D3>
        bool residual(const Eigen::MatrixBase<D1> &     _p1,
                      const Eigen::QuaternionBase<D2> & _q1,
                      const Eigen::MatrixBase<D1> &     _v1,
                      const Eigen::MatrixBase<D1> &     _ab1,
                      const Eigen::MatrixBase<D1> &     _wb1,
                      const Eigen::MatrixBase<D1> &     _p2,
                      const Eigen::QuaternionBase<D2> & _q2,
                      const Eigen::MatrixBase<D1> &     _v2,
                      const Eigen::MatrixBase<D1> &     _ab2,
                      const Eigen::MatrixBase<D1> &     _wb2,
                      Eigen::MatrixBase<D3> &           _res) const;

        /* Function expectation(...)
         * params :
         * Vector3s _p1 : position in imu frame
         * Vector4s _q1 : orientation quaternion in imu frame
         * Vector3s _v1 : velocity in imu frame
         * Vector3s _p2 : position in current frame
         * Vector4s _q2 : orientation quaternion in current frame
         * Vector3s _v2 : velocity in current frame
         * Scalar   _dt : time interval between the two states
         *          _pe : expected position delta
         *          _qe : expected quaternion delta
         *          _ve : expected velocity delta
        */
        template<typename D1, typename D2, typename D3, typename D4>
        void expectation(const Eigen::MatrixBase<D1> &      _p1,
                         const Eigen::QuaternionBase<D2> &  _q1,
                         const Eigen::MatrixBase<D1> &      _v1,
                         const Eigen::MatrixBase<D1> &      _p2,
                         const Eigen::QuaternionBase<D2> &  _q2,
                         const Eigen::MatrixBase<D1> &      _v2,
                         typename D1::Scalar                _dt,
                         Eigen::MatrixBase<D3> &            _pe,
                         Eigen::QuaternionBase<D4> &        _qe,
                         Eigen::MatrixBase<D3> &            _ve) const;

        /* \brief : return the expected delta given the state blocks in the wolf tree
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

///////////////////// IMPLEMENTAITON ////////////////////////////

inline ConstraintIMU::ConstraintIMU(const FeatureIMUPtr&    _ftr_ptr,
                                    const CaptureIMUPtr&    _cap_origin_ptr,
                                    const ProcessorBasePtr& _processor_ptr,
                                    bool                    _apply_loss_function,
                                    ConstraintStatus        _status) :
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
        dp_preint_(_ftr_ptr->getMeasurement().head<3>()), // dp, dv, dq at preintegration time
        dq_preint_(_ftr_ptr->getMeasurement().data()+3),
        dv_preint_(_ftr_ptr->getMeasurement().tail<3>()),
        acc_bias_preint_(_ftr_ptr->getAccBiasPreint()), // state biases at preintegration time
        gyro_bias_preint_(_ftr_ptr->getGyroBiasPreint()),
        dDp_dab_(_ftr_ptr->getJacobianBias().block(0,0,3,3)), // Jacs of dp dv dq wrt biases
        dDv_dab_(_ftr_ptr->getJacobianBias().block(6,0,3,3)),
        dDp_dwb_(_ftr_ptr->getJacobianBias().block(0,3,3,3)),
        dDv_dwb_(_ftr_ptr->getJacobianBias().block(6,3,3,3)),
        dDq_dwb_(_ftr_ptr->getJacobianBias().block(3,3,3,3)),
        dt_(_ftr_ptr->getFramePtr()->getTimeStamp() - _cap_origin_ptr->getTimeStamp()),
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
                                       T* _res) const
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

    Map<Matrix<T,15,1> > res(_res);

    residual(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, res);

    return true;
}

template<typename D1, typename D2, typename D3>
inline bool ConstraintIMU::residual(const Eigen::MatrixBase<D1> &       _p1,
                                    const Eigen::QuaternionBase<D2> &   _q1,
                                    const Eigen::MatrixBase<D1> &       _v1,
                                    const Eigen::MatrixBase<D1> &       _ab1,
                                    const Eigen::MatrixBase<D1> &       _wb1,
                                    const Eigen::MatrixBase<D1> &       _p2,
                                    const Eigen::QuaternionBase<D2> &   _q2,
                                    const Eigen::MatrixBase<D1> &       _v2,
                                    const Eigen::MatrixBase<D1> &       _ab2,
                                    const Eigen::MatrixBase<D1> &       _wb2,
                                    Eigen::MatrixBase<D3> &             _res) const
{

    /*  Help for the IMU residual function
     *
     *  Notations:
     *    D_* : a motion in the Delta manifold                           -- implemented as 10-vectors with [Dp, Dq, Dv]
     *    T_* : a motion difference in the Tangent space to the manifold -- implemented as  9-vectors with [Dp, Do, Dv]
     *    b*  : a bias
     *    J*  : a Jacobian matrix
     *
     *  We use the following functions from imu_tools.h:
     *    D  = betweenStates(x1,x2,dt) : obtain a Delta from two states separated dt=t2-t1
     *    D2 = plus(D1,T)              : plus operator,  D2 = D1 (+) T
     *    T  = diff(D1,D2)             : minus operator, T  = D2 (-) D1
     *
     *  Two methods are possible (select with #define below this note) :
     *
     *  Computations common to methods 1 and 2:
     *    D_exp  = betweenStates(x1,x2,dt)   // Predict delta from states
     *    T_step = J_preint * (b - b_preint) // compute the delta correction step due to bias change
     *
     *  Method 1:
     *    D_corr = plus(D_preint, T_step)    // correct the pre-integrated delta with correction step due to bias change
     *    T_err  = diff(D_exp, D_corr)       // compare against expected delta
     *    res    = W.sqrt * T_err
     *
     *   results in:
     *    res    = W.sqrt * ( diff ( D_exp, plus(D_preint, J_preint * (b - b_preint) ) ) )
     *
     *  Method 2:
     *    T_diff = diff(D_preint, D_exp)     // compare pre-integrated against expected delta
     *    T_err  = T_diff - T_step           // the difference should match the correction step due to bias change
     *    res    = W.sqrt * err
     *
     *   results in :
     *    res    = W.sqrt * ( ( diff ( D_preint , D_exp ) ) - J_preint * (b - b_preint) )
     *
     *
     * NOTE: See optimization report at the end of this file for comparisons of both methods.
     */
//#define METHOD_1 // if commented, then METHOD_2 will be applied


    //needed typedefs
    typedef typename D1::Scalar T;

    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D3, 15);

    // 1. Expected delta from states
    Eigen::Matrix<T, 3, 1 >   dp_exp;
    Eigen::Quaternion<T>      dq_exp;
    Eigen::Matrix<T, 3, 1>    dv_exp;

    imu::betweenStates(_p1, _q1, _v1, _p2, _q2, _v2, (T)dt_, dp_exp, dq_exp, dv_exp);


    // 2. Corrected integrated delta: delta_corr = delta_preint (+) J_bias * (bias_current - bias_preint)

    // 2.a. Compute the delta step in tangent space:   step = J_bias * (bias - bias_preint)
    Eigen::Matrix<T, 9, 1> d_step;

    d_step.block(0,0,3,1) = dDp_dab_.cast<T>() * (_ab1 - acc_bias_preint_ .cast<T>()) + dDp_dwb_.cast<T>() * (_wb1 - gyro_bias_preint_.cast<T>());
    d_step.block(3,0,3,1) =                                                             dDq_dwb_.cast<T>() * (_wb1 - gyro_bias_preint_.cast<T>());
    d_step.block(6,0,3,1) = dDv_dab_.cast<T>() * (_ab1 - acc_bias_preint_ .cast<T>()) + dDv_dwb_.cast<T>() * (_wb1 - gyro_bias_preint_.cast<T>());

#ifdef METHOD_1 // method 1
    Eigen::Matrix<T, 3, 1> dp_step = d_step.block(0,0,3,1);
    Eigen::Matrix<T, 3, 1> do_step = d_step.block(3,0,3,1);
    Eigen::Matrix<T, 3, 1> dv_step = d_step.block(6,0,3,1);

    // 2.b. Add the correction step to the preintegrated delta:    delta_cor = delta_preint (+) step
    Eigen::Matrix<T,3,1> dp_correct;
    Eigen::Quaternion<T> dq_correct;
    Eigen::Matrix<T,3,1> dv_correct;

    imu::plus(dp_preint_.cast<T>(), dq_preint_.cast<T>(), dv_preint_.cast<T>(),
              dp_step, do_step, dv_step,
              dp_correct, dq_correct, dv_correct);


    // 3. Delta error in minimal form: D_err = diff(D_exp , D_corr)
    // Note the Dt here is zero because it's the delta-time between the same time stamps!
    Eigen::Matrix<T, 9, 1> d_error;
    Eigen::Map<Eigen::Matrix<T, 3, 1> >   dp_error(d_error.data()    );
    Eigen::Map<Eigen::Matrix<T, 3, 1> >   do_error(d_error.data() + 3);
    Eigen::Map<Eigen::Matrix<T, 3, 1> >   dv_error(d_error.data() + 6);

    Eigen::Matrix<T, 3, 3> J_do_dq1, J_do_dq2;
    Eigen::Matrix<T, 9, 9> J_err_corr;

//#define WITH_JAC
#ifdef WITH_JAC
    imu::diff(dp_exp, dq_exp, dv_exp, dp_correct, dq_correct, dv_correct, dp_error, do_error, dv_error, J_do_dq1, J_do_dq2);

    J_err_corr.setIdentity();
    J_err_corr.block(3,3,3,3) = J_do_dq2;

    // 4. Residuals are the weighted errors
    _res.head(9)       = J_err_corr.inverse().transpose() * getMeasurementSquareRootInformationUpper().cast<T>() * d_error;
#else
    imu::diff(dp_exp, dq_exp, dv_exp, dp_correct, dq_correct, dv_correct, dp_error, do_error, dv_error);
    _res.head(9)       = getMeasurementSquareRootInformationUpper().cast<T>() * d_error;
#endif

#else // method 2

    Eigen::Matrix<T, 9, 1> d_diff;
    Eigen::Map<Eigen::Matrix<T, 3, 1> >   dp_diff(d_diff.data()    );
    Eigen::Map<Eigen::Matrix<T, 3, 1> >   do_diff(d_diff.data() + 3);
    Eigen::Map<Eigen::Matrix<T, 3, 1> >   dv_diff(d_diff.data() + 6);

    imu::diff(dp_preint_.cast<T>(), dq_preint_.cast<T>(), dv_preint_.cast<T>(),
              dp_exp, dq_exp, dv_exp,
              dp_diff, do_diff, dv_diff);

    Eigen::Matrix<T, 9, 1> d_error;
    d_error << d_diff - d_step;

    // 4. Residuals are the weighted errors
    _res.head(9)       = getMeasurementSquareRootInformationUpper().cast<T>() * d_error;

#endif

    // Errors between biases
    Eigen::Matrix<T,3,1> ab_error(_ab1 - _ab2); // KF1.bias - KF2.bias
    Eigen::Matrix<T,3,1> wb_error(_wb1 - _wb2);

    // 4. Residuals are the weighted errors
    _res.segment(9,3)  = sqrt_A_r_dt_inv.cast<T>() * ab_error;
    _res.tail(3)       = sqrt_W_r_dt_inv.cast<T>() * wb_error;


    //////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////       PRINT VALUES       ///////////////////////////////////
#if 0
    // print values -----------------------
    Matrix<T, 10, 1> x1; x1 << _p1 , _q1.coeffs(), _v1;
    Matrix<T, 10, 1> x2; x2 << _p2 , _q2.coeffs(), _v2;
    print("x1 ", x1);
    print("x2 ", x2);
    Matrix<T, 6, 1> bp; bp << acc_bias_preint_.cast<T>(), gyro_bias_preint_.cast<T>();
    Matrix<T, 6, 1> b1; b1 << _ab1, _wb1;
    Matrix<T, 6, 1> b2; b2 << _ab2, _wb2;
    print("bp ", bp);
    print("b1 ", b1);
    print("b2 ", b2);
    Matrix<T, 10, 1> exp; exp << dp_exp , dq_exp.coeffs(), dv_exp;
    print("D exp", exp);
    Matrix<T, 10, 1> pre; pre << dp_preint_.cast<T>() , dq_preint_.cast<T>().coeffs(), dv_preint_.cast<T>();
    print("D preint", pre);
    Matrix<T, 9, 6> J_b_r0; J_b_r0.setZero();
    J_b_r0.block(0,0,3,3) = dDp_dab_.cast<T>();
    J_b_r0.block(0,3,3,3) = dDp_dwb_.cast<T>();
    J_b_r0.block(3,3,3,3) = dDq_dwb_.cast<T>();
    J_b_r0.block(6,0,3,3) = dDv_dab_.cast<T>();
    J_b_r0.block(6,3,3,3) = dDv_dwb_.cast<T>();
    print("J bias", J_b_r0);
    print("D step", d_step);
#ifndef METHOD_1
    Matrix<T, 9, 1> dif; dif << dp_diff , do_diff, dv_diff;
    print("D diff", dif);
#endif
    print("D err", d_error);
    WOLF_TRACE("-----------------------------------------")
#endif
    /////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////



    return true;
}


inline Eigen::VectorXs ConstraintIMU::expectation() const
{
    FrameBasePtr frm_current = getFeaturePtr()->getFramePtr();
    FrameBasePtr frm_previous = getFrameOtherPtr();

    //get information on current_frame in the ConstraintIMU
    const Eigen::Vector3s frame_current_pos    = (frm_current->getPPtr()->getState());
    const Eigen::Quaternions frame_current_ori   (frm_current->getOPtr()->getState().data());
    const Eigen::Vector3s frame_current_vel    = (frm_current->getVPtr()->getState());

    // get info on previous frame in the ConstraintIMU
    const Eigen::Vector3s frame_previous_pos   = (frm_previous->getPPtr()->getState());
    const Eigen::Quaternions frame_previous_ori  (frm_previous->getOPtr()->getState().data());
    const Eigen::Vector3s frame_previous_vel   = (frm_previous->getVPtr()->getState());

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


} // namespace wolf

#endif


/*
 * Optimization results
 * ================================================
 *
 *
 * Using gtest_IMU.cpp
 *
 * Conclusion: Residuals with method 1 and 2 are essentially identical, after exactly the same number of iterations.
 *
 * You can verify this by looking at the 'Iterations' and 'Final costs' in the Ceres reports below.
 *
 *
 *
 * With Method 1:
 *
[ RUN      ] Process_Constraint_IMU.Var_B1_B2_Invar_P1_Q1_V1_P2_Q2_V2
[trace][10:58:16] [gtest_IMU.cpp L488 : TestBody] Ceres Solver Report: Iterations: 3, Initial cost: 1.092909e+02, Final cost: 1.251480e-06, Termination: CONVERGENCE
[trace][10:58:16] [gtest_IMU.cpp L490 : TestBody] w * DT (rather be lower than 1.57 approx) = 0.5   1 1.5
[       OK ] Process_Constraint_IMU.Var_B1_B2_Invar_P1_Q1_V1_P2_Q2_V2 (53 ms)
[ RUN      ] Process_Constraint_IMU.Var_P1_Q1_V1_B1_B2_Invar_P2_Q2_V2
[trace][10:58:16] [gtest_IMU.cpp L564 : TestBody] Ceres Solver Report: Iterations: 16, Initial cost: 1.238969e+03, Final cost: 1.059256e-19, Termination: CONVERGENCE
[       OK ] Process_Constraint_IMU.Var_P1_Q1_V1_B1_B2_Invar_P2_Q2_V2 (56 ms)
[ RUN      ] Process_Constraint_IMU.Var_P1_Q1_B1_V2_B2_Invar_V1_P2_Q2
[trace][10:58:16] [gtest_IMU.cpp L638 : TestBody] Ceres Solver Report: Iterations: 17, Initial cost: 4.769588e+03, Final cost: 3.767740e-19, Termination: CONVERGENCE
[       OK ] Process_Constraint_IMU.Var_P1_Q1_B1_V2_B2_Invar_V1_P2_Q2 (50 ms)
[----------] 3 tests from Process_Constraint_IMU (159 ms total)

[----------] 2 tests from Process_Constraint_IMU_ODO
[ RUN      ] Process_Constraint_IMU_ODO.Var_P0_Q0_V0_B0_P1_Q1_B1__Invar_V1
[trace][10:58:16] [gtest_IMU.cpp L711 : TestBody] Ceres Solver Report: Iterations: 19, Initial cost: 6.842446e+03, Final cost: 1.867678e-22, Termination: CONVERGENCE
[       OK ] Process_Constraint_IMU_ODO.Var_P0_Q0_V0_B0_P1_Q1_B1__Invar_V1 (68 ms)
[ RUN      ] Process_Constraint_IMU_ODO.Var_P0_Q0_B0_P1_Q1_V1_B1__Invar_V0
[trace][10:58:16] [gtest_IMU.cpp L783 : TestBody] Ceres Solver Report: Iterations: 16, Initial cost: 1.363681e+04, Final cost: 1.879880e-20, Termination: CONVERGENCE
[       OK ] Process_Constraint_IMU_ODO.Var_P0_Q0_B0_P1_Q1_V1_B1__Invar_V0 (52 ms)
[----------] 2 tests from Process_Constraint_IMU_ODO (120 ms total)
*
*
*
* With Method 2:
*
[ RUN      ] Process_Constraint_IMU.Var_B1_B2_Invar_P1_Q1_V1_P2_Q2_V2
[trace][11:15:43] [gtest_IMU.cpp L488 : TestBody] Ceres Solver Report: Iterations: 3, Initial cost: 1.092909e+02, Final cost: 1.251479e-06, Termination: CONVERGENCE
[trace][11:15:43] [gtest_IMU.cpp L490 : TestBody] w * DT (rather be lower than 1.57 approx) = 0.5   1 1.5
[       OK ] Process_Constraint_IMU.Var_B1_B2_Invar_P1_Q1_V1_P2_Q2_V2 (37 ms)
[ RUN      ] Process_Constraint_IMU.Var_P1_Q1_V1_B1_B2_Invar_P2_Q2_V2
[trace][11:15:43] [gtest_IMU.cpp L564 : TestBody] Ceres Solver Report: Iterations: 16, Initial cost: 1.238985e+03, Final cost: 1.058935e-19, Termination: CONVERGENCE
[       OK ] Process_Constraint_IMU.Var_P1_Q1_V1_B1_B2_Invar_P2_Q2_V2 (48 ms)
[ RUN      ] Process_Constraint_IMU.Var_P1_Q1_B1_V2_B2_Invar_V1_P2_Q2
[trace][11:15:43] [gtest_IMU.cpp L638 : TestBody] Ceres Solver Report: Iterations: 17, Initial cost: 4.769603e+03, Final cost: 3.762091e-19, Termination: CONVERGENCE
[       OK ] Process_Constraint_IMU.Var_P1_Q1_B1_V2_B2_Invar_V1_P2_Q2 (47 ms)
[----------] 3 tests from Process_Constraint_IMU (133 ms total)

[----------] 2 tests from Process_Constraint_IMU_ODO
[ RUN      ] Process_Constraint_IMU_ODO.Var_P0_Q0_V0_B0_P1_Q1_B1__Invar_V1
[trace][11:15:43] [gtest_IMU.cpp L711 : TestBody] Ceres Solver Report: Iterations: 19, Initial cost: 6.842446e+03, Final cost: 1.855814e-22, Termination: CONVERGENCE
[       OK ] Process_Constraint_IMU_ODO.Var_P0_Q0_V0_B0_P1_Q1_B1__Invar_V1 (68 ms)
[ RUN      ] Process_Constraint_IMU_ODO.Var_P0_Q0_B0_P1_Q1_V1_B1__Invar_V0
[trace][11:15:43] [gtest_IMU.cpp L783 : TestBody] Ceres Solver Report: Iterations: 16, Initial cost: 1.363675e+04, Final cost: 1.880084e-20, Termination: CONVERGENCE
[       OK ] Process_Constraint_IMU_ODO.Var_P0_Q0_B0_P1_Q1_V1_B1__Invar_V0 (59 ms)
[----------] 2 tests from Process_Constraint_IMU_ODO (127 ms total)
*
*
*/
