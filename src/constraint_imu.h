#ifndef CONSTRAINT_IMU_THETA_H_
#define CONSTRAINT_IMU_THETA_H_

//Wolf includes
#include "constraint_sparse.h"
#include "feature_imu.h"
#include "frame_imu.h"
#include "rotations.h"

//Eigen include


namespace wolf {
    
WOLF_PTR_TYPEDEFS(ConstraintIMU);

//class
class ConstraintIMU : public ConstraintSparse<15, 3, 4, 3, 3, 3, 3, 4, 3>
{
    public:
        ConstraintIMU(FeatureIMUPtr _ftr_ptr, FrameIMUPtr _frame_ptr, bool _apply_loss_function = false,
                      ConstraintStatus _status = CTR_ACTIVE);

        virtual ~ConstraintIMU();

        /* \brief : compute the residual from the state blocks being iterated by the solver.
            -> computes the expected measurement
            -> compares the actual measurement with the expected one
            -> weights the result with the covariance of the noise (residual = sqrt_info_matrix * err;)
        */
        template<typename T>
        bool operator ()(const T* const _p1, const T* const _o1, const T* const _v1, const T* const _b_a1, const T* _b_g1,
                         const T* const _p2, const T* const _o2, const T* const _v2,
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
        bool getResiduals(const Eigen::MatrixBase<D1> & _p1, const Eigen::QuaternionBase<D2> & _q1, const Eigen::MatrixBase<D1> & _v1, const Eigen::MatrixBase<D1> & _ab, const Eigen::MatrixBase<D1> & _wb,
                        const Eigen::MatrixBase<D1> & _p2, const Eigen::QuaternionBase<D2> & _q2, const Eigen::MatrixBase<D1> & _v2, const Eigen::MatrixBase<D3> & _residuals) const;

        virtual JacobianMethod getJacobianMethod() const;

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
        template<typename D1, typename D2, typename D3>
        void expectation(const Eigen::MatrixBase<D1> & _p1, const Eigen::QuaternionBase<D2> & _q1, const Eigen::MatrixBase<D1> & _v1, const Eigen::MatrixBase<D1> & _ab, const Eigen::MatrixBase<D1> & _wb,
                        const Eigen::MatrixBase<D1> & _p2, const Eigen::QuaternionBase<D2> & _q2, const Eigen::MatrixBase<D1> & _v2, const Eigen::MatrixBase<D3> & _result) const;

        /* \brief : return the expected value given the state blocks in the wolf tree
            current frame data is taken from constraintIMU object.
            IMU frame is taken from wolf tree
        */
        Eigen::VectorXs expectation() const
        {
            Eigen::Matrix<wolf::Scalar, 10, 1> exp;
            FrameBasePtr frm_current    = getFeaturePtr()->getCapturePtr()->getFramePtr();
            FrameBasePtr frm_imu     = getFrameOtherPtr();
            
            //get information on current_frame in the constraintIMU
            const Eigen::Vector3s frame_current_pos  = dp_preint_;
            const Eigen::Quaternions frame_current_ori  = dq_preint_;
            const Eigen::Vector3s frame_current_vel  = dv_preint_;
            const Eigen::Vector3s frame_current_ab  = acc_bias_preint_;
            const Eigen::Vector3s frame_current_wb  = gyro_bias_preint_;
            const Eigen::Vector3s frame_imu_pos   = (frm_imu->getPPtr()->getVector());
            const Eigen::Vector4s frame_imu_ori   = (frm_imu->getOPtr()->getVector());
            const Eigen::Vector3s frame_imu_vel  = (frm_imu->getVPtr()->getVector());

            Eigen::Quaternions frame_imu_ori_q(frame_imu_ori);
            
            expectation(frame_current_pos, frame_current_ori, frame_current_vel, frame_current_ab, frame_current_wb, frame_imu_pos, frame_imu_ori_q, frame_imu_vel, exp);
            return exp;
        }

    public:
        static wolf::ConstraintBasePtr create(FeatureIMUPtr _feature_ptr, NodeBasePtr _correspondant_ptr);

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
        const wolf::Scalar dt_, dt_2_; ///< delta-time and delta-time-squared between keyframes
        const Eigen::Vector3s g_; ///< acceleration of gravity in World frame
};

inline ConstraintIMU::ConstraintIMU(FeatureIMUPtr _ftr_ptr, FrameIMUPtr _frame_ptr, bool _apply_loss_function,
                                    ConstraintStatus _status) :
        ConstraintSparse<15, 3, 4, 3, 3, 3, 3, 4, 3>(CTR_IMU, _frame_ptr, nullptr, nullptr, _apply_loss_function, _status,
                                                    _frame_ptr->getPPtr(), _frame_ptr->getOPtr(), _frame_ptr->getVPtr(),
                                                    _frame_ptr->getAccBiasPtr(), _frame_ptr->getGyroBiasPtr(),
                                                    _ftr_ptr->getFramePtr()->getPPtr(),
                                                    _ftr_ptr->getFramePtr()->getOPtr(),
                                                    _ftr_ptr->getFramePtr()->getVPtr()),
        dp_preint_(_ftr_ptr->dp_preint_), // dp, dv, dq at preintegration time
        dq_preint_(_ftr_ptr->dq_preint_),
        dv_preint_(_ftr_ptr->dv_preint_),
        acc_bias_preint_(_ftr_ptr->acc_bias_preint_), // state biases at preintegration time
        gyro_bias_preint_(_ftr_ptr->gyro_bias_preint_),
        dDp_dab_(_ftr_ptr->dDp_dab_), // Jacs of dp dv dq wrt biases
        dDv_dab_(_ftr_ptr->dDv_dab_),
        dDp_dwb_(_ftr_ptr->dDp_dwb_),
        dDv_dwb_(_ftr_ptr->dDv_dwb_),
        dDq_dwb_(_ftr_ptr->dDq_dwb_),
        dt_(_ftr_ptr->getFramePtr()->getTimeStamp() - _frame_ptr->getTimeStamp()),
        dt_2_(dt_*dt_),
        g_(wolf::gravity())

{
    setType("IMU");
}

inline ConstraintIMU::~ConstraintIMU()
{
    //
}

template<typename T>
inline bool ConstraintIMU::operator ()(const T* const _p1, const T* const _q1, const T* const _v1, const T* const _ab, const T* _wb,
                                       const T* const _p2, const T* const _q2, const T* const _v2,
                                       T* _residuals) const
{
    T a_stdev = (T)0.001; //for standard deviation
    T w_stdev = (T)0.001;
    //const Eigen::Matrix<T,3,3> A_r(Eigen::Matrix<T,3,3>::Identity()*a_stdev);
    //const Eigen::Matrix<T,3,3> W_r(Eigen::Matrix<T,3,3>::Identity()*w_stdev);
    const Eigen::Matrix<T,3,3> sqr_A_r(Eigen::Matrix<T,3,3>::Identity() * a_stdev * a_stdev);
    const Eigen::Matrix<T,3,3> sqr_W_r(Eigen::Matrix<T,3,3>::Identity() * w_stdev * w_stdev);

    // MAPS
    Eigen::Map<const Eigen::Matrix<T,3,1> > p1(_p1);
    const Eigen::Quaternion<T> q1(_q1);
    //Eigen::Map<const Eigen::Quaternion<T> > q1(_q1);
    Eigen::Map<const Eigen::Matrix<T,3,1> > v1(_v1);
    Eigen::Map<const Eigen::Matrix<T,3,1> > ab(_ab);
    Eigen::Map<const Eigen::Matrix<T,3,1> > wb(_wb);

    Eigen::Map<const Eigen::Matrix<T,3,1> > p2(_p2);
    Eigen::Map<const Eigen::Quaternion<T> > q2(_q2);
    Eigen::Map<const Eigen::Matrix<T,3,1> > v2(_v2);

    Eigen::Map<Eigen::Matrix<T,15,1> > residuals(_residuals);


    // Predict delta: d_pred = x2 (-) x1
    Eigen::Matrix<T,3,1> dp_predict = q1.conjugate() * ( p2 - p1 - v1 * (T)dt_ - (T)0.5 * g_.cast<T>() * (T)dt_2_ );
    Eigen::Matrix<T,3,1> dv_predict = q1.conjugate() * ( v2 - v1 - g_.cast<T>() * (T)dt_ );
    Eigen::Quaternion<T> dq_predict = q1.conjugate() * q2;
    Eigen::Matrix<T,3,1> ab_error(acc_bias_preint_.cast<T>() - ab); //bias used for preintegration - bias in KeyFrame
    Eigen::Matrix<T,3,1> wb_error(gyro_bias_preint_.cast<T>() - wb);

    // Correct measured delta: delta_corr = delta + J_bias * (bias - bias_measured)
    Eigen::Matrix<T,3,1> dp_correct = dp_preint_.cast<T>() + dDp_dab_.cast<T>() * (ab - acc_bias_preint_.cast<T>()) + dDp_dwb_.cast<T>() * (wb - gyro_bias_preint_.cast<T>());
    Eigen::Matrix<T,3,1> dv_correct = dv_preint_.cast<T>() + dDv_dab_.cast<T>() * (ab - acc_bias_preint_.cast<T>()) + dDv_dwb_.cast<T>() * (wb - gyro_bias_preint_.cast<T>());
    Eigen::Matrix<T,3,1> do_step    = dDq_dwb_  .cast<T>() * (wb - gyro_bias_preint_.cast<T>());
    Eigen::Quaternion<T> dq_correct = dq_preint_.cast<T>() * v2q(do_step);

    // Delta error in minimal form: d_min = log(delta_pred (-) delta_corr)
    // Note the Dt here is zero because it's the delta-time between the same time stamps!
    Eigen::Matrix<T,3,1> dp_error   = dp_predict - dp_correct;
    Eigen::Matrix<T,3,1> dv_error   = dv_predict - dv_correct;
    Eigen::Matrix<T,3,1> do_error   = q2v(dq_correct.conjugate() * dq_predict); // In the name, 'o' of orientation, not 'q'

    // Assign to residuals vector
    residuals.head(3)       = dp_error;
    residuals.segment(3,3)  = do_error;
    residuals.segment(6,3)  = dv_error;
    residuals.segment(9,3)  = sqr_A_r.inverse() * ab_error; //const diagonal matrix. could be more efficient to store the inverse somewhere 
    residuals.tail(3)       = sqr_W_r.inverse() * wb_error; //instead of calculating it everytime

    return true;
}

template<typename D1, typename D2, typename D3>
inline bool ConstraintIMU::getResiduals(const Eigen::MatrixBase<D1> & _p1, const Eigen::QuaternionBase<D2> & _q1, const Eigen::MatrixBase<D1> & _v1, const Eigen::MatrixBase<D1> & _ab, const Eigen::MatrixBase<D1> & _wb,
                        const Eigen::MatrixBase<D1> & _p2, const Eigen::QuaternionBase<D2> & _q2, const Eigen::MatrixBase<D1> & _v2, const Eigen::MatrixBase<D3> & _residuals) const
{
    //needed typedefs
    typedef typename D2::Scalar DataType;

    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D3, 9)

    const Eigen::Matrix<DataType,3,1> ab(_ab);
    const Eigen::Matrix<DataType,3,1> wb(_wb);
    Eigen::Matrix<DataType,10,1> expected;
    this->expectation(_p1, _q1, _v1, _ab, _wb, _p2, _q2, _v2, expected);

    // Correct measured delta: delta_corr = delta + J_bias * (bias - bias_measured)
    Eigen::Matrix<DataType,3,1> dp_correct = dp_preint_.cast<DataType>() + dDp_dab_.cast<DataType>() * (ab - acc_bias_preint_.cast<DataType>()) + dDp_dwb_.cast<DataType>() * (wb - gyro_bias_preint_.cast<DataType>());
    Eigen::Matrix<DataType,3,1> dv_correct = dv_preint_.cast<DataType>() + dDv_dab_.cast<DataType>() * (ab - acc_bias_preint_.cast<DataType>()) + dDv_dwb_.cast<DataType>() * (wb - gyro_bias_preint_.cast<DataType>());
    Eigen::Matrix<DataType,3,1> do_step    = dDq_dwb_  .cast<DataType>() * (wb - gyro_bias_preint_.cast<DataType>());
    Eigen::Quaternion<DataType> dq_correct = dq_preint_.cast<DataType>() * v2q(do_step);

    // Delta error in minimal form: d_min = log(delta_pred (-) delta_corr)
    // Note the Dt here is zero because it's the delta-time between the same time stamps!
    Eigen::Quaternion<DataType> dq_predict((expected.segment(3,4)).data());
    Eigen::Matrix<DataType,3,1> dp_error   = expected.head(3) - dp_correct;
    Eigen::Matrix<DataType,3,1> do_error   = q2v(dq_correct.conjugate() * dq_predict); // In the name, 'o' of orientation, not 'q'
    Eigen::Matrix<DataType,3,1> dv_error   = expected.tail(3) - dv_correct;

    // Assign to residuals vector
    const_cast< Eigen::MatrixBase<D3>& > (_residuals).head(3) = dp_error;
    const_cast< Eigen::MatrixBase<D3>& > (_residuals).segment(3,3) = do_error;
    const_cast< Eigen::MatrixBase<D3>& > (_residuals).tail(3) = dv_error;

    return true;
}

template<typename D1, typename D2, typename D3>
inline void ConstraintIMU::expectation(const Eigen::MatrixBase<D1> & _p1, const Eigen::QuaternionBase<D2> & _q1, const Eigen::MatrixBase<D1> & _v1, const Eigen::MatrixBase<D1> & _ab, const Eigen::MatrixBase<D1> & _wb,
                        const Eigen::MatrixBase<D1> & _p2, const Eigen::QuaternionBase<D2> & _q2, const Eigen::MatrixBase<D1> & _v2, const Eigen::MatrixBase<D3> & _result) const
{
    //needed typedefs
    typedef typename D2::Vector3 Vector3Map;
    typedef typename D2::Scalar DataType;
    typedef Eigen::Map <Eigen::Matrix<DataType,4,1> > ConstVector4Map;

    //instead of maps we use static_asserts from eigen to detect size at compile time
    //check entry sizes
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D1, 3)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D3, 10)
     
    // Predict delta: d_pred = x2 (-) x1
    Vector3Map dp_predict = (_q1.conjugate() * ( _p2 - _p1 - _v1 * (DataType)dt_ - (DataType)0.5 * g_.cast<DataType>() * (DataType)dt_2_ ));
    Vector3Map dv_predict (_q1.conjugate() * ( _v2 - _v1 - g_.cast<DataType>() * (DataType)dt_ ));
    Eigen::Quaternion<DataType> dq_predict (_q1.conjugate() * _q2);
    ConstVector4Map dq_vec4(dq_predict.coeffs().data());

    const_cast< Eigen::MatrixBase<D3>& > (_result).head(3) = dp_predict;
    const_cast< Eigen::MatrixBase<D3>& > (_result).segment(3,4) = dq_vec4;
    const_cast< Eigen::MatrixBase<D3>& > (_result).tail(3) = dv_predict;
}

inline JacobianMethod ConstraintIMU::getJacobianMethod() const
{
    return JAC_AUTO;
}

inline wolf::ConstraintBasePtr ConstraintIMU::create(FeatureIMUPtr _feature_ptr, NodeBasePtr _correspondant_ptr)
{
    return std::make_shared<ConstraintIMU>(_feature_ptr, std::static_pointer_cast<FrameIMU>(_correspondant_ptr));
}

} // namespace wolf

#endif
