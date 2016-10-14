#ifndef CONSTRAINT_IMU_THETA_H_
#define CONSTRAINT_IMU_THETA_H_

//Wolf includes
#include "constraint_sparse.h"
#include "feature_imu.h"
#include "frame_imu.h"
#include "rotations.h"


namespace wolf {

class ConstraintIMU : public ConstraintSparse<9, 3, 4, 3, 3, 3, 3, 4, 3>
{
    public:
//        static const unsigned int N_BLOCKS = 8;

        ConstraintIMU(FeatureIMU* _ftr_ptr, FrameIMU* _frame_ptr, bool _apply_loss_function = false,
                      ConstraintStatus _status = CTR_ACTIVE);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         **/
        virtual ~ConstraintIMU();

        template<typename T>
        bool operator ()(const T* const _p1, const T* const _o1, const T* const _v1, const T* const _b_a1, const T* _b_g1,
                         const T* const _p2, const T* const _o2, const T* const _v2,
                         T* _residuals) const;

        virtual JacobianMethod getJacobianMethod() const;

    public:
        static wolf::ConstraintBasePtr create(FeatureIMU* _feature_ptr, NodeBase* _correspondant_ptr);

    private:
        /// Preintegrated delta
        Eigen::Vector3s dp_preint_;
        Eigen::Vector3s dv_preint_;
        Eigen::Quaternions dq_preint_;

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

inline ConstraintIMU::ConstraintIMU(FeatureIMU* _ftr_ptr, FrameIMU* _frame_ptr, bool _apply_loss_function,
                                    ConstraintStatus _status) :
        ConstraintSparse<9, 3, 4, 3, 3, 3, 3, 4, 3>(CTR_IMU, _frame_ptr, _apply_loss_function, _status,
                                                    _frame_ptr->getPPtr(), _frame_ptr->getOPtr(), _frame_ptr->getVPtr(),
                                                    _frame_ptr->getBAPtr(), _frame_ptr->getBGPtr(),
                                                    _ftr_ptr->getFramePtr()->getPPtr(),
                                                    _ftr_ptr->getFramePtr()->getOPtr(),
                                                    _ftr_ptr->getFramePtr()->getVPtr()),
        dp_preint_(_ftr_ptr->dp_preint_), // dp, dv, dq at preintegration time
        dv_preint_(_ftr_ptr->dv_preint_),
        dq_preint_(_ftr_ptr->dq_preint_),
        acc_bias_preint_(_ftr_ptr->acc_bias_preint_), // state biases at preintegration time
        gyro_bias_preint_(_ftr_ptr->gyro_bias_preint_),
        dDp_dab_(_ftr_ptr->dDp_dab_), // Jacs of dp dv dq wrt biases
        dDv_dab_(_ftr_ptr->dDv_dab_),
        dDp_dwb_(_ftr_ptr->dDp_dwb_),
        dDv_dwb_(_ftr_ptr->dDv_dwb_),
        dDq_dwb_(_ftr_ptr->dDq_dwb_),
        dt_(_frame_ptr->getTimeStamp() - getFeaturePtr()->getFramePtr()->getTimeStamp()),
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
    // MAPS
    Eigen::Map<const Eigen::Matrix<T,3,1> > p1(_p1);
    Eigen::Map<const Eigen::Quaternion<T> > q1(_q1);
    Eigen::Map<const Eigen::Matrix<T,3,1> > v1(_v1);
    Eigen::Map<const Eigen::Matrix<T,3,1> > ab(_ab);
    Eigen::Map<const Eigen::Matrix<T,3,1> > wb(_wb);

    Eigen::Map<const Eigen::Matrix<T,3,1> > p2(_p2);
    Eigen::Map<const Eigen::Quaternion<T> > q2(_q2);
    Eigen::Map<const Eigen::Matrix<T,3,1> > v2(_v2);

    Eigen::Map<Eigen::Matrix<T,10,1> > residuals(_residuals);


    // Predict delta: d_pred = x2 (-) x1
    Eigen::Matrix<T,3,1> dp_predict = q1.conjugate() * ( p2 - p1 - v1 * (T)dt_ - (T)0.5 * g_.cast<T>() * (T)dt_2_ );
    Eigen::Matrix<T,3,1> dv_predict = q1.conjugate() * ( v2 - v1 - g_.cast<T>() * (T)dt_ );
    Eigen::Quaternion<T> dq_predict = q1.conjugate() * q2;

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
    residuals.segment(3,3)  = dv_error;
    residuals.tail(3)       = do_error;

    return true;
}

inline JacobianMethod ConstraintIMU::getJacobianMethod() const
{
    return JAC_AUTO;
}

inline wolf::ConstraintBasePtr ConstraintIMU::create(FeatureIMU* _feature_ptr, NodeBase* _correspondant_ptr)
{
    return new ConstraintIMU(_feature_ptr, (FrameIMU*)(_correspondant_ptr));
}

} // namespace wolf

#endif
