/*
 * constraint_odom_3D.h
 *
 *  Created on: Oct 7, 2016
 *      Author: jsola
 */

#ifndef CONSTRAINT_ODOM_3D_H_
#define CONSTRAINT_ODOM_3D_H_

#include "constraint_autodiff.h"
#include "rotations.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(ConstraintOdom3D);
    
//class
class ConstraintOdom3D : public ConstraintAutodiff<ConstraintOdom3D,6,3,4,3,4>
{
    public:
        ConstraintOdom3D(const FeatureBasePtr& _ftr_current_ptr,
                         const FrameBasePtr& _frame_past_ptr,
                         const ProcessorBasePtr& _processor_ptr = nullptr,
                         bool _apply_loss_function = false,
                         ConstraintStatus _status = CTR_ACTIVE);

        virtual ~ConstraintOdom3D() = default;

        JacobianMethod getJacobianMethod() const override {return JAC_AUTO;}

        template<typename T>
                bool operator ()(const T* const _p_current,
                                 const T* const _q_current,
                                 const T* const _p_past,
                                 const T* const _q_past,
                                 T* _residuals) const;

        template<typename T>
                bool expectation(const T* const _p_current,
                                 const T* const _q_current,
                                 const T* const _p_past,
                                 const T* const _q_past,
                                 T* _expectation_dp,
                                 T* _expectation_dq) const;

        Eigen::VectorXs expectation() const;

    private:
        template<typename T>
        void printRes(const Eigen::Matrix<T, 6, 1>& r) const;


};

template<typename T>
inline void ConstraintOdom3D::printRes(const Eigen::Matrix<T, 6, 1>& r) const
{
    std::cout << "Jet residual = " << std::endl;
    std::cout << r(0) << std::endl;
    std::cout << r(1) << std::endl;
    std::cout << r(2) << std::endl;
    std::cout << r(3) << std::endl;
    std::cout << r(4) << std::endl;
    std::cout << r(5) << std::endl;
}

template<>
inline void ConstraintOdom3D::printRes (const  Eigen::Matrix<Scalar,6,1> & r) const
{
    std::cout << "Scalar residual = " << std::endl;
    std::cout << r.transpose() << std::endl;
}


inline ConstraintOdom3D::ConstraintOdom3D(const FeatureBasePtr& _ftr_current_ptr,
                                          const FrameBasePtr& _frame_past_ptr,
                                          const ProcessorBasePtr& _processor_ptr,
                                          bool _apply_loss_function,
                                          ConstraintStatus _status) :
        ConstraintAutodiff<ConstraintOdom3D, 6, 3, 4, 3, 4>(CTR_ODOM_3D,        // type
                                        _frame_past_ptr,    // frame other
                                        nullptr,            // capture other
                                        nullptr,            // feature other
                                        nullptr,            // landmark other
                                        _processor_ptr,     // processor
                                        _apply_loss_function,
                                        _status,
                                        _ftr_current_ptr->getFramePtr()->getPPtr(), // current frame P
                                        _ftr_current_ptr->getFramePtr()->getOPtr(), // current frame Q
                                        _frame_past_ptr->getPPtr(), // past frame P
                                        _frame_past_ptr->getOPtr()) // past frame Q
{
    setType("ODOM 3D");
    //
}

template<typename T>
inline bool wolf::ConstraintOdom3D::expectation(const T* const _p_current, const T* const _q_current, const T* const _p_past,
                                                const T* const _q_past, T* _expectation_dp, T* _expectation_dq) const
{
    Eigen::Map<const Eigen::Matrix<T,3,1> > p_current(_p_current);
    Eigen::Map<const Eigen::Quaternion<T> > q_current(_q_current);
    Eigen::Map<const Eigen::Matrix<T,3,1> > p_past(_p_past);
    Eigen::Map<const Eigen::Quaternion<T> > q_past(_q_past);
    Eigen::Map<Eigen::Matrix<T,3,1> > expectation_dp(_expectation_dp);
    Eigen::Map<Eigen::Quaternion<T> > expectation_dq(_expectation_dq);


//     std::cout << "p_current: " << p_current(0) << std::endl << p_current(1) << std::endl << p_current(2) << std::endl;
//     std::cout << "q_current: " << q_current.x() << std::endl << q_current.y() << std::endl << q_current.z() << std::endl << q_current.w() << std::endl;
//     std::cout << "p_past: " << p_past(0) << std::endl << p_past(1) << std::endl << p_past(2) << std::endl;
//     std::cout << "q_past: " << q_past.x() << std::endl << q_past.y() << std::endl << q_past.z() << std::endl << q_past.w() << std::endl;

    // estimate motion increment, dp, dq;
    expectation_dp = q_past.conjugate() * (p_current - p_past);
    expectation_dq =  q_past.conjugate() * q_current;

//    std::cout << "exp_p: " << expectation_dp(0) << std::endl << expectation_dp(1) << std::endl << expectation_dp(2) << std::endl;
//    std::cout << "exp_q: " << expectation(3) << std::endl << expectation(4) << std::endl << expectation(5) << std::endl << expectation(6) << std::endl;

    return true;
}

inline Eigen::VectorXs wolf::ConstraintOdom3D::expectation() const
{
    Eigen::VectorXs exp(7);
    FrameBasePtr frm_current = getFeaturePtr()->getFramePtr();
    FrameBasePtr frm_past = getFrameOtherPtr();
    const Scalar * const frame_current_pos  = frm_current->getPPtr()->getState().data();
    const Scalar * const frame_current_ori  = frm_current->getOPtr()->getState().data();
    const Scalar * const frame_past_pos     = frm_past->getPPtr()->getState().data();
    const Scalar * const frame_past_ori     = frm_past->getOPtr()->getState().data();

//    std::cout << "frame_current_pos: " << frm_current->getPPtr()->getVector().transpose() << std::endl;
//    std::cout << "frame_past_pos: " << frm_past->getPPtr()->getVector().transpose() << std::endl;

    expectation(frame_current_pos,
                frame_current_ori,
                frame_past_pos,
                frame_past_ori,
                exp.data(),
                exp.data()+3);
    return exp;
}

template<typename T>
inline bool wolf::ConstraintOdom3D::operator ()(const T* const _p_current, const T* const _q_current, const T* const _p_past,
                                                const T* const _q_past, T* _residuals) const
{

    /* Residual expression
     * -------------------
     *
     * Given two states x_i, x_j, with
     *
     *   x_i = [p_i , q_i] // position and quaternion: PQ pose
     *
     * we define the (-) operator as
     *
     *   x_j (-) x_i = [q_i.conj * (p_j - p_i) , q_i.conj * q_j] // PQ pose increment
     *
     * we also define Log and Exp maps as
     *
     *   Log(x) = [p, Log(q)]   // PQ pose to vector pose
     *   Exp(v) = [p, Exp(o)]   // vector pose to PQ pose
     *
     * where
     *
     *   v = [p,o] is the vector representation of a [p,q] pose.
     *
     * Note: The Log(q) and Exp(o) maps are here implemented as q2v() and v2q() respectively.
     *
     * Finally the residual is developed as follows. Given a measurement m
     *
     *   m = [p_m, o_m] \in R^6
     *
     * then
     *
     *   r = log [ exp(m) (-) ( x_j (-) x_i ) ]
     *
     * In the code below, we use _i and _j indices as follows:
     *
     *   x_i = x_past
     *   x_j = x_current
     */

    Eigen::Map<Eigen::Matrix<T,6,1> > residuals(_residuals);

    Eigen::Matrix<T, Eigen::Dynamic, 1> expected(7) ;
    expectation(_p_current, _q_current, _p_past, _q_past, expected.data(), expected.data()+3);

    // measured motion increment, dp_m, dq_m
    Eigen::Matrix<T,3,1> dp_m = getMeasurement().head<3>().cast<T>();
    Eigen::Quaternion<T> dq_m(getMeasurement().tail<4>().cast<T>());

    Eigen::Matrix<T,3,1> dp = expected.head(3);
    Eigen::Quaternion<T> dq;
    dq.x() = expected(3);
    dq.y() = expected(4);
    dq.z() = expected(5);
    dq.w() = expected(6);

//    std::cout << "operator dp: " << dp(0) << std::endl << dp(1) << std::endl << dp(2) << std::endl;
//    std::cout << "operator dq: " << dq.x() << std::endl << dq.y() << std::endl << dq.z() << std::endl << dq.w() << std::endl;

    residuals.head(3) = dp_m - dp; // being a residual, rotating it has no implications, so we skip the product by dq.conj
    residuals.tail(3) = q2v(dq.conjugate() * dq_m);

    residuals = getMeasurementSquareRootInformationTransposed().cast<T>() * residuals;

    //Eigen::Matrix<T,6,1> r = residuals;

    //printRes(r);

    return true;
}

} /* namespace wolf */

#endif /* CONSTRAINT_ODOM_3D_H_ */
