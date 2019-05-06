/*
 * factor_odom_3D.h
 *
 *  Created on: Oct 7, 2016
 *      Author: jsola
 */

#ifndef FACTOR_ODOM_3D_H_
#define FACTOR_ODOM_3D_H_

#include "base/factor/factor_autodiff.h"
#include "base/math/rotations.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(FactorOdom3D);
    
//class
class FactorOdom3D : public FactorAutodiff<FactorOdom3D,6,3,4,3,4>
{
    public:
        FactorOdom3D(const FeatureBasePtr& _ftr_current_ptr,
                         const FrameBasePtr& _frame_past_ptr,
                         const ProcessorBasePtr& _processor_ptr = nullptr,
                         bool _apply_loss_function = false,
                         FactorStatus _status = FAC_ACTIVE);

        virtual ~FactorOdom3D() = default;

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
inline void FactorOdom3D::printRes(const Eigen::Matrix<T, 6, 1>& r) const
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
inline void FactorOdom3D::printRes (const  Eigen::Matrix<Scalar,6,1> & r) const
{
    std::cout << "Scalar residual = " << std::endl;
    std::cout << r.transpose() << std::endl;
}

inline FactorOdom3D::FactorOdom3D(const FeatureBasePtr& _ftr_current_ptr,
                                          const FrameBasePtr& _frame_past_ptr,
                                          const ProcessorBasePtr& _processor_ptr,
                                          bool _apply_loss_function,
                                          FactorStatus _status) :
        FactorAutodiff<FactorOdom3D, 6, 3, 4, 3, 4>("ODOM 3D",        // type
                                        _frame_past_ptr,    // frame other
                                        nullptr,            // capture other
                                        nullptr,            // feature other
                                        nullptr,            // landmark other
                                        _processor_ptr,     // processor
                                        _apply_loss_function,
                                        _status,
                                        _ftr_current_ptr->getFrame()->getP(), // current frame P
                                        _ftr_current_ptr->getFrame()->getO(), // current frame Q
                                        _frame_past_ptr->getP(), // past frame P
                                        _frame_past_ptr->getO()) // past frame Q
{
//    WOLF_TRACE("Constr ODOM 3D  (f", _ftr_current_ptr->id(),
//               " F", _ftr_current_ptr->getCapture()->getFrame()->id(),
//               ") (Fo", _frame_past_ptr->id(), ")");
//
//    WOLF_TRACE("delta preint: ", _ftr_current_ptr->getMeasurement().transpose());
//
//    WOLF_TRACE("Omega_delta.sqrt: \n", _ftr_current_ptr->getMeasurementSquareRootInformationUpper());
    //
}

template<typename T>
inline bool FactorOdom3D::expectation(const T* const _p_current, const T* const _q_current, const T* const _p_past,
                                                const T* const _q_past, T* _expectation_dp, T* _expectation_dq) const
{
    Eigen::Map<const Eigen::Matrix<T,3,1> > p_current(_p_current);
    Eigen::Map<const Eigen::Quaternion<T> > q_current(_q_current);
    Eigen::Map<const Eigen::Matrix<T,3,1> > p_past(_p_past);
    Eigen::Map<const Eigen::Quaternion<T> > q_past(_q_past);
    Eigen::Map<Eigen::Matrix<T,3,1> > expectation_dp(_expectation_dp);
    Eigen::Map<Eigen::Quaternion<T> > expectation_dq(_expectation_dq);

//     std::cout << "p_current: " << p_current(0) << " "
//                                << p_current(1) << " "
//                                << p_current(2) << "\n";
//     std::cout << "q_current: " << q_current.coeffs()(0) << " "
//                                << q_current.coeffs()(1) << " "
//                                << q_current.coeffs()(2) << " "
//                                << q_current.coeffs()(3) << "\n";
//     std::cout << "p_past: " << p_past(0) << " "
//                             << p_past(1) << " "
//                             << p_past(2) << "\n";
//     std::cout << "q_past: " << q_past.coeffs()(0) << " "
//                             << q_past.coeffs()(1) << " "
//                             << q_past.coeffs()(2) << " "
//                             << q_past.coeffs()(3) << "\n";

    // estimate motion increment, dp, dq;
    expectation_dp = q_past.conjugate() * (p_current - p_past);
    expectation_dq = q_past.conjugate() * q_current;

//    std::cout << "exp_p: " << expectation_dp(0) << " "
//                           << expectation_dp(1) << " "
//                           << expectation_dp(2) << "\n";
//    std::cout << "exp_q: " << expectation_dq.coeffs()(0) << " "
//                           << expectation_dq.coeffs()(1) << " "
//                           << expectation_dq.coeffs()(2) << " "
//                           << expectation_dq.coeffs()(3) << "\n";

    return true;
}

inline Eigen::VectorXs FactorOdom3D::expectation() const
{
    Eigen::VectorXs exp(7);
    FrameBasePtr frm_current = getFeature()->getFrame();
    FrameBasePtr frm_past = getFrameOther();
    const Eigen::VectorXs frame_current_pos  = frm_current->getP()->getState();
    const Eigen::VectorXs frame_current_ori  = frm_current->getO()->getState();
    const Eigen::VectorXs frame_past_pos     = frm_past->getP()->getState();
    const Eigen::VectorXs frame_past_ori     = frm_past->getO()->getState();

//    std::cout << "frame_current_pos: " << frm_current->getP()->getState().transpose() << std::endl;
//    std::cout << "frame_past_pos: " << frm_past->getP()->getState().transpose() << std::endl;

    expectation(frame_current_pos.data(),
                frame_current_ori.data(),
                frame_past_pos.data(),
                frame_past_ori.data(),
                exp.data(),
                exp.data()+3);
    return exp;
}

template<typename T>
inline bool FactorOdom3D::operator ()(const T* const _p_current, const T* const _q_current, const T* const _p_past,
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

    residuals.head(3) = dp_m - dp; // being a residual, rotating it has no implications, so we skip the product by dq.conj
    residuals.tail(3) = q2v(dq.conjugate() * dq_m);

    residuals = getMeasurementSquareRootInformationUpper().cast<T>() * residuals;

    return true;
}

} /* namespace wolf */

#endif /* FACTOR_ODOM_3D_H_ */
