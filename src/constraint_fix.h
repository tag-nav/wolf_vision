
#ifndef CONSTRAINT_FIX_H_
#define CONSTRAINT_FIX_H_

//Wolf includes
#include "constraint_autodiff.h"
#include "frame_base.h"
#include "rotations.h"

//#include "ceres/jet.h"


namespace wolf {
    
WOLF_PTR_TYPEDEFS(ConstraintFix);

//class
class ConstraintFix: public ConstraintAutodiff<ConstraintFix,3,2,1>
{
    public:
        ConstraintFix(FeatureBasePtr _ftr_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintAutodiff<ConstraintFix, 3, 2, 1>(CTR_FIX, nullptr, nullptr, nullptr, nullptr, nullptr,_apply_loss_function, _status, _ftr_ptr->getFramePtr()->getPPtr(), _ftr_ptr->getFramePtr()->getOPtr())
        {
            setType("FIX");
//            std::cout << "created ConstraintFix " << std::endl;
        }

        virtual ~ConstraintFix() = default;

        template<typename T>
        bool operator ()(const T* const _p, const T* const _o, T* _residuals) const;

        /** \brief Returns the jacobians computation method
         *
         * Returns the jacobians computation method
         *
         **/
        virtual JacobianMethod getJacobianMethod() const override
        {
            return JAC_AUTO;
        }

};

template<typename T>
inline bool ConstraintFix::operator ()(const T* const _p, const T* const _o, T* _residuals) const
{
    // measurement
    Eigen::Matrix<T,3,1> meas =  getMeasurement().cast<T>();

    // error
    Eigen::Matrix<T,3,1> er;
    er(0) = meas(0) - _p[0];
    er(1) = meas(1) - _p[1];
    er(2) = meas(2) - _o[0];
    while (er[2] > T(M_PI))
        er(2) = er(2) - T(2*M_PI);
    while (er(2) <= T(-M_PI))
        er(2) = er(2) + T(2*M_PI);

    // residual
    Eigen::Map<Eigen::Matrix<T,3,1>> res(_residuals);
    res = getFeaturePtr()->getMeasurementSquareRootInformationUpper().cast<T>() * er;

    ////////////////////////////////////////////////////////
    // print Jacobian. Uncomment this as you wish (remember to uncomment #include "ceres/jet.h" above):
//    using ceres::Jet;
//    Eigen::MatrixXs J(3,3);
//    J.row(0) = ((Jet<Scalar, 3>)(er(0))).v;
//    J.row(1) = ((Jet<Scalar, 3>)(er(1))).v;
//    J.row(2) = ((Jet<Scalar, 3>)(er(2))).v;
//    J.row(0) = ((Jet<Scalar, 3>)(res(0))).v;
//    J.row(1) = ((Jet<Scalar, 3>)(res(1))).v;
//    J.row(2) = ((Jet<Scalar, 3>)(res(2))).v;
//    if (sizeof(er(0)) != sizeof(double))
//    {
//        std::cout << "ConstraintFix::Jacobian(c" << id() << ") = \n " << J << std::endl;
//        std::cout << "ConstraintFix::Weighted Jacobian(c" << id() << ") = \n " << J << std::endl;
//        std::cout << "Sqrt Info(c" << id() << ") = \n " << getMeasurementSquareRootInformationTransposed() << std::endl;
//    }
    ////////////////////////////////////////////////////////

    return true;
}

} // namespace wolf

#endif
