
#ifndef FACTOR_POSE_2D_H_
#define FACTOR_POSE_2D_H_

//Wolf includes
#include "base/factor/factor_autodiff.h"
#include "base/frame/frame_base.h"
#include "base/math/rotations.h"

//#include "ceres/jet.h"

namespace wolf {
    
WOLF_PTR_TYPEDEFS(FactorPose2D);

//class
class FactorPose2D: public FactorAutodiff<FactorPose2D,3,2,1>
{
    public:
        FactorPose2D(FeatureBasePtr _ftr_ptr, bool _apply_loss_function = false, FactorStatus _status = FAC_ACTIVE) :
                FactorAutodiff<FactorPose2D, 3, 2, 1>("POSE 2D", nullptr, nullptr, nullptr, nullptr, nullptr,_apply_loss_function, _status, _ftr_ptr->getFrame()->getP(), _ftr_ptr->getFrame()->getO())
        {
//            std::cout << "created FactorPose2D " << std::endl;
        }

        virtual ~FactorPose2D() = default;

        template<typename T>
        bool operator ()(const T* const _p, const T* const _o, T* _residuals) const;

};

template<typename T>
inline bool FactorPose2D::operator ()(const T* const _p, const T* const _o, T* _residuals) const
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
    res = getFeature()->getMeasurementSquareRootInformationUpper().cast<T>() * er;

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
//        std::cout << "FactorPose2D::Jacobian(c" << id() << ") = \n " << J << std::endl;
//        std::cout << "FactorPose2D::Weighted Jacobian(c" << id() << ") = \n " << J << std::endl;
//        std::cout << "Sqrt Info(c" << id() << ") = \n " << getMeasurementSquareRootInformationUpper() << std::endl;
//    }
    ////////////////////////////////////////////////////////

    return true;
}

} // namespace wolf

#endif
