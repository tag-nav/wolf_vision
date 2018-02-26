
#ifndef CONSTRAINT_FIX_BIAS_H_
#define CONSTRAINT_FIX_BIAS_H_

//Wolf includes
#include "constraint_autodiff.h"
#include "frame_base.h"
#include "rotations.h"
#include "capture_imu.h"
#include "feature_imu.h"

//#include "ceres/jet.h"


namespace wolf {
    
WOLF_PTR_TYPEDEFS(ConstraintFixBias);

//class
class ConstraintFixBias: public ConstraintAutodiff<ConstraintFixBias,6,3,3>
{
    public:
        ConstraintFixBias(FeatureBasePtr _ftr_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintAutodiff<ConstraintFixBias, 6, 3, 3>(CTR_FIX_BIAS, nullptr, nullptr, nullptr, nullptr, nullptr, _apply_loss_function, _status, std::static_pointer_cast<CaptureIMU>(_ftr_ptr->getCapturePtr())->getAccBiasPtr(),
                                          std::static_pointer_cast<CaptureIMU>(_ftr_ptr->getCapturePtr())->getGyroBiasPtr())
        {
            setType("FIX BIAS");
            // std::cout << "created ConstraintFixBias " << std::endl;
        }

        virtual ~ConstraintFixBias() = default;

        template<typename T>
        bool operator ()(const T* const _ab, const T* const _wb, T* _residuals) const;

};

template<typename T>
inline bool ConstraintFixBias::operator ()(const T* const _ab, const T* const _wb, T* _residuals) const
{
    // measurement
    Eigen::Matrix<T,6,1> meas =  getMeasurement().cast<T>();
    Eigen::Matrix<T,3,1> ab(_ab);
    Eigen::Matrix<T,3,1> wb(_wb);

    // error
    Eigen::Matrix<T,6,1> er;
    er.head(3) = meas.head(3) - ab;
    er.tail(3) = meas.tail(3) - wb;

    // residual
    Eigen::Map<Eigen::Matrix<T,6,1>> res(_residuals);
    res = getFeaturePtr()->getMeasurementSquareRootInformationUpper().cast<T>() * er;

    ////////////////////////////////////////////////////////
    // print Jacobian. Uncomment this as you wish (remember to uncomment #include "ceres/jet.h" above):
//    using ceres::Jet;
//    Eigen::MatrixXs J(6,6);
//    J.row(0) = ((Jet<Scalar, 3>)(er(0))).v;
//    J.row(1) = ((Jet<Scalar, 3>)(er(1))).v;
//    J.row(2) = ((Jet<Scalar, 3>)(er(2))).v;
//    J.row(3) = ((Jet<Scalar, 3>)(er(3))).v;
//    J.row(4) = ((Jet<Scalar, 3>)(er(4))).v;
//    J.row(5) = ((Jet<Scalar, 3>)(er(5))).v;

//    J.row(0) = ((Jet<Scalar, 3>)(res(0))).v;
//    J.row(1) = ((Jet<Scalar, 3>)(res(1))).v;
//    J.row(2) = ((Jet<Scalar, 3>)(res(2))).v;
//    J.row(3) = ((Jet<Scalar, 3>)(res(3))).v;
//    J.row(4) = ((Jet<Scalar, 3>)(res(4))).v;
//    J.row(5) = ((Jet<Scalar, 3>)(res(5))).v;

//    if (sizeof(er(0)) != sizeof(double))
//    {
//        std::cout << "ConstraintFixBias::Jacobian(c" << id() << ") = \n " << J << std::endl;
//        std::cout << "ConstraintFixBias::Weighted Jacobian(c" << id() << ") = \n " << J << std::endl;
//        std::cout << "Sqrt Info(c" << id() << ") = \n " << getMeasurementSquareRootInformationUpper() << std::endl;
//    }
    ////////////////////////////////////////////////////////

    return true;
}

} // namespace wolf

#endif
