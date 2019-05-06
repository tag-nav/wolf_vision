
#ifndef FACTOR_POSE_3D_H_
#define FACTOR_POSE_3D_H_

//Wolf includes
#include "base/factor/factor_autodiff.h"
#include "base/frame/frame_base.h"
#include "base/math/rotations.h"

namespace wolf {

WOLF_PTR_TYPEDEFS(FactorPose3D);

//class
class FactorPose3D: public FactorAutodiff<FactorPose3D,6,3,4>
{
    public:

        FactorPose3D(FeatureBasePtr _ftr_ptr, bool _apply_loss_function = false, FactorStatus _status = FAC_ACTIVE) :
            FactorAutodiff<FactorPose3D,6,3,4>("POSE 3D",
                                               nullptr,
                                               nullptr,
                                               nullptr,
                                               nullptr,
                                               nullptr,
                                               _apply_loss_function,
                                               _status,
                                               _ftr_ptr->getFrame()->getP(),
                                               _ftr_ptr->getFrame()->getO())
        {
            //
        }

        virtual ~FactorPose3D() = default;

        template<typename T>
        bool operator ()(const T* const _p, const T* const _o, T* _residuals) const;

};

template<typename T>
inline bool FactorPose3D::operator ()(const T* const _p, const T* const _o, T* _residuals) const
{

    // states
    Eigen::Matrix<T, 3, 1>  p(_p);
    Eigen::Quaternion<T>    q(_o);

    // measurements
    Eigen::Vector3s     p_measured(getMeasurement().data() + 0);
    Eigen::Quaternions  q_measured(getMeasurement().data() + 3);

    // error
    Eigen::Matrix<T, 6, 1> er;
    er.head(3)        = p_measured.cast<T>() - p;

    //    er.tail(3)        = q2v(q.conjugate() * q_measured.cast<T>()); // Local Error
    er.tail(3)        = q2v(q_measured.cast<T>() * q.conjugate());       // Global Error

    // residual
    Eigen::Map<Eigen::Matrix<T, 6, 1>> res(_residuals);
    res               = getFeature()->getMeasurementSquareRootInformationUpper().cast<T>() * er;

    return true;
}

} // namespace wolf

#endif
