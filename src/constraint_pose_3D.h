
#ifndef CONSTRAINT_POSE_3D_H_
#define CONSTRAINT_POSE_3D_H_

//Wolf includes
#include "constraint_autodiff.h"
#include "frame_base.h"
#include "rotations.h"


namespace wolf {

WOLF_PTR_TYPEDEFS(ConstraintPose3D);

//class
class ConstraintPose3D: public ConstraintAutodiff<ConstraintPose3D,6,3,4>
{
    public:

        ConstraintPose3D(FeatureBasePtr _ftr_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintAutodiff<ConstraintPose3D,6,3,4>(CTR_POSE_3D, nullptr, nullptr, nullptr, nullptr, nullptr, _apply_loss_function, _status, _ftr_ptr->getFramePtr()->getPPtr(), _ftr_ptr->getFramePtr()->getOPtr())
        {
            setType("FIX3D");
        }

        virtual ~ConstraintPose3D() = default;

        template<typename T>
        bool operator ()(const T* const _p, const T* const _o, T* _residuals) const;

        virtual JacobianMethod getJacobianMethod() const override
        {
            return JAC_AUTO;
        }

};

template<typename T>
inline bool ConstraintPose3D::operator ()(const T* const _p, const T* const _o, T* _residuals) const
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
    er.tail(3)        = q2v(q.conjugate() * q_measured.cast<T>());

    // residual
    Eigen::Map<Eigen::Matrix<T, 6, 1>> res(_residuals);
    res               = getFeaturePtr()->getMeasurementSquareRootInformationUpper().cast<T>() * er;

    return true;
}

} // namespace wolf

#endif
