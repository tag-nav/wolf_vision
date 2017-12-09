
#ifndef CONSTRAINT_ABSOLUTE_ORIENTATION_H_
#define CONSTRAINT_ABSOLUTE_ORIENTATION_H_

//Wolf includes
#include "constraint_autodiff.h"
#include "frame_base.h"
#include "rotations.h"


namespace wolf {

WOLF_PTR_TYPEDEFS(ConstraintAbsO);

//class
class ConstraintAbsO: public ConstraintAutodiff<ConstraintAbsO,3,4>
{
    public:

        ConstraintAbsO(FeatureBasePtr _ftr_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintAutodiff<ConstraintAbsO,3,4>(CTR_ABS_O, nullptr, nullptr, nullptr, nullptr, nullptr, _apply_loss_function, _status, _ftr_ptr->getFramePtr()->getOPtr())
        {
            setType("ABS O");
        }

        virtual ~ConstraintAbsO() = default;

        template<typename T>
        bool operator ()(const T* const _o, T* _residuals) const;

        virtual JacobianMethod getJacobianMethod() const override
        {
            return JAC_AUTO;
        }

};

template<typename T>
inline bool ConstraintAbsO::operator ()(const T* const _o, T* _residuals) const
{

    // state
    Eigen::Quaternion<T>    q(_o);

    // measurements
    Eigen::Quaternions  q_measured(getMeasurement().data() + 0);

    // error
    Eigen::Matrix<T, 3, 1> er;
    er        = q2v(q.conjugate() * q_measured.cast<T>());

    // residual
    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(_residuals);
    res               = getFeaturePtr()->getMeasurementSquareRootInformationUpper().cast<T>() * er;

    return true;
}

} // namespace wolf

#endif
