
#ifndef CONSTRAINT_ABSOLUTE_VELOCITY_H_
#define CONSTRAINT_ABSOLUTE_VELOCITY_H_

//Wolf includes
#include "constraint_autodiff.h"
#include "frame_base.h"


namespace wolf {

WOLF_PTR_TYPEDEFS(ConstraintAbsV);

//class
class ConstraintAbsV: public ConstraintAutodiff<ConstraintAbsV,6,3>
{
    public:

        ConstraintAbsV(FeatureBasePtr _ftr_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintAutodiff<ConstraintAbsV,3,3>(CTR_ABS_V, nullptr, nullptr, nullptr, nullptr, nullptr, _apply_loss_function, _status, _ftr_ptr->getFramePtr()->getVPtr())
        {
            setType("FIX V");
        }

        virtual ~ConstraintAbsV() = default;

        template<typename T>
        bool operator ()(const T* const _v, T* _residuals) const;

        virtual JacobianMethod getJacobianMethod() const override
        {
            return JAC_AUTO;
        }

};

template<typename T>
inline bool ConstraintAbsV::operator ()(const T* const _v, T* _residuals) const
{

    // states
    Eigen::Matrix<T, 3, 1>  v(_v);

    // measurements
    Eigen::Vector3s     v_measured(getMeasurement().data() + 0);

    // error
    Eigen::Matrix<T, 3, 1> er;
    er       = v_measured.cast<T>() - v;

    // residual
    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(_residuals);
    res               = getFeaturePtr()->getMeasurementSquareRootInformationUpper().cast<T>() * er;

    return true;
}

} // namespace wolf

#endif
