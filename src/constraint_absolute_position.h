
#ifndef CONSTRAINT_ABSOLUTE_POSITION_H_
#define CONSTRAINT_ABSOLUTE_POSITION_H_

//Wolf includes
#include "constraint_autodiff.h"
#include "frame_base.h"


namespace wolf {

WOLF_PTR_TYPEDEFS(ConstraintAbsP);

//class
class ConstraintAbsP: public ConstraintAutodiff<ConstraintAbsP,3,3>
{
    public:

        ConstraintAbsP(FeatureBasePtr _ftr_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintAutodiff<ConstraintAbsP,3,3>(CTR_ABS_P, nullptr, nullptr, nullptr, nullptr, nullptr, _apply_loss_function, _status, _ftr_ptr->getFramePtr()->getPPtr())
        {
            setType("FIX P");
        }

        virtual ~ConstraintAbsP() = default;

        template<typename T>
        bool operator ()(const T* const _p, T* _residuals) const;

        virtual JacobianMethod getJacobianMethod() const override
        {
            return JAC_AUTO;
        }

};

template<typename T>
inline bool ConstraintAbsP::operator ()(const T* const _p, T* _residuals) const
{

    // states
    Eigen::Matrix<T, 3, 1>  p(_p);

    // measurements
    Eigen::Vector3s     p_measured(getMeasurement().data() + 0);

    // error
    Eigen::Matrix<T, 3, 1> er;
    er       = p_measured.cast<T>() - p;

    // residual
    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(_residuals);
    res               = getFeaturePtr()->getMeasurementSquareRootInformationUpper().cast<T>() * er;

    return true;
}

} // namespace wolf

#endif
