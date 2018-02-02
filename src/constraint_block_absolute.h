/**
 * \file constraint_block_absolute.h
 *
 *  Created on: Dec 15, 2017
 *      \author: AtDinesh
 */

#ifndef CONSTRAINT_BLOCK_ABSOLUTE_H_
#define CONSTRAINT_BLOCK_ABSOLUTE_H_

//Wolf includes
#include "constraint_autodiff.h"
#include "frame_base.h"


namespace wolf {

WOLF_PTR_TYPEDEFS(ConstraintBlockAbsolute);

//class
class ConstraintBlockAbsolute: public ConstraintAutodiff<ConstraintBlockAbsolute,3,3>
{
    public:

        ConstraintBlockAbsolute(StateBlockPtr _sb_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintAutodiff<ConstraintBlockAbsolute,3,3>(CTR_BLOCK_ABS, nullptr, nullptr, nullptr, nullptr, nullptr, _apply_loss_function, _status, _sb_ptr)
        {
            setType("FIX SB");
        }

        virtual ~ConstraintBlockAbsolute() = default;

        template<typename T>
        bool operator ()(const T* const _sb, T* _residuals) const;

        virtual JacobianMethod getJacobianMethod() const override
        {
            return JAC_AUTO;
        }

};

template<typename T>
inline bool ConstraintBlockAbsolute::operator ()(const T* const _sb, T* _residuals) const
{

    // states
    Eigen::Matrix<T, 3, 1>  sb(_sb);

    // measurements
    Eigen::Vector3s     measured_state(getMeasurement().data() + 0);

    // error
    Eigen::Matrix<T, 3, 1> er;
    er       = measured_state.cast<T>() - sb;

    // residual
    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(_residuals);
    res               = getFeaturePtr()->getMeasurementSquareRootInformationUpper().cast<T>() * er;

    return true;
}

} // namespace wolf

#endif
