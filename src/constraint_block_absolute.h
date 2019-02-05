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
            ConstraintAutodiff<ConstraintBlockAbsolute,3,3>("BLOCK ABS",
                                                            nullptr, nullptr, nullptr, nullptr, nullptr, _apply_loss_function, _status, _sb_ptr)
        {
            //
        }

        virtual ~ConstraintBlockAbsolute() = default;

        template<typename T>
        bool operator ()(const T* const _sb, T* _residuals) const;

};

template<typename T>
inline bool ConstraintBlockAbsolute::operator ()(const T* const _sb, T* _residuals) const
{
    // Maps
    Eigen::Map<T, 3, 1> sb(_sb); // state
    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(_residuals); // residual

    // residual
    res = getFeaturePtr()->getMeasurementSquareRootInformationUpper().cast<T>() * (getMeasurement().cast<T>() - sb);

    return true;
}

} // namespace wolf

#endif
