/**
 * \file constraint_quaternion_absolute.h
 *
 *  Created on: Dec 15, 2017
 *      \author: AtDinesh
 */

#ifndef CONSTRAINT_QUATERNION_ABSOLUTE_H_
#define CONSTRAINT_QUATERNION_ABSOLUTE_H_

//Wolf includes
#include "constraint_autodiff.h"
#include "frame_base.h"
#include "rotations.h"


namespace wolf {

WOLF_PTR_TYPEDEFS(ConstraintQuaternionAbsolute);

//class
class ConstraintQuaternionAbsolute: public ConstraintAutodiff<ConstraintQuaternionAbsolute,3,4>
{
    public:

        ConstraintQuaternionAbsolute(StateBlockPtr _sb_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintAutodiff<ConstraintQuaternionAbsolute,3,4>(CTR_BLOCK_ABS, nullptr, nullptr, nullptr, nullptr, nullptr, _apply_loss_function, _status, _sb_ptr)
        {
            setType("FIX Q");
        }

        virtual ~ConstraintQuaternionAbsolute() = default;

        template<typename T>
        bool operator ()(const T* const _o, T* _residuals) const;

        virtual JacobianMethod getJacobianMethod() const override
        {
            return JAC_AUTO;
        }

};

template<typename T>
inline bool ConstraintQuaternionAbsolute::operator ()(const T* const _o, T* _residuals) const
{

    // states
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
