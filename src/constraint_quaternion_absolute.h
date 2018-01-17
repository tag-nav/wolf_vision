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
#include "local_parametrization_quaternion.h"
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
    Eigen::Quaternion<T>  q1(_o);

    // measurements
    Eigen::Quaternions  q2(getMeasurement().data() + 0); //q_measured

    /* error
     * to compute the difference between both quaternions, we do 
     *      diff = log(q2 * q1.conj)
     * isolating q2 we get 
     *      q2 = exp(diff) * q1  ==> exp on the left means global.
     *
     * In rotations.h, we have
     *      minus(q1,q2) = log_q(q1.conjugate() * q2); --> this is a local 'minus'
     * But we could use this function to make a global 'minus' :
     *      minus(q2.conjugate(),q1.conjugate()) = log_q(q2 * q1.conjugate());
     * 
     * Other option : define plus_left(), plus_right(), minus_left() and minus_right(), 
     *                as we defined Jac_SO3_left() and Jac_SO3_right() in rotations.h.
     */ 

    Eigen::Matrix<T, 3, 1> er;
    er = log_q( q2.cast<T>() * q1.conjugate() );

    // residual
    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(_residuals);
    res               = getFeaturePtr()->getMeasurementSquareRootInformationUpper().cast<T>() * er;

    return true;
}

} // namespace wolf

#endif
