
#ifndef CONSTRAINT_ODOM_2D_COMPLEX_ANGLE_H_
#define CONSTRAINT_ODOM_2D_COMPLEX_ANGLE_H_

//Wolf includes
#include "wolf.h"
#include "constraint_sparse.h"

class ConstraintOdom2DComplexAngle: public ConstraintSparse<2,2,2,2,2>
{
	public:
		static const unsigned int N_BLOCKS = 2;

		ConstraintOdom2DComplexAngle(FeatureBase* _ftr_ptr,  WolfScalar* _block0Ptr, WolfScalar* _block1Ptr, WolfScalar* _block2Ptr, WolfScalar* _block3Ptr) :
			ConstraintSparse<2,2,2,2,2>(_ftr_ptr,CTR_ODOM_2D_COMPLEX_ANGLE, _block0Ptr, _block1Ptr, _block2Ptr, _block3Ptr)
		{
			//
		}

		ConstraintOdom2DComplexAngle(FeatureBase* _ftr_ptr, StateBase* _state0Ptr, StateBase* _state1Ptr, StateBase* _state2Ptr, StateBase* _state3Ptr) :
			ConstraintSparse<2,2,2,2,2>(_ftr_ptr,CTR_ODOM_2D_COMPLEX_ANGLE,  _state0Ptr->getPtr(), _state1Ptr->getPtr(),_state2Ptr->getPtr(), _state3Ptr->getPtr())
		{
			//
		}

        virtual ~ConstraintOdom2DComplexAngle()
		{
			//
		}

		template <typename T>
		bool operator()(const T* const _p1, const T* const _o1, const T* const _p2, const T* const _o2, T* _residuals) const
		{
			// Expected measurement
			T expected_range = (_p2[0]-_p1[0])*(_p2[0]-_p1[0]) + (_p2[1]-_p1[1])*(_p2[1]-_p1[1]); //square of the range
			T expected_rotation = atan2(_o2[1]*_o1[0] - _o2[0]*_o1[1], _o1[0]*_o2[0] + _o1[1]*_o2[1]);

			// Residuals
			_residuals[0] = (expected_range - T(measurement_(0))*T(measurement_(0))) / T(measurement_covariance_(0,0));
			_residuals[1] = (expected_rotation - T(measurement_(1))) / T(measurement_covariance_(1,1));

			return true;
		}
};
#endif
