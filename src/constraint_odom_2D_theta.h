
#ifndef CONSTRAINT_ODOM_2D_THETA_H_
#define CONSTRAINT_ODOM_2D_THETA_H_

//Wolf includes
#include "wolf.h"
#include "constraint_sparse.h"

class ConstraintOdom2DTheta: public ConstraintSparse<2,2,1,2,1>
{
	public:
		static const unsigned int N_BLOCKS = 2;

		ConstraintOdom2DTheta(FeatureBase*_ftr_ptr,  WolfScalar* _block0Ptr, WolfScalar* _block1Ptr, WolfScalar* _block2Ptr, WolfScalar* _block3Ptr) :
			ConstraintSparse<2,2,1,2,1>(_ftr_ptr,CTR_ODOM_2D_THETA, _block0Ptr, _block1Ptr, _block2Ptr, _block3Ptr)
		{
			//
		}

		ConstraintOdom2DTheta(FeatureBase* _ftr_ptr, StateBase* _state0Ptr, StateBase* _state1Ptr, StateBase* _state2Ptr, StateBase* _state3Ptr) :
			ConstraintSparse<2,2,1,2,1>(_ftr_ptr,CTR_ODOM_2D_THETA,  _state0Ptr->getPtr(), _state1Ptr->getPtr(),_state2Ptr->getPtr(), _state3Ptr->getPtr())
		{
			//
		}
        
		virtual ~ConstraintOdom2DTheta()
		{
			//
		}

		template <typename T>
		bool operator()(const T* const _p1, const T* const _o1, const T* const _p2, const T* const _o2, T* _residuals) const
		{
			// Expected measurement
			T expected_range = (_p2[0]-_p1[0])*(_p2[0]-_p1[0]) + (_p2[1]-_p1[1])*(_p2[1]-_p1[1]); //square of the range
			T expected_rotation = _o2[0]-_o1[0];

			// Residuals
			_residuals[0] = (expected_range - T((*measurement_ptr_)(0))*T((*measurement_ptr_)(0))) / T((*measurement_covariance_ptr_)(0,0));
			_residuals[1] = (expected_rotation - T((*measurement_ptr_)(1))) / T((*measurement_covariance_ptr_)(1,1));

			return true;
		}
};
#endif
