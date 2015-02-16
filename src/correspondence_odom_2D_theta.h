
#ifndef CORRESPONDENCE_ODOM_2D_THETA_H_
#define CORRESPONDENCE_ODOM_2D_THETA_H_

//Wolf includes
#include "wolf.h"
#include "correspondence_sparse.h"

class CorrespondenceOdom2DTheta: public CorrespondenceSparse<2,2,1,2,1>
{
	public:
		static const unsigned int N_BLOCKS = 2;

		CorrespondenceOdom2DTheta(const FeatureBasePtr& _ftr_ptr,  WolfScalar* _block0Ptr, WolfScalar* _block1Ptr, WolfScalar* _block2Ptr, WolfScalar* _block3Ptr) :
			CorrespondenceSparse<2,2,1,2,1>(_ftr_ptr,CORR_ODOM_2D_THETA, _block0Ptr, _block1Ptr, _block2Ptr, _block3Ptr)
		{
			//
		}

		CorrespondenceOdom2DTheta(const FeatureBasePtr& _ftr_ptr, const StateBaseShPtr& _state0Ptr, const StateBaseShPtr& _state1Ptr, const StateBaseShPtr& _state2Ptr, const StateBaseShPtr& _state3Ptr) :
			CorrespondenceSparse<2,2,1,2,1>(_ftr_ptr,CORR_ODOM_2D_THETA,  _state0Ptr->getPtr(), _state1Ptr->getPtr(),_state2Ptr->getPtr(), _state3Ptr->getPtr())
		{
			//
		}

		virtual ~CorrespondenceOdom2DTheta()
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
