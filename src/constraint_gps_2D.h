
#ifndef CONSTRAINT_GPS_2D_H_
#define CONSTRAINT_GPS_2D_H_

//Wolf includes
#include "wolf.h"
#include "constraint_sparse.h"

class ConstraintGPS2D: public ConstraintSparse<2,2>
{
	public:
		static const unsigned int N_BLOCKS = 1;

		ConstraintGPS2D(const FeatureBasePtr& _ftr_ptr, WolfScalar* _statePtr) :
			ConstraintSparse<2,2>(_ftr_ptr,CTR_GPS_FIX_2D, _statePtr)
		{
			//
		}

		ConstraintGPS2D(const FeatureBasePtr& _ftr_ptr, const StateBaseShPtr& _statePtr):
			ConstraintSparse<2,2>(_ftr_ptr,CTR_GPS_FIX_2D, _statePtr->getPtr())
		{
			//
		}

//         ConstraintGPS2D(WolfScalar* _statePtr) :
//             ConstraintSparse<2,2>(CORR_GPS_FIX_2D, _statePtr)
//         {
//             //
//         }
// 
//         ConstraintGPS2D(const StateBaseShPtr& _statePtr):
//             ConstraintSparse<2,2>(CORR_GPS_FIX_2D, _statePtr->getPtr())
//         {
//             //
//         }
        
		virtual ~ConstraintGPS2D()
		{
			//
		}

		template <typename T>
		bool operator()(const T* const _x, T* _residuals) const
		{
			_residuals[0] = (T((*measurement_ptr_)(0))   - _x[0]) / T((*measurement_covariance_ptr_)(0,0));
			_residuals[1] = (T((*measurement_ptr_)(1)) - _x[1]) / T((*measurement_covariance_ptr_)(1,1));
			return true;
		}
};
#endif
