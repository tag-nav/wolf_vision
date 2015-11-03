
#ifndef CONSTRAINT_FIX_H_
#define CONSTRAINT_FIX_H_

//Wolf includes
#include "wolf.h"
#include "constraint_sparse.h"

class ConstraintFix: public ConstraintSparse<3,2,1>
{
	public:
		static const unsigned int N_BLOCKS = 2;

		ConstraintFix(FeatureBase* _ftr_ptr, StateBase* _pPtr, StateBase* _oPtr):
			ConstraintSparse<3,2,1>(_ftr_ptr, CTR_FIX, _pPtr, _oPtr)
		{
			//
		}
        
		virtual ~ConstraintFix()
		{
			//
		}

		template <typename T>
		bool operator()(const T* const _p, const T* const _o, T* _residuals) const
		{
		    _residuals[0] = (T(measurement_(0)) - _p[0]) / T(sqrt(measurement_covariance_(0,0)));
            _residuals[1] = (T(measurement_(1)) - _p[1]) / T(sqrt(measurement_covariance_(1,1)));
            _residuals[2] = T(measurement_(2)) - _o[0];

//            std::cout << "+++++++  fix constraint +++++++" << std::endl;
//            std::cout << "orientation:   " << _o[0] << std::endl;
//            std::cout << "measurement:   " << T(measurement_(2)) << std::endl;
//            std::cout << "residual:      " << _residuals[2] << std::endl;
//            std::cout << "is > PI        " << bool(_residuals[2] > T(2*M_PI)) << std::endl;
//            std::cout << "is >= PI       " << bool(_residuals[2] <= T(-2*M_PI)) << std::endl;

            while (_residuals[2] > T(M_PI))
            	_residuals[2] = _residuals[2] - T(2*M_PI);
            while (_residuals[2] <= T(-M_PI))
            	_residuals[2] = _residuals[2] + T(2*M_PI);

//            std::cout << "residual:      " << _residuals[2] << std::endl << std::endl;
//			  _residuals[2] = _residuals[2] / T(sqrt(measurement_covariance_(2,2)));

			return true;
		}
};
#endif
