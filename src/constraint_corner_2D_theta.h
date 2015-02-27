
#ifndef CONSTRAINT_CORNER_2D_THETA_H_
#define CONSTRAINT_CORNER_2D_THETA_H_

//Wolf includes
#include "wolf.h"
#include "constraint_sparse.h"

class ConstraintCorner2DTheta: public ConstraintSparse<3,2,1,2,1>
{
	protected:
		LandmarkCorner2DPtr lmk_ptr_;

	public:
		static const unsigned int N_BLOCKS = 4;

		ConstraintCorner2DTheta(const FeatureBasePtr& _ftr_ptr, const LandmarkCorner2DPtr& _lmk_ptr,  WolfScalar* _robotPPtr, WolfScalar* _robotOPtr, WolfScalar* _landmarkPPtr, WolfScalar* _landmarkOPtr) :
			ConstraintSparse<3,2,1,2,1>(_ftr_ptr,CTR_CORNER_2D_THETA, _robotPPtr, _robotOPtr, _landmarkPPtr, _landmarkOPtr),
			lmk_ptr_(_lmk_ptr)
		{
			//
		}

		ConstraintCorner2DTheta(const FeatureBasePtr& _ftr_ptr, const LandmarkCorner2DPtr& _lmk_ptr, const StateBaseShPtr& _robotPPtr, const StateBaseShPtr& _robotOPtr, const StateBaseShPtr& _landmarkPPtr, const StateBaseShPtr& _landmarkOPtr) :
			ConstraintSparse<3,2,1,2,1>(_ftr_ptr,CTR_CORNER_2D_THETA,  _robotPPtr->getPtr(), _robotOPtr->getPtr(),_landmarkPPtr->getPtr(), _landmarkOPtr->getPtr()),
			lmk_ptr_(_lmk_ptr)
		{
			//
		}
        
		virtual ~ConstraintCorner2DTheta()
		{
			//
		}

		template <typename T>
		bool operator()(const T* const _robotP, const T* const _robotO, const T* const _landmarkP, const T* const _landmarkO, T* _residuals) const
		{
			// Mapping
			Eigen::Map<const Eigen::Matrix<T,2,1>> relative_landmark_position(_landmarkP);
			Eigen::Map<const Eigen::Matrix<T,2,1>> robot_position(_robotP);

			Eigen::Matrix<T,2,2> InvRot;
			InvRot << cos(*_robotO), sin(*_robotO),
					 -sin(*_robotO), cos(*_robotO);

			// Expected measurement
			Eigen::Matrix<T,2,1> expected_relative_landmark_position = InvRot * (relative_landmark_position - robot_position);
			T expected_relative_landmark_orientation = (*_landmarkO) +(*_robotO);

			// Residuals
			_residuals[0] = (expected_relative_landmark_position(0) - T((*measurement_ptr_)(0))) / T((*measurement_covariance_ptr_)(0,0));
			_residuals[1] = (expected_relative_landmark_position(1) - T((*measurement_ptr_)(1))) / T((*measurement_covariance_ptr_)(1,1));
			_residuals[2] = (expected_relative_landmark_orientation - T((*measurement_ptr_)(2))) / T((*measurement_covariance_ptr_)(2,2));

			return true;
		}
};
#endif
