
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
			std::cout << "measurement_ptr_: "<< (*measurement_ptr_).transpose() << std::endl;
			//TODO: Not compute every step InvRot
			// Mapping
			Eigen::Map<const Eigen::Matrix<T,2,1>> landmark_position(_landmarkP);
			Eigen::Map<const Eigen::Matrix<T,2,1>> robot_position(_robotP);
			std::cout << "robot pose: " << std::endl;
			for (int i=0; i < 2; i++)
			   std::cout << _robotP[i] << " ";
			std::cout << std::endl;

			Eigen::Matrix<T,2,2> InvRot;
			InvRot << cos(*_robotO), sin(*_robotO),
					 -sin(*_robotO), cos(*_robotO);

			// Expected measurement
			Eigen::Matrix<T,2,1> expected_landmark_relative_position = InvRot * (landmark_position - robot_position);
			T expected_landmark_relative_orientation = (*_landmarkO) - (*_robotO);
			std::cout << "expected_measurement: "<< std::endl;
			for (int i=0; i < 2; i++)
				std::cout << expected_landmark_relative_position.data()[i] << " ";
			std::cout << std::endl;
			// Residuals
			_residuals[0] = (expected_landmark_relative_position(0) - T((*measurement_ptr_)(0))) / T((*measurement_covariance_ptr_)(0,0));
			_residuals[1] = (expected_landmark_relative_position(1) - T((*measurement_ptr_)(1))) / T((*measurement_covariance_ptr_)(1,1));
			_residuals[2] = (expected_landmark_relative_orientation - T((*measurement_ptr_)(2))) / T((*measurement_covariance_ptr_)(2,2));

			std::cout << "_residuals: "<< std::endl;
			for (int i=0; i < 3; i++)
				std::cout << _residuals[i] << " ";
			std::cout << std::endl;

			return true;
		}
};
#endif
