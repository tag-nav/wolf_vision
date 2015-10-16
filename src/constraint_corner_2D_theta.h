#ifndef CONSTRAINT_CORNER_2D_THETA_H_
#define CONSTRAINT_CORNER_2D_THETA_H_

//Wolf includes
#include "wolf.h"
#include "constraint_sparse.h"
#include "landmark_corner_2D.h"

class ConstraintCorner2DTheta: public ConstraintSparse<3,2,1,2,1>
{
	protected:
		LandmarkCorner2D* lmk_ptr_;

	public:
		static const unsigned int N_BLOCKS = 4;

//		ConstraintCorner2DTheta(FeatureBase* _ftr_ptr, LandmarkCorner2D* _lmk_ptr,  WolfScalar* _robotPPtr, WolfScalar* _robotOPtr, WolfScalar* _landmarkPPtr, WolfScalar* _landmarkOPtr) :
//			ConstraintSparse<3,2,1,2,1>(_ftr_ptr,CTR_CORNER_2D_THETA, _robotPPtr, _robotOPtr, _landmarkPPtr, _landmarkOPtr),
//			lmk_ptr_(_lmk_ptr)
//		{
//			//
//		}

		ConstraintCorner2DTheta(FeatureBase* _ftr_ptr, LandmarkCorner2D* _lmk_ptr, StateBase* _robotPPtr, StateOrientation* _robotOPtr, StateBase* _landmarkPPtr, StateOrientation* _landmarkOPtr) :
			ConstraintSparse<3,2,1,2,1>(_ftr_ptr,CTR_CORNER_2D_THETA,  _robotPPtr, _robotOPtr,_landmarkPPtr, _landmarkOPtr),
			lmk_ptr_(_lmk_ptr)
		{
			lmk_ptr_->hit(this);
		}
        
		virtual ~ConstraintCorner2DTheta()
		{
			//std::cout << "deleting ConstraintCorner2DTheta " << nodeId() << std::endl;
			lmk_ptr_->unhit(this);
		}

		LandmarkCorner2D* getLandmarkPtr()
		{
			return lmk_ptr_;
		}

		template <typename T>
		bool operator()(const T* const _robotP, const T* const _robotO, const T* const _landmarkP, const T* const _landmarkO, T* _residuals) const
		{
			//TODO: Not computing the transformations each iteration
			// Mapping
			Eigen::Map<const Eigen::Matrix<T,2,1>> landmark_position(_landmarkP);
			Eigen::Map<const Eigen::Matrix<T,2,1>> robot_position(_robotP);

//			std::cout << "getSensorPosition: " << std::endl;
//			std::cout << getCapturePtr()->getSensorPtr()->getSensorPosition()->head(2).transpose() << std::endl;
//			std::cout << "getSensorRotation: " << std::endl;
//			std::cout << getCapturePtr()->getSensorPtr()->getSensorRotation()->topLeftCorner<2,2>() << std::endl;
//			std::cout << "atan2: " << atan2(getCapturePtr()->getSensorPtr()->getSensorRotation()->transpose()(0,1),getCapturePtr()->getSensorPtr()->getSensorRotation()->transpose()(0,0)) << std::endl;

			// sensor transformation
			Eigen::Matrix<T,2,1> sensor_position = getCapturePtr()->getSensorPtr()->getPPtr()->getVector().head(2).cast<T>();
			Eigen::Matrix<T,2,2> inverse_R_sensor = (getCapturePtr()->getSensorPtr()->getOPtr()->getRotationMatrix().topLeftCorner<2,2>().transpose()).cast<T>();

			Eigen::Matrix<T,2,2> inverse_R_robot;
			inverse_R_robot << cos(*_robotO), sin(*_robotO),
					 	 	  -sin(*_robotO), cos(*_robotO);

			// Expected measurement
			Eigen::Matrix<T,2,1> expected_landmark_relative_position = inverse_R_sensor * (inverse_R_robot * (landmark_position - robot_position) - sensor_position);
			T expected_landmark_relative_orientation = (*_landmarkO) - (*_robotO) - atan2(inverse_R_sensor(0,1),inverse_R_sensor(0,0));

			while (expected_landmark_relative_orientation > T(M_PI))
				expected_landmark_relative_orientation = expected_landmark_relative_orientation - T(2*M_PI);
			while (expected_landmark_relative_orientation <= T(-M_PI))
				expected_landmark_relative_orientation = expected_landmark_relative_orientation + T(2*M_PI);

			// Residuals
			_residuals[0] = (expected_landmark_relative_position(0) - T(measurement_(0))) / T(sqrt(measurement_covariance_(0,0)));
			_residuals[1] = (expected_landmark_relative_position(1) - T(measurement_(1))) / T(sqrt(measurement_covariance_(1,1)));
			_residuals[2] = expected_landmark_relative_orientation - T(measurement_(2));

			while (_residuals[2] > T(M_PI))
                _residuals[2] = _residuals[2] - T(2*M_PI);
            while (_residuals[2] <= T(-M_PI))
                _residuals[2] = _residuals[2] + T(2*M_PI);

            _residuals[2] = _residuals[2] / T(sqrt(measurement_covariance_(2,2)));

//			std::cout << "\nCONSTRAINT: " << nodeId() << std::endl;
//			std::cout << "Feature: " << getFeaturePtr()->nodeId() << std::endl;
//			std::cout << "Landmark: " << lmk_ptr_->nodeId() << std::endl;
//			std::cout << "measurement:\n\t" << (*measurement_ptr_).transpose() << std::endl;
//
//			std::cout << "robot pose:";
//			for (int i=0; i < 2; i++)
//			   std::cout  << "\n\t" << _robotP[i];
//			std::cout  << "\n\t" << _robotO[0];
//			std::cout << std::endl;
//
//			std::cout << "landmark pose:";
//			for (int i=0; i < 2; i++)
//			   std::cout  << "\n\t" << _landmarkP[i];
//			std::cout  << "\n\t" << _landmarkO[0];
//			std::cout << std::endl;
//
//			std::cout << "expected_measurement: ";
//			for (int i=0; i < 2; i++)
//				std::cout << "\n\t" << expected_landmark_relative_position.data()[i];
//			std::cout << std::endl;
//
//			std::cout << "_residuals: "<< std::endl;
//			for (int i=0; i < 3; i++)
//				std::cout  << "\n\t" << _residuals[i] << " ";
//			std::cout << std::endl;

			return true;
		}
};
#endif
