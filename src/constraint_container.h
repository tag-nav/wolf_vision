#ifndef CONSTRAINT_CONTAINER_H_
#define CONSTRAINT_CONTAINER_H_

//Wolf includes
#include "wolf.h"
#include "constraint_sparse.h"
#include "landmark_container.h"

class ConstraintContainer: public ConstraintSparse<3,2,1,2,1>
{
	protected:
		LandmarkContainer* lmk_ptr_;
		unsigned int corner_;

	public:
		static const unsigned int N_BLOCKS = 4;

		ConstraintContainer(FeatureBase* _ftr_ptr, LandmarkContainer* _lmk_ptr, const unsigned int _corner, StateBase* _robotPPtr, StateOrientation* _robotOPtr, StateBase* _landmarkPPtr, StateOrientation* _landmarkOPtr) :
			ConstraintSparse<3,2,1,2,1>(_ftr_ptr,CTR_CONTAINER,  _robotPPtr, _robotOPtr,_landmarkPPtr, _landmarkOPtr),
			lmk_ptr_(_lmk_ptr),
			corner_(_corner)
		{
            assert(_corner >= 0 && _corner <= 4 && "wrong corner id in constraint container constructor");
            lmk_ptr_->hit(this);
		}
        
		virtual ~ConstraintContainer()
		{
			//std::cout << "deleting ConstraintContainer " << nodeId() << std::endl;
			lmk_ptr_->unhit(this);
		}

		LandmarkContainer* getLandmarkPtr()
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
			Eigen::Matrix<T,2,2> R_landmark;
			R_landmark << cos(*_landmarkO),-sin(*_landmarkO),
                          sin(*_landmarkO), cos(*_landmarkO);

			Eigen::Matrix<T,2,1> corner_position = lmk_ptr_->getCorner(corner_).head(2).cast<T>();

			// Expected measurement
			Eigen::Matrix<T,2,1> expected_measurement_position = inverse_R_sensor * (inverse_R_robot * (landmark_position - robot_position + R_landmark * corner_position) - sensor_position);
			T expected_measurement_orientation = (*_landmarkO) - (*_robotO) - T(getCapturePtr()->getSensorPtr()->getOPtr()->getYaw()) + T(lmk_ptr_->getCorner(corner_)(2));

			// Residuals
			_residuals[0] = (expected_measurement_position(0) - T(measurement_(0))) / T(sqrt(measurement_covariance_(0,0)));
			_residuals[1] = (expected_measurement_position(1) - T(measurement_(1))) / T(sqrt(measurement_covariance_(1,1)));
			_residuals[2] = expected_measurement_orientation - T(measurement_(2));

			while (_residuals[2] > T(M_PI))
			    _residuals[2] = _residuals[2] - T(2*M_PI);
            while (_residuals[2] <= T(-M_PI))
                _residuals[2] = _residuals[2] + T(2*M_PI);

            _residuals[2] = _residuals[2] / T(sqrt(measurement_covariance_(2,2)));

            //std::cout << "\nCONSTRAINT: " << nodeId() << std::endl;
            //std::cout << "Feature: " << getFeaturePtr()->nodeId() << std::endl;
            //std::cout << "Landmark: " << lmk_ptr_->nodeId() << std::endl;
            //std::cout << "measurement:\n\t" << measurement_.transpose() << std::endl;
            //
            //std::cout << "robot pose:";
            //for (int i=0; i < 2; i++)
            //   std::cout  << "\n\t" << _robotP[i];
            //std::cout  << "\n\t" << _robotO[0];
            //std::cout << std::endl;
            //
            //std::cout << "landmark pose:";
            //for (int i=0; i < 2; i++)
            //   std::cout  << "\n\t" << _landmarkP[i];
            //std::cout  << "\n\t" << _landmarkO[0];
            //std::cout << std::endl;
            //
            //std::cout << "expected_measurement: ";
            //for (int i=0; i < 2; i++)
            //    std::cout << "\n\t" << expected_measurement_position.data()[i];
            //std::cout << "\n\t" << expected_measurement_orientation << std::endl;
            //
            //std::cout << "_residuals: "<< std::endl;
            //for (int i=0; i < 3; i++)
            //    std::cout  << "\n\t" << _residuals[i] << " ";
            //std::cout << std::endl;

			return true;
		}
};
#endif
