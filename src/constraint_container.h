#ifndef CONSTRAINT_CONTAINER_H_
#define CONSTRAINT_CONTAINER_H_

//Wolf includes
#include "wolf.h"
#include "constraint_sparse.h"
#include "landmark_container.h"

namespace wolf {
    
WOLF_PTR_TYPEDEFS(ConstraintContainer);

class ConstraintContainer: public ConstraintSparse<3,2,1,2,1>
{
	protected:
		LandmarkContainerWPtr lmk_ptr_;
		unsigned int corner_;

	public:

      ConstraintContainer(const FeatureBasePtr& _ftr_ptr,
                          const LandmarkContainerPtr& _lmk_ptr,
                          const ProcessorBasePtr& _processor_ptr,
                          const unsigned int _corner,
                          bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
      ConstraintSparse<3,2,1,2,1>(CTR_CONTAINER, nullptr, nullptr, _lmk_ptr, _processor_ptr, _apply_loss_function, _status, _ftr_ptr->getFramePtr()->getPPtr(),_ftr_ptr->getFramePtr()->getOPtr(), _lmk_ptr->getPPtr(), _lmk_ptr->getOPtr()),
			lmk_ptr_(_lmk_ptr),
			corner_(_corner)
		{
            assert(/*_corner >= 0 &&*/ _corner <= 3 && "Wrong corner id in constraint container constructor");
            setType("CONTAINER");

            std::cout << "new constraint container: corner idx = " << corner_ << std::endl;
		}

    virtual ~ConstraintContainer() = default;

		LandmarkContainerPtr getLandmarkPtr()
		{
			return lmk_ptr_.lock();
		}

		template <typename T>
		bool operator()(const T* const _robotP, const T* const _robotO, const T* const _landmarkP, const T* const _landmarkO, T* _residuals) const
		{
			// Mapping
			Eigen::Map<const Eigen::Matrix<T,2,1>> landmark_position_map(_landmarkP);
			Eigen::Map<const Eigen::Matrix<T,2,1>> robot_position_map(_robotP);
			Eigen::Map<Eigen::Matrix<T,3,1>> residuals_map(_residuals);

            //std::cout << "getSensorPosition: " << std::endl;
            //std::cout << getCapturePtr()->getSensorPtr()->getSensorPosition()->head(2).transpose() << std::endl;
            //std::cout << "getSensorRotation: " << std::endl;
            //std::cout << getCapturePtr()->getSensorPtr()->getSensorRotation()->topLeftCorner<2,2>() << std::endl;
            //std::cout << "atan2: " << atan2(getCapturePtr()->getSensorPtr()->getSensorRotation()->transpose()(0,1),getCapturePtr()->getSensorPtr()->getSensorRotation()->transpose()(0,0)) << std::endl;

			// sensor transformation
            Eigen::Matrix<T,2,1> sensor_position = getCapturePtr()->getSensorPtr()->getPPtr()->getState().head(2).cast<T>();
            Eigen::Matrix<T,2,2> inverse_R_sensor = Eigen::Rotation2D<T>(T(-getCapturePtr()->getSensorOPtr()->getState()(0))).matrix();
            // robot information
            Eigen::Matrix<T,2,2> inverse_R_robot = Eigen::Rotation2D<T>(-_robotO[0]).matrix();
            Eigen::Matrix<T,2,2> R_landmark = Eigen::Rotation2D<T>(_landmarkO[0]).matrix();
            Eigen::Matrix<T,2,1> corner_position = lmk_ptr_.lock()->getCorner(corner_).head<2>().cast<T>();

            // Expected measurement
            Eigen::Matrix<T,2,1> expected_measurement_position = inverse_R_sensor * (inverse_R_robot * (landmark_position_map - robot_position_map + R_landmark * corner_position) - sensor_position);
            T expected_measurement_orientation = _landmarkO[0] - _robotO[0] - T(getCapturePtr()->getSensorPtr()->getOPtr()->getState()(0)) + T(lmk_ptr_.lock()->getCorner(corner_)(2));

            // Error
            residuals_map.head(2) = expected_measurement_position - getMeasurement().head<2>().cast<T>();
            residuals_map(2) = expected_measurement_orientation - T(getMeasurement()(2));

            // pi 2 pi
            while (_residuals[2] > T(M_PI))
                _residuals[2] = _residuals[2] - T(2*M_PI);
            while (_residuals[2] <= T(-M_PI))
                _residuals[2] = _residuals[2] + T(2*M_PI);

            // Residuals
            residuals_map = getMeasurementSquareRootInformationTransposed().cast<T>() * residuals_map;

            //std::cout << "\nCONSTRAINT: " << id() << std::endl;
            //std::cout << "Feature: " << getFeaturePtr()->id() << std::endl;
            //std::cout << "Landmark: " << lmk_ptr_->id() << std::endl;
            //std::cout << "measurement:\n\t" << getMeasurement().transpose() << std::endl;
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
            //std::cout << "landmark pose (w.r.t sensor):";
            //Eigen::Matrix<T,2,1> relative_landmark_position = inverse_R_sensor * (inverse_R_robot * (landmark_position - robot_position) - sensor_position);
            //for (int i=0; i < 2; i++)
            //   std::cout  << "\n\t" << relative_landmark_position.data()[i];
            //std::cout  << "\n\t" << _landmarkO[0] - _robotO[0] - T( *(getCapturePtr()->getSensorPtr()->getOPtr()->getPtr()) );
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

    /** \brief Returns the jacobians computation method
     *
     * Returns the jacobians computation method
     *
     **/
    virtual JacobianMethod getJacobianMethod() const override
    {
      return JAC_AUTO;
    }


  public:
    static ConstraintBasePtr create(const FeatureBasePtr& _feature_ptr,
                                    const NodeBasePtr& _correspondant_ptr,
                                    const ProcessorBasePtr& _processor_ptr = nullptr)
    {
      unsigned int corner = 0; // Hard-coded, but this class is nevertheless deprecated.

      return std::make_shared<ConstraintContainer>(_feature_ptr, std::static_pointer_cast<LandmarkContainer>(_correspondant_ptr), _processor_ptr, corner);
    }

};

} // namespace wolf

#endif
