#ifndef CONSTRAINT_CORNER_2D_THETA_H_
#define CONSTRAINT_CORNER_2D_THETA_H_

//Wolf includes
#include "base/constraint/constraint_autodiff.h"
#include "base/landmark/landmark_corner_2D.h"

namespace wolf {

WOLF_PTR_TYPEDEFS(ConstraintCorner2D);
    
class ConstraintCorner2D: public ConstraintAutodiff<ConstraintCorner2D, 3,2,1,2,1>
{
	public:

    ConstraintCorner2D(const FeatureBasePtr _ftr_ptr,
                       const LandmarkCorner2DPtr _lmk_ptr,
                       const ProcessorBasePtr& _processor_ptr,
                       bool _apply_loss_function = false,
                       ConstraintStatus _status = CTR_ACTIVE) :
        ConstraintAutodiff<ConstraintCorner2D,3,2,1,2,1>("CORNER 2D",
                nullptr, nullptr, nullptr, _lmk_ptr, _processor_ptr, _apply_loss_function, _status, _ftr_ptr->getFramePtr()->getPPtr(),_ftr_ptr->getFramePtr()->getOPtr(), _lmk_ptr->getPPtr(), _lmk_ptr->getOPtr())
    {
      //
    }

    virtual ~ConstraintCorner2D() = default;

    LandmarkCorner2DPtr getLandmarkPtr()
    {
      return std::static_pointer_cast<LandmarkCorner2D>( landmark_other_ptr_.lock() );
    }

    template <typename T>
    bool operator ()(const T* const _robotP, const T* const _robotO, const T* const _landmarkP,
                     const T* const _landmarkO, T* _residuals) const;

    static ConstraintBasePtr create(const FeatureBasePtr& _feature_ptr, const NodeBasePtr& _correspondant_ptr, const ProcessorBasePtr& _processor_ptr = nullptr)
    {
      return std::make_shared<ConstraintCorner2D>(_feature_ptr, std::static_pointer_cast<LandmarkCorner2D>(_correspondant_ptr), _processor_ptr);
    }

};

template<typename T>
inline bool ConstraintCorner2D::operator ()(const T* const _robotP, const T* const _robotO, const T* const _landmarkP,
                                            const T* const _landmarkO, T* _residuals) const
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
    Eigen::Matrix<T, 2, 1> sensor_position = getCapturePtr()->getSensorPtr()->getPPtr()->getState().head(2).cast<T>();
    Eigen::Matrix<T,2,2> inverse_R_sensor = Eigen::Rotation2D<T>(T(-getCapturePtr()->getSensorOPtr()->getState()(0))).matrix();
    // robot transformation
    Eigen::Matrix<T,2,2> inverse_R_robot = Eigen::Rotation2D<T>(-_robotO[0]).matrix();

    // Expected measurement
    Eigen::Matrix<T, 2, 1> expected_measurement_position = inverse_R_sensor * (inverse_R_robot * (landmark_position_map - robot_position_map) - sensor_position);
    T expected_measurement_orientation = _landmarkO[0] - _robotO[0] - T(getCapturePtr()->getSensorPtr()->getOPtr()->getState()(0));

    // Error
    residuals_map.head(2) = expected_measurement_position - getMeasurement().head<2>().cast<T>();
    residuals_map(2) = expected_measurement_orientation - T(getMeasurement()(2));

    // pi 2 pi
    while (_residuals[2] > T(M_PI))
        _residuals[2] = _residuals[2] - T(2*M_PI);
    while (_residuals[2] <= T(-M_PI))
        _residuals[2] = _residuals[2] + T(2*M_PI);

    // Residuals
    residuals_map = getMeasurementSquareRootInformationUpper().topLeftCorner<3,3>().cast<T>() * residuals_map;

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
    //std::cout << "expected_measurement: ";
    //for (int i=0; i < 2; i++)
    //    std::cout << "\n\t" << expected_measurement_position.data()[i];
    //std::cout << std::endl;
    //
    //std::cout << "_residuals: "<< std::endl;
    //for (int i=0; i < 3; i++)
    //    std::cout  << "\n\t" << _residuals[i] << " ";
    //std::cout << std::endl;
    return true;
}

} // namespace wolf

#endif
