#ifndef FACTOR_POINT_TO_LINE_2D_H_
#define FACTOR_POINT_TO_LINE_2D_H_

//Wolf includes
#include "base/factor/factor_autodiff.h"
#include "base/feature/feature_polyline_2D.h"
#include "base/landmark/landmark_polyline_2D.h"

namespace wolf {
    
WOLF_PTR_TYPEDEFS(FactorPointToLine2D);
    
//class
class FactorPointToLine2D: public FactorAutodiff<FactorPointToLine2D, 1,2,1,2,1,2,2>
{
    protected:
		int landmark_point_id_;
		int landmark_point_aux_id_;
        unsigned int feature_point_id_;
        StateBlockPtr point_state_ptr_;
        StateBlockPtr point_aux_state_ptr_;
        Eigen::VectorXs measurement_;                   ///<  the measurement vector
        Eigen::MatrixXs measurement_covariance_;        ///<  the measurement covariance matrix
        Eigen::MatrixXs measurement_sqrt_information_;  ///<  the squared root information matrix

    public:

    FactorPointToLine2D(const FeaturePolyline2DPtr& _ftr_ptr,
                            const LandmarkPolyline2DPtr& _lmk_ptr,
                            const ProcessorBasePtr& _processor_ptr,
                            unsigned int _ftr_point_id, int _lmk_point_id,  int _lmk_point_aux_id,
                            bool _apply_loss_function = false, FactorStatus _status = FAC_ACTIVE) :
        FactorAutodiff<FactorPointToLine2D, 1,2,1,2,1,2,2>("POINT TO LINE 2D",
                nullptr, nullptr, nullptr, _lmk_ptr, _processor_ptr, _apply_loss_function, _status, _ftr_ptr->getFrame()->getP(), _ftr_ptr->getFrame()->getO(), _lmk_ptr->getP(), _lmk_ptr->getO(), _lmk_ptr->getPointStateBlock(_lmk_point_id), _lmk_ptr->getPointStateBlock(_lmk_point_aux_id)),
        landmark_point_id_(_lmk_point_id), landmark_point_aux_id_(_lmk_point_aux_id), feature_point_id_(_ftr_point_id), point_state_ptr_(_lmk_ptr->getPointStateBlock(_lmk_point_id)), point_aux_state_ptr_(_lmk_ptr->getPointStateBlock(_lmk_point_aux_id)), measurement_(_ftr_ptr->getPoints().col(_ftr_point_id)), measurement_covariance_(_ftr_ptr->getPointsCov().middleCols(_ftr_point_id*2,2))
    {
        //std::cout << "FactorPointToLine2D" << std::endl;
        Eigen::LLT<Eigen::MatrixXs> lltOfA(measurement_covariance_); // compute the Cholesky decomposition of A
        Eigen::MatrixXs measurement_sqrt_covariance = lltOfA.matrixU();
        measurement_sqrt_information_ = measurement_sqrt_covariance.inverse().transpose(); // retrieve factor U  in the decomposition
    }

    virtual ~FactorPointToLine2D() = default;

    LandmarkPolyline2DPtr getLandmark()
    {
      return std::static_pointer_cast<LandmarkPolyline2D>( landmark_other_ptr_.lock() );
    }

    int getLandmarkPointId()
    {
      return landmark_point_id_;
    }

    int getLandmarkPointAuxId()
    {
      return landmark_point_aux_id_;
    }

    unsigned int getFeaturePointId()
    {
      return feature_point_id_;
    }

    StateBlockPtr getLandmarkPoint()
    {
      return point_state_ptr_;
    }

    StateBlockPtr getLandmarkPointAux()
    {
      return point_state_ptr_;
    }

    template <typename T>
    bool operator ()(const T* const _robotP, const T* const _robotO, const T* const _landmarkOriginPosition, const T* const _landmarkOriginOrientation, const T* const _landmarkPoint, const T* const _landmarkPointAux, T* _residuals) const;

    /** \brief Returns a reference to the feature measurement
         **/
    virtual const Eigen::VectorXs& getMeasurement() const override
    {
      return measurement_;
    }

    /** \brief Returns a reference to the feature measurement covariance
         **/
    virtual const Eigen::MatrixXs& getMeasurementCovariance() const override
    {
      return measurement_covariance_;
    }

    /** \brief Returns a reference to the feature measurement square root information
         **/
    virtual const Eigen::MatrixXs& getMeasurementSquareRootInformationUpper() const override
    {
      return measurement_sqrt_information_;
    }
};

template<typename T>
inline bool FactorPointToLine2D::operator ()(const T* const _robotP, const T* const _robotO, const T* const _landmarkOriginPosition, const T* const _landmarkOriginOrientation, const T* const _landmarkPoint, const T* const _landmarkPointAux, T* _residuals) const
{
	//std::cout << "FactorPointToLine2D::operator" << std::endl;
    // Mapping
    Eigen::Map<const Eigen::Matrix<T,2,1>> landmark_origin_position_map(_landmarkOriginPosition);
    Eigen::Map<const Eigen::Matrix<T,2,1>> landmark_position_map(_landmarkPoint);
    Eigen::Map<const Eigen::Matrix<T,2,1>> landmark_aux_position_map(_landmarkPointAux);
    Eigen::Map<const Eigen::Matrix<T,2,1>> robot_position_map(_robotP);

    Eigen::Matrix<T,2,1> landmark_point = landmark_origin_position_map + Eigen::Rotation2D<T>(*_landmarkOriginOrientation) * landmark_position_map;
    Eigen::Matrix<T,2,1> landmark_point_aux = landmark_origin_position_map + Eigen::Rotation2D<T>(*_landmarkOriginOrientation) * landmark_aux_position_map;

    // sensor transformation
    Eigen::Matrix<T,2,1> sensor_position = getCapture()->getSensor()->getP()->getState().head(2).cast<T>();
    Eigen::Matrix<T,2,2> inverse_R_sensor = Eigen::Rotation2D<T>(T(-getCapture()->getSensorO()->getState()(0))).matrix();
    // robot transformation
    Eigen::Matrix<T,2,2> inverse_R_robot = Eigen::Rotation2D<T>(-_robotO[0]).matrix();

    // Expected measurement
    Eigen::Matrix<T,2,1> expected_P = inverse_R_sensor * (inverse_R_robot * (landmark_point - robot_position_map) - sensor_position);
    Eigen::Matrix<T,2,1> expected_Paux = inverse_R_sensor * (inverse_R_robot * (landmark_point_aux - robot_position_map) - sensor_position);

    // Case projection inside the segment P-Paux

    // Case projection ouside (P side)

    // Case projection ouside (Paux side)

    // normal vector
    Eigen::Matrix<T,2,1> normal(expected_Paux(1)-expected_P(1), expected_P(0)-expected_Paux(0));
    normal = normal / normal.norm();

    // Residual: distance = projection to the normal vector
    _residuals[0] = ((expected_P-measurement_.head<2>().cast<T>()).dot(normal));

    T projected_cov = normal.transpose() * measurement_covariance_.cast<T>() * normal;
    _residuals[0] = _residuals[0] / sqrt(projected_cov);

	//std::cout << "landmark points:" << std::endl;
	//std::cout << "A: " << expected_P(0) << " " << expected_P(1) << std::endl;
	//std::cout << "Aaux: " << expected_Paux(0) << " " << expected_Paux(1) << std::endl;
	//std::cout << "B: " << measurement_.transpose() << std::endl;
	//std::cout << "d: " << _residuals[0] << std::endl;
    return true;
}

} // namespace wolf

#endif
