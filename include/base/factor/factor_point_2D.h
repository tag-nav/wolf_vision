#ifndef CONSTRAINT_POINT_2D_THETA_H_
#define CONSTRAINT_POINT_2D_THETA_H_

//Wolf includes
#include "base/factor/factor_autodiff.h"
#include "base/feature/feature_polyline_2D.h"
#include "base/landmark/landmark_polyline_2D.h"

namespace wolf {
    
WOLF_PTR_TYPEDEFS(FactorPoint2D);
    
/**
 * @brief The FactorPoint2D class
 */
class FactorPoint2D: public FactorAutodiff<FactorPoint2D, 2,2,1,2,1,2>
{
    protected:
        unsigned int feature_point_id_;
        int landmark_point_id_;
        StateBlockPtr point_state_ptr_;
        Eigen::VectorXs measurement_;                   ///<  the measurement vector
        Eigen::MatrixXs measurement_covariance_;        ///<  the measurement covariance matrix
        Eigen::MatrixXs measurement_sqrt_information_;  ///<  the squared root information matrix

    public:

    FactorPoint2D(const FeaturePolyline2DPtr& _ftr_ptr,
                      const LandmarkPolyline2DPtr& _lmk_ptr,
                      const ProcessorBasePtr& _processor_ptr,
                      unsigned int _ftr_point_id, int _lmk_point_id, bool _apply_loss_function = false, FactorStatus _status = CTR_ACTIVE) :
        FactorAutodiff<FactorPoint2D,2,2,1,2,1,2>("POINT 2D",
                nullptr, nullptr, nullptr, _lmk_ptr, _processor_ptr, _apply_loss_function, _status, _ftr_ptr->getFrame()->getP(), _ftr_ptr->getFrame()->getO(), _lmk_ptr->getP(), _lmk_ptr->getO(), _lmk_ptr->getPointStateBlockPtr(_lmk_point_id)),
        feature_point_id_(_ftr_point_id), landmark_point_id_(_lmk_point_id), point_state_ptr_(_lmk_ptr->getPointStateBlockPtr(_lmk_point_id)), measurement_(_ftr_ptr->getPoints().col(_ftr_point_id)), measurement_covariance_(_ftr_ptr->getPointsCov().middleCols(_ftr_point_id*2,2))
    {
        //std::cout << "Constriant point: feature " << _ftr_ptr->id() << " landmark " << _lmk_ptr->id() << "(point " << _lmk_point_id << ")" << std::endl;
        //std::cout << "landmark state block " << _lmk_ptr->getPointStateBlockPtr(_lmk_point_id)->getVector().transpose() << std::endl;
        Eigen::LLT<Eigen::MatrixXs> lltOfA(measurement_covariance_); // compute the Cholesky decomposition of A
        Eigen::MatrixXs measurement_sqrt_covariance = lltOfA.matrixU();
        measurement_sqrt_information_ = measurement_sqrt_covariance.inverse().transpose(); // retrieve factor U  in the decomposition
    }

    virtual ~FactorPoint2D() = default;

    /**
     * @brief getLandmarkPtr
     * @return
     */
    LandmarkPolyline2DPtr getLandmark()
    {
        return std::static_pointer_cast<LandmarkPolyline2D>(landmark_other_ptr_.lock());
    }

    /**
     * @brief getLandmarkPointId
     * @return
     */
    int getLandmarkPointId()
    {
      return landmark_point_id_;
    }

    /**
     * @brief getFeaturePointId
     * @return
     */
    unsigned int getFeaturePointId()
    {
      return feature_point_id_;
    }

    /**
     * @brief getLandmarkPointPtr
     * @return
     */
    StateBlockPtr getLandmarkPoint()
    {
      return point_state_ptr_;
    }

    /**
     *
     */
    template <typename T>
    bool operator ()(const T* const _robotP, const T* const _robotO, const T* const _landmarkOriginP, const T* const _landmarkOriginO, const T* const _landmarkPoint, T* _residuals) const;

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
inline bool FactorPoint2D::operator ()(const T* const _robotP, const T* const _robotO, const T* const _landmarkOriginP, const T* const _landmarkOriginO, const T* const _landmarkPoint, T* _residuals) const
{
	//std::cout << "FactorPointToLine2D::operator" << std::endl;
    // Mapping
    Eigen::Map<const Eigen::Matrix<T,2,1>> landmark_origin_position_map(_landmarkOriginP);
    Eigen::Map<const Eigen::Matrix<T,2,1>> landmark_position_map(_landmarkPoint);
    Eigen::Map<const Eigen::Matrix<T,2,1>> robot_position_map(_robotP);
    Eigen::Map<Eigen::Matrix<T,2,1>> residuals_map(_residuals);

    // Landmark point global position
    Eigen::Matrix<T,2,1> landmark_point = landmark_origin_position_map + Eigen::Rotation2D<T>(*_landmarkOriginO) * landmark_position_map;

    // sensor transformation
    Eigen::Matrix<T,2,1> sensor_position = getCapture()->getSensor()->getP()->getState().head(2).cast<T>();
    Eigen::Matrix<T,2,2> inverse_R_sensor = Eigen::Rotation2D<T>(T(-getCapture()->getSensorO()->getState()(0))).matrix();
    // robot transformation
    Eigen::Matrix<T,2,2> inverse_R_robot = Eigen::Rotation2D<T>(-_robotO[0]).matrix();

    // Expected measurement
    Eigen::Matrix<T,2,1> expected_measurement_position = inverse_R_sensor * (inverse_R_robot * (landmark_point - robot_position_map) - sensor_position);

    // Residuals
    residuals_map = getMeasurementSquareRootInformationUpper().cast<T>() * (expected_measurement_position - getMeasurement().head<2>().cast<T>());

	//std::cout << "residuals_map" << residuals_map[0] << std::endl;
    return true;
}

} // namespace wolf

#endif
