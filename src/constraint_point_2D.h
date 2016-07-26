#ifndef CONSTRAINT_POINT_2D_THETA_H_
#define CONSTRAINT_POINT_2D_THETA_H_

//Wolf includes
#include "constraint_sparse.h"
#include "feature_polyline_2D.h"
#include "landmark_polyline_2D.h"

namespace wolf {

class ConstraintPoint2D: public ConstraintSparse<2,2,1,2>
{
    protected:
        StateBlock* point_state_ptr_;
        Eigen::VectorXs measurement_;                   ///<  the measurement vector
        Eigen::MatrixXs measurement_covariance_;        ///<  the measurement covariance matrix
        Eigen::MatrixXs measurement_sqrt_information_;        ///<  the squared root information matrix
	public:
		static const unsigned int N_BLOCKS = 3;

		ConstraintPoint2D(FeaturePolyline2D* _ftr_ptr, LandmarkPolyline2D* _lmk_ptr, unsigned int _ftr_point_id, int _lmk_point_id, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
			ConstraintSparse<2,2,1,2>(CTR_POINT_2D, _lmk_ptr, _apply_loss_function, _status, _ftr_ptr->getFramePtr()->getPPtr(), _ftr_ptr->getFramePtr()->getOPtr(), _lmk_ptr->getPointStateBlockPtr(_lmk_point_id)),
			point_state_ptr_(_lmk_ptr->getPointStateBlockPtr(_lmk_point_id)), measurement_(_ftr_ptr->getPoints().col(_ftr_point_id)), measurement_covariance_(_ftr_ptr->getPointsCov().middleCols(_ftr_point_id*2,2))
		{
			//std::cout << "Constriant point: feature " << _ftr_ptr->id() << " landmark " << _lmk_ptr->id() << "(point " << _lmk_point_id << ")" << std::endl;
			//std::cout << "landmark state block " << _lmk_ptr->getPointStateBlockPtr(_lmk_point_id)->getVector().transpose() << std::endl;
            setType("CORNER 2D");
            Eigen::LLT<Eigen::MatrixXs> lltOfA(measurement_covariance_); // compute the Cholesky decomposition of A
            Eigen::MatrixXs measurement_sqrt_covariance = lltOfA.matrixU();
            measurement_sqrt_information_ = measurement_sqrt_covariance.inverse().transpose(); // retrieve factor U  in the decomposition
		}

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~ConstraintPoint2D()
        {
            //std::cout << "deleting ConstraintPoint2D " << nodeId() << std::endl;
        }

        LandmarkPolyline2D* getLandmarkPtr()
		{
			return (LandmarkPolyline2D*) landmark_ptr_;
		}

        StateBlock* getLandmarkPointPtr()
        {
            return point_state_ptr_;
        }

		template <typename T>
        bool operator ()(const T* const _robotP, const T* const _robotO, const T* const _landmarkP, T* _residuals) const;

        /** \brief Returns the jacobians computation method
         *
         * Returns the jacobians computation method
         *
         **/
        virtual JacobianMethod getJacobianMethod() const
        {
            return JAC_AUTO;
        }

        /** \brief Returns a reference to the feature measurement
         **/
        virtual const Eigen::VectorXs& getMeasurement() const
        {
            return measurement_;
        }

        /** \brief Returns a reference to the feature measurement covariance
         **/
        virtual const Eigen::MatrixXs& getMeasurementCovariance() const
        {
            return measurement_covariance_;
        }

        /** \brief Returns a reference to the feature measurement square root information
         **/
        virtual const Eigen::MatrixXs& getMeasurementSquareRootInformation() const
        {
            return measurement_sqrt_information_;
        }
};

template<typename T>
inline bool ConstraintPoint2D::operator ()(const T* const _robotP, const T* const _robotO, const T* const _landmarkP, T* _residuals) const
{
	//std::cout << "ConstraintPointToLine2D::operator" << std::endl;
    // Mapping
    Eigen::Map<const Eigen::Matrix<T,2,1>> landmark_position_map(_landmarkP);
    Eigen::Map<const Eigen::Matrix<T,2,1>> robot_position_map(_robotP);
    Eigen::Map<Eigen::Matrix<T,2,1>> residuals_map(_residuals);

    // sensor transformation
    Eigen::Matrix<T,2,1> sensor_position = getCapturePtr()->getSensorPtr()->getPPtr()->getVector().head(2).cast<T>();
    Eigen::Matrix<T,2,2> inverse_R_sensor = Eigen::Rotation2D<T>(T(-getCapturePtr()->getSensorOPtr()->getVector()(0))).matrix();
    // robot transformation
    Eigen::Matrix<T,2,2> inverse_R_robot = Eigen::Rotation2D<T>(-_robotO[0]).matrix();

    // Expected measurement
    Eigen::Matrix<T,2,1> expected_measurement_position = inverse_R_sensor * (inverse_R_robot * (landmark_position_map - robot_position_map) - sensor_position);

    // Residuals
    residuals_map = getMeasurementSquareRootInformation().cast<T>() * (expected_measurement_position - getMeasurement().head<2>().cast<T>());

	//std::cout << "residuals_map" << residuals_map[0] << std::endl;
    return true;
}

} // namespace wolf

#endif
