#ifndef CONSTRAINT_ODOM_2D_ANALYTIC_H_
#define CONSTRAINT_ODOM_2D_ANALYTIC_H_

//Wolf includes
#include "base/constraint/constraint_relative_2D_analytic.h"
#include <Eigen/StdVector>

namespace wolf {
    
WOLF_PTR_TYPEDEFS(ConstraintOdom2DAnalytic);
    
//class
class ConstraintOdom2DAnalytic : public ConstraintRelative2DAnalytic
{
    public:
        ConstraintOdom2DAnalytic(const FeatureBasePtr& _ftr_ptr,
                                 const FrameBasePtr& _frame_ptr,
                                 const ProcessorBasePtr& _processor_ptr = nullptr,
                                 bool _apply_loss_function = false,
                                 ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintRelative2DAnalytic("ODOM_2D", _ftr_ptr,
                                         _frame_ptr, _processor_ptr, _apply_loss_function, _status)
        {
            //
        }

        virtual ~ConstraintOdom2DAnalytic() = default;

//        /** \brief Returns the constraint residual size
//         *
//         * Returns the constraint residual size
//         *
//         **/
//        virtual unsigned int getSize() const
//        {
//            return 3;
//        }
//
//        /** \brief Returns the residual evaluated in the states provided
//         *
//         * Returns the residual evaluated in the states provided in std::vector of mapped Eigen::VectorXs
//         *
//         **/
//        virtual Eigen::VectorXs evaluateResiduals(const std::vector<Eigen::Map<const Eigen::VectorXs>>& _st_vector) const
//        {
//            Eigen::VectorXs residual(3);
//            Eigen::VectorXs expected_measurement(3);
//
//            // Expected measurement
//            Eigen::Matrix2s R = Eigen::Rotation2D<Scalar>(-_st_vector[1](0)).matrix();
//            expected_measurement.head(2) = R * (_st_vector[2]-_st_vector[0]); // rotar menys l'angle de primer (-_o1)
//            expected_measurement(2) = _st_vector[3](0) - _st_vector[1](0);
//
//            // Residual
//            residual = expected_measurement - getMeasurement();
//            while (residual(2) > M_PI)
//                residual(2) = residual(2) - 2*M_PI;
//            while (residual(2) <= -M_PI)
//                residual(2) = residual(2) + 2*M_PI;
//            residual = getMeasurementSquareRootInformationUpper() * residual;
//
//            return residual;
//        }
//
//        /** \brief Returns the jacobians evaluated in the states provided
//         *
//         * Returns the jacobians evaluated in the states provided in std::vector of mapped Eigen::VectorXs.
//         * IMPORTANT: only fill the jacobians of the state blocks specified in _compute_jacobian.
//         *
//         * \param _st_vector is a vector containing the mapped eigen vectors of all state blocks involved in the constraint
//         * \param jacobians is an output vector of mapped eigen matrices that sould contain the jacobians w.r.t each state block
//         * \param _compute_jacobian is a vector that specifies whether the ith jacobian sould be computed or not
//         *
//         **/
//        virtual void evaluateJacobians(const std::vector<Eigen::Map<const Eigen::VectorXs>>& _st_vector, std::vector<Eigen::Map<Eigen::MatrixXs>>& jacobians, const std::vector<bool>& _compute_jacobian) const
//        {
//            jacobians[0] << -cos(_st_vector[1](0)), -sin(_st_vector[1](0)),
//                             sin(_st_vector[1](0)), -cos(_st_vector[1](0)),
//                             0,                     0;
//            jacobians[0] = getMeasurementSquareRootInformationUpper() * jacobians[0];
//
//            jacobians[1] << -(_st_vector[2](0) - _st_vector[0](0)) * sin(_st_vector[1](0)) + (_st_vector[2](1) - _st_vector[0](1)) * cos(_st_vector[1](0)),
//                            -(_st_vector[2](0) - _st_vector[0](0)) * cos(_st_vector[1](0)) - (_st_vector[2](1) - _st_vector[0](1)) * sin(_st_vector[1](0)),
//                            -1;
//            jacobians[1] = getMeasurementSquareRootInformationUpper() * jacobians[0];
//
//            jacobians[2] << cos(_st_vector[1](0)), sin(_st_vector[1](0)),
//                            -sin(_st_vector[1](0)),cos(_st_vector[1](0)),
//                            0,                     0;
//            jacobians[2] = getMeasurementSquareRootInformationUpper() * jacobians[0];
//
//            jacobians[3] << 0, 0, 1;
//            jacobians[3] = getMeasurementSquareRootInformationUpper() * jacobians[0];
//        }

    public:
        static ConstraintBasePtr create(const FeatureBasePtr& _feature_ptr,
                                              const NodeBasePtr& _correspondant_ptr,
                                              const ProcessorBasePtr& _processor_ptr = nullptr)
        {
            return std::make_shared<ConstraintOdom2DAnalytic>(_feature_ptr, std::static_pointer_cast<FrameBase>(_correspondant_ptr), _processor_ptr);
        }

};

} // namespace wolf

#endif
