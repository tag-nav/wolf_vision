#ifndef CONSTRAINT_ODOM_2D_ANALYTIC_H_
#define CONSTRAINT_ODOM_2D_ANALYTIC_H_

//Wolf includes
#include "wolf.h"
#include "constraint_relative_2D_analytic.h"

namespace wolf {

class ConstraintOdom2DAnalytic : public ConstraintRelative2DAnalytic
{
    public:
        ConstraintOdom2DAnalytic(FeatureBase* _ftr_ptr, FrameBase* _frame_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintRelative2DAnalytic(_ftr_ptr, CTR_ODOM_2D, _frame_ptr, _apply_loss_function, _status)
        {
            setType("ODOM 2D ANALYTIC");
        }

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~ConstraintOdom2DAnalytic()
        {
            //
        }

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
//            residual = getMeasurementSquareRootInformation() * residual;
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
//            jacobians[0] = getMeasurementSquareRootInformation() * jacobians[0];
//
//            jacobians[1] << -(_st_vector[2](0) - _st_vector[0](0)) * sin(_st_vector[1](0)) + (_st_vector[2](1) - _st_vector[0](1)) * cos(_st_vector[1](0)),
//                            -(_st_vector[2](0) - _st_vector[0](0)) * cos(_st_vector[1](0)) - (_st_vector[2](1) - _st_vector[0](1)) * sin(_st_vector[1](0)),
//                            -1;
//            jacobians[1] = getMeasurementSquareRootInformation() * jacobians[0];
//
//            jacobians[2] << cos(_st_vector[1](0)), sin(_st_vector[1](0)),
//                            -sin(_st_vector[1](0)),cos(_st_vector[1](0)),
//                            0,                     0;
//            jacobians[2] = getMeasurementSquareRootInformation() * jacobians[0];
//
//            jacobians[3] << 0, 0, 1;
//            jacobians[3] = getMeasurementSquareRootInformation() * jacobians[0];
//        }
//
//        /** \brief Returns the jacobians computation method
//         *
//         * Returns the jacobians computation method
//         *
//         **/
//        virtual JacobianMethod getJacobianMethod() const
//        {
//            return AUTO;
//        }


    public:
        static wolf::ConstraintBase* create(FeatureBase* _feature_ptr, //
                                            NodeBase* _correspondant_ptr)
        {
            return new ConstraintOdom2DAnalytic(_feature_ptr, (FrameBase*)_correspondant_ptr);
        }

};

} // namespace wolf

#endif
