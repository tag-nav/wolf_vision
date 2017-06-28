#ifndef CONSTRAINT_RELATIVE_2D_ANALYTIC_H_
#define CONSTRAINT_RELATIVE_2D_ANALYTIC_H_

//Wolf includes
#include "constraint_analytic.h"
#include "landmark_base.h"
#include <Eigen/StdVector>

namespace wolf {
    
WOLF_PTR_TYPEDEFS(ConstraintRelative2DAnalytic);
    
//class
class ConstraintRelative2DAnalytic : public ConstraintAnalytic
{
    public:

        /** \brief Constructor of category CTR_FRAME
         **/
        ConstraintRelative2DAnalytic(FeatureBasePtr _ftr_ptr, ConstraintType _tp, FrameBasePtr _frame_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintAnalytic(_tp, _frame_ptr, _apply_loss_function, _status, _frame_ptr->getPPtr(), _frame_ptr->getOPtr(), _ftr_ptr->getFramePtr()->getPPtr(), _ftr_ptr->getFramePtr()->getOPtr())
        {
            //
        }

        /** \brief Constructor of category CTR_FEATURE
         **/
        ConstraintRelative2DAnalytic(FeatureBasePtr _ftr_ptr, ConstraintType _tp, FeatureBasePtr _ftr_other_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintAnalytic(_tp, _ftr_other_ptr, _apply_loss_function, _status, _ftr_ptr->getFramePtr()->getPPtr(), _ftr_ptr->getFramePtr()->getOPtr(), _ftr_other_ptr->getFramePtr()->getPPtr(), _ftr_other_ptr->getFramePtr()->getOPtr() )
        {
            //
        }

        /** \brief Constructor of category CTR_LANDMARK
         **/
        ConstraintRelative2DAnalytic(FeatureBasePtr _ftr_ptr, ConstraintType _tp, LandmarkBasePtr _landmark_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintAnalytic(_tp, _landmark_ptr, _apply_loss_function, _status, _ftr_ptr->getFramePtr()->getPPtr(), _ftr_ptr->getFramePtr()->getOPtr(), _landmark_ptr->getPPtr(), _landmark_ptr->getOPtr())
        {
            //
        }

        virtual ~ConstraintRelative2DAnalytic()
        {
            //
        }

        /** \brief Returns the constraint residual size
         **/
        virtual unsigned int getSize() const
        {
            return 3;
        }

        /** \brief Returns the residual evaluated in the states provided
         *
         * Returns the residual evaluated in the states provided in a std::vector of mapped Eigen::VectorXs
         **/
        virtual Eigen::VectorXs evaluateResiduals(
                const std::vector<Eigen::Map<const Eigen::VectorXs> >& _st_vector) const;

        /** \brief Returns the jacobians evaluated in the states provided
         *
         * Returns the jacobians evaluated in the states provided in std::vector of mapped Eigen::VectorXs.
         * IMPORTANT: only fill the jacobians of the state blocks specified in _compute_jacobian.
         *
         * \param _st_vector is a vector containing the mapped eigen vectors of all state blocks involved in the constraint
         * \param jacobians is an output vector of mapped eigen matrices that sould contain the jacobians w.r.t each state block
         * \param _compute_jacobian is a vector that specifies whether the ith jacobian sould be computed or not
         *
         **/
        virtual void evaluateJacobians(const std::vector<Eigen::Map<const Eigen::VectorXs> >& _st_vector,
                                       std::vector<Eigen::Map<Eigen::MatrixXs> >& jacobians,
                                       const std::vector<bool>& _compute_jacobian) const;

        /** \brief Returns the pure jacobians (without measurement noise) evaluated in the state blocks values
         * \param _st_vector is a vector containing the mapped eigen vectors of all state blocks involved in the constraint
         * \param jacobians is an output vector of mapped eigen matrices that sould contain the jacobians w.r.t each state block
         *
         **/
        virtual void evaluatePureJacobians(std::vector<Eigen::MatrixXs>& jacobians) const;

        /** \brief Returns the jacobians computation method
         **/
        virtual JacobianMethod getJacobianMethod() const
        {
            return JAC_AUTO;
        }
};


/// IMPLEMENTATION ///

inline Eigen::VectorXs ConstraintRelative2DAnalytic::evaluateResiduals(
        const std::vector<Eigen::Map<const Eigen::VectorXs> >& _st_vector) const
{
    Eigen::VectorXs residual(3);
    Eigen::VectorXs expected_measurement(3);
    // Expected measurement
    Eigen::Matrix2s R = Eigen::Rotation2D<Scalar>(-_st_vector[1](0)).matrix();
    expected_measurement.head(2) = R * (_st_vector[2] - _st_vector[0]); // rotar menys l'angle de primer (-_o1)
    expected_measurement(2) = _st_vector[3](0) - _st_vector[1](0);
    // Residual
    residual = expected_measurement - getMeasurement();
    while (residual(2) > M_PI)
        residual(2) = residual(2) - 2 * M_PI;
    while (residual(2) <= -M_PI)
        residual(2) = residual(2) + 2 * M_PI;
    residual = getMeasurementSquareRootInformationTransposed() * residual;
    return residual;
}

inline void ConstraintRelative2DAnalytic::evaluateJacobians(
        const std::vector<Eigen::Map<const Eigen::VectorXs> >& _st_vector,
        std::vector<Eigen::Map<Eigen::MatrixXs> >& jacobians, const std::vector<bool>& _compute_jacobian) const
{
    jacobians[0] << -cos(_st_vector[1](0)), -sin(_st_vector[1](0)), sin(_st_vector[1](0)), -cos(_st_vector[1](0)), 0, 0;
    jacobians[0] = getMeasurementSquareRootInformationTransposed() * jacobians[0];
    jacobians[1]
            << -(_st_vector[2](0) - _st_vector[0](0)) * sin(_st_vector[1](0))
                    + (_st_vector[2](1) - _st_vector[0](1)) * cos(_st_vector[1](0)), -(_st_vector[2](0)
            - _st_vector[0](0)) * cos(_st_vector[1](0)) - (_st_vector[2](1) - _st_vector[0](1)) * sin(_st_vector[1](0)), -1;
    jacobians[1] = getMeasurementSquareRootInformationTransposed() * jacobians[0];
    jacobians[2] << cos(_st_vector[1](0)), sin(_st_vector[1](0)), -sin(_st_vector[1](0)), cos(_st_vector[1](0)), 0, 0;
    jacobians[2] = getMeasurementSquareRootInformationTransposed() * jacobians[0];
    jacobians[3] << 0, 0, 1;
    jacobians[3] = getMeasurementSquareRootInformationTransposed() * jacobians[0];
}

inline void ConstraintRelative2DAnalytic::evaluatePureJacobians(std::vector<Eigen::MatrixXs>& jacobians) const
{
    jacobians[0]
              << -cos(getStateBlockPtrVector()[1]->getState()(0)), -sin(getStateBlockPtrVector()[1]->getState()(0)), sin(
            getStateBlockPtrVector()[1]->getState()(0)), -cos(getStateBlockPtrVector()[1]->getState()(0)), 0, 0;

    jacobians[1]
              << -(getStateBlockPtrVector()[2]->getState()(0) - getStateBlockPtrVector()[0]->getState()(0))
                    * sin(getStateBlockPtrVector()[1]->getState()(0))
                    + (getStateBlockPtrVector()[2]->getState()(1) - getStateBlockPtrVector()[0]->getState()(1))
                            * cos(getStateBlockPtrVector()[1]->getState()(0)), -(getStateBlockPtrVector()[2]->getState()(0)
            - getStateBlockPtrVector()[0]->getState()(0)) * cos(getStateBlockPtrVector()[1]->getState()(0))
            - (getStateBlockPtrVector()[2]->getState()(1) - getStateBlockPtrVector()[0]->getState()(1))
                    * sin(getStateBlockPtrVector()[1]->getState()(0)), -1;

    jacobians[2]
              << cos(getStateBlockPtrVector()[1]->getState()(0)), sin(getStateBlockPtrVector()[1]->getState()(0)), -sin(
            getStateBlockPtrVector()[1]->getState()(0)), cos(getStateBlockPtrVector()[1]->getState()(0)), 0, 0;

    jacobians[3]
              << 0, 0, 1;
}

} // namespace wolf

#endif
