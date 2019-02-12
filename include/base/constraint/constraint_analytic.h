
#ifndef CONSTRAINT_ANALYTIC_H_
#define CONSTRAINT_ANALYTIC_H_

//Wolf includes
#include "base/constraint/constraint_base.h"
#include <Eigen/StdVector>

namespace wolf {
    
WOLF_PTR_TYPEDEFS(ConstraintAnalytic);
  
class ConstraintAnalytic: public ConstraintBase
{
    protected:
        std::vector<StateBlockPtr> state_ptr_vector_;
        std::vector<unsigned int> state_block_sizes_vector_;

    public:

        /** \brief Constructor of category CTR_ABSOLUTE
         *
         * Constructor of category CTR_ABSOLUTE
         *
         **/
        ConstraintAnalytic(const std::string&  _tp,
                           bool _apply_loss_function,
                           ConstraintStatus _status,
                           StateBlockPtr _state0Ptr,
                           StateBlockPtr _state1Ptr = nullptr,
                           StateBlockPtr _state2Ptr = nullptr,
                           StateBlockPtr _state3Ptr = nullptr,
                           StateBlockPtr _state4Ptr = nullptr,
                           StateBlockPtr _state5Ptr = nullptr,
                           StateBlockPtr _state6Ptr = nullptr,
                           StateBlockPtr _state7Ptr = nullptr,
                           StateBlockPtr _state8Ptr = nullptr,
                           StateBlockPtr _state9Ptr = nullptr ) ;

        ConstraintAnalytic(const std::string&  _tp,
                           const FrameBasePtr& _frame_other_ptr,
                           const CaptureBasePtr& _capture_other_ptr,
                           const FeatureBasePtr& _feature_other_ptr,
                           const LandmarkBasePtr& _landmark_other_ptr,
                           const ProcessorBasePtr& _processor_ptr,
                           bool _apply_loss_function,
                           ConstraintStatus _status,
                           StateBlockPtr _state0Ptr,
                           StateBlockPtr _state1Ptr = nullptr,
                           StateBlockPtr _state2Ptr = nullptr,
                           StateBlockPtr _state3Ptr = nullptr,
                           StateBlockPtr _state4Ptr = nullptr,
                           StateBlockPtr _state5Ptr = nullptr,
                           StateBlockPtr _state6Ptr = nullptr,
                           StateBlockPtr _state7Ptr = nullptr,
                           StateBlockPtr _state8Ptr = nullptr,
                           StateBlockPtr _state9Ptr = nullptr );

        virtual ~ConstraintAnalytic() = default;

        /** \brief Returns a vector of pointers to the states
         *
         * Returns a vector of pointers to the state in which this constraint depends
         *
         **/
        virtual std::vector<StateBlockPtr> getStateBlockPtrVector() const override;

        /** \brief Returns a vector of sizes of the state blocks
         *
         * Returns a vector of sizes of the state blocks
         *
         **/
        virtual std::vector<unsigned int> getStateSizes() const override;

        /** \brief Returns the constraint residual size
         *
         * Returns the constraint residual size
         *
         **/
//        virtual unsigned int getSize() const = 0;

        /** \brief Evaluate the constraint given the input parameters and returning the residuals and jacobians
        **/
        virtual bool evaluate(Scalar const* const* parameters, Scalar* residuals, Scalar** jacobians) const override
        {
            // load parameters evaluation value
            std::vector<Eigen::Map<const Eigen::VectorXs>> state_blocks_map_;
            for (unsigned int i = 0; i < state_block_sizes_vector_.size(); i++)
                state_blocks_map_.push_back(Eigen::Map<const Eigen::VectorXs>((Scalar*)parameters[i], state_block_sizes_vector_[i]));

            // residuals
            Eigen::Map<Eigen::VectorXs> residuals_map((Scalar*)residuals, getSize());
            residuals_map = evaluateResiduals(state_blocks_map_);

            // also compute jacobians
            if (jacobians != nullptr)
            {
                std::vector<Eigen::Map<Eigen::MatrixXs>> jacobians_map_;
                std::vector<bool> compute_jacobians_(state_block_sizes_vector_.size());

                for (unsigned int i = 0; i < state_block_sizes_vector_.size(); i++)
                {
                    compute_jacobians_[i] = (jacobians[i] != nullptr);
                    if (jacobians[i] != nullptr)
                        jacobians_map_.push_back(Eigen::Map<Eigen::MatrixXs>((Scalar*)jacobians[i], getSize(), state_block_sizes_vector_[i]));
                    else
                        jacobians_map_.push_back(Eigen::Map<Eigen::MatrixXs>(nullptr, 0, 0)); //TODO: check if it can be done
                }

                // evaluate jacobians
                evaluateJacobians(state_blocks_map_, jacobians_map_, compute_jacobians_);
            }
            return true;

            return true;
        };

        /** Returns the residual vector and a vector of Jacobian matrix corresponding to each state block evaluated in the point provided in _states_ptr
         **/
        // TODO
        virtual void evaluate(const std::vector<const Scalar*>& _states_ptr, Eigen::VectorXs& residual_, std::vector<Eigen::MatrixXs>& jacobians_) const override
        {
        };

        /** \brief Returns the residual evaluated in the states provided
         *
         * Returns the residual evaluated in the states provided in std::vector of mapped Eigen::VectorXs
         *
         **/
        virtual Eigen::VectorXs evaluateResiduals(const std::vector<Eigen::Map<const Eigen::VectorXs>>& _st_vector) const = 0;

        /** \brief Returns the normalized jacobians evaluated in the states
         *
         * Returns the normalized (by the measurement noise sqrt information) jacobians evaluated in the states provided in std::vector of mapped Eigen::VectorXs.
         * IMPORTANT: only fill the jacobians of the state blocks specified in _compute_jacobian.
         *
         * \param _st_vector is a vector containing the mapped eigen vectors of all state blocks involved in the constraint
         * \param jacobians is an output vector of mapped eigen matrices that sould contain the jacobians w.r.t each state block
         * \param _compute_jacobian is a vector that specifies whether the ith jacobian sould be computed or not
         *
         **/
        virtual void evaluateJacobians(const std::vector<Eigen::Map<const Eigen::VectorXs>>& _st_vector, std::vector<Eigen::Map<Eigen::MatrixXs>>& jacobians, const std::vector<bool>& _compute_jacobian) const = 0;

        /** \brief Returns the jacobians (without measurement noise normalization) evaluated in the current state blocks values
         *
         * Returns the jacobians (without measurement noise normalization) evaluated in the current state blocks values
         *
         * \param jacobians is an output vector of mapped eigen matrices that sould contain the jacobians w.r.t each state block
         *
         **/
        virtual void evaluatePureJacobians(std::vector<Eigen::MatrixXs>& jacobians) const = 0;

        /** \brief Returns the jacobians computation method
         *
         * Returns the jacobians computation method
         *
         **/
        virtual JacobianMethod getJacobianMethod() const override;

    private:
        void resizeVectors();
};

} // namespace wolf

#endif
