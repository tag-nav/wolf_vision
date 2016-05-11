
#ifndef CONSTRAINT_ANALYTIC_H_
#define CONSTRAINT_ANALYTIC_H_

//Wolf includes
#include "wolf.h"
#include "constraint_base.h"

namespace wolf {

class ConstraintAnalytic: public ConstraintBase
{
    protected:
        std::vector<StateBlock*> state_ptr_vector_;
        std::vector<unsigned int> state_block_sizes_vector_;

    public:

        /** \brief Constructor of category CTR_ABSOLUTE
         *
         * Constructor of category CTR_ABSOLUTE
         *
         **/
        ConstraintAnalytic(FeatureBase* _ftr_ptr, ConstraintType _tp, bool _apply_loss_function, ConstraintStatus _status,
                         StateBlock* _state0Ptr,
                         StateBlock* _state1Ptr = nullptr,
                         StateBlock* _state2Ptr = nullptr,
                         StateBlock* _state3Ptr = nullptr,
                         StateBlock* _state4Ptr = nullptr,
                         StateBlock* _state5Ptr = nullptr,
                         StateBlock* _state6Ptr = nullptr,
                         StateBlock* _state7Ptr = nullptr,
                         StateBlock* _state8Ptr = nullptr,
                         StateBlock* _state9Ptr = nullptr ) ;

        /** \brief Constructor of category CTR_FRAME
         *
         * Constructor of category CTR_FRAME
         *
         **/
        ConstraintAnalytic(FeatureBase* _ftr_ptr, ConstraintType _tp, FrameBase* _frame_ptr, bool _apply_loss_function, ConstraintStatus _status,
                         StateBlock* _state0Ptr,
                         StateBlock* _state1Ptr = nullptr,
                         StateBlock* _state2Ptr = nullptr,
                         StateBlock* _state3Ptr = nullptr,
                         StateBlock* _state4Ptr = nullptr,
                         StateBlock* _state5Ptr = nullptr,
                         StateBlock* _state6Ptr = nullptr,
                         StateBlock* _state7Ptr = nullptr,
                         StateBlock* _state8Ptr = nullptr,
                         StateBlock* _state9Ptr = nullptr );

        /** \brief Constructor of category CTR_FEATURE
         *
         * Constructor of category CTR_FEATURE
         *
         **/
        ConstraintAnalytic(FeatureBase* _ftr_ptr, ConstraintType _tp, FeatureBase* _feature_ptr, bool _apply_loss_function, ConstraintStatus _status,
                         StateBlock* _state0Ptr,
                         StateBlock* _state1Ptr = nullptr,
                         StateBlock* _state2Ptr = nullptr,
                         StateBlock* _state3Ptr = nullptr,
                         StateBlock* _state4Ptr = nullptr,
                         StateBlock* _state5Ptr = nullptr,
                         StateBlock* _state6Ptr = nullptr,
                         StateBlock* _state7Ptr = nullptr,
                         StateBlock* _state8Ptr = nullptr,
                         StateBlock* _state9Ptr = nullptr ) ;

        /** \brief Constructor of category CTR_LANDMARK
         *
         * Constructor of category CTR_LANDMARK
         *
         **/
        ConstraintAnalytic(FeatureBase* _ftr_ptr, ConstraintType _tp, LandmarkBase* _landmark_ptr, bool _apply_loss_function, ConstraintStatus _status,
                         StateBlock* _state0Ptr,
                         StateBlock* _state1Ptr = nullptr,
                         StateBlock* _state2Ptr = nullptr,
                         StateBlock* _state3Ptr = nullptr,
                         StateBlock* _state4Ptr = nullptr,
                         StateBlock* _state5Ptr = nullptr,
                         StateBlock* _state6Ptr = nullptr,
                         StateBlock* _state7Ptr = nullptr,
                         StateBlock* _state8Ptr = nullptr,
                         StateBlock* _state9Ptr = nullptr ) ;

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         * 
         **/        
        virtual ~ConstraintAnalytic();

        /** \brief Returns a vector of pointers to the state blocks
         *
         * Returns a vector of pointers to the state blocks in which this constraint depends
         *
         **/
        virtual const std::vector<Scalar*> getStateBlockPtrVector();

        /** \brief Returns a vector of pointers to the states
         *
         * Returns a vector of pointers to the state in which this constraint depends
         *
         **/
        virtual const std::vector<StateBlock*> getStatePtrVector() const;

        /** \brief Returns a vector of sizes of the state blocks
         *
         * Returns a vector of sizes of the state blocks
         *
         **/
        virtual std::vector<unsigned int> getStateSizes() const;

        /** \brief Returns the constraint residual size
         *
         * Returns the constraint residual size
         *
         **/
        virtual unsigned int getSize() const = 0;

        /** \brief Returns the residual evaluated in the states provided
         *
         * Returns the residual evaluated in the states provided in std::vector of mapped Eigen::VectorXs
         *
         **/
        virtual Eigen::VectorXs evaluateResiduals(const std::vector<Eigen::Map<const Eigen::VectorXs>>& _st_vector) const = 0;

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
        virtual void evaluateJacobians(const std::vector<Eigen::Map<const Eigen::VectorXs>>& _st_vector, std::vector<Eigen::Map<Eigen::MatrixXs>>& jacobians, const std::vector<bool>& _compute_jacobian) const = 0;

        /** \brief Returns the pure jacobians (without measurement noise) evaluated in the state blocks values
         *
         * Returns the pure jacobians (without measurement noise) evaluated in the state blocks values
         *
         * \param _st_vector is a vector containing the mapped eigen vectors of all state blocks involved in the constraint
         * \param jacobians is an output vector of mapped eigen matrices that sould contain the jacobians w.r.t each state block
         *
         **/
        virtual void evaluatePureJacobians(std::vector<Eigen::MatrixXs>& jacobians) const = 0;

        /** \brief Returns the jacobians computation method
         *
         * Returns the jacobians computation method
         *
         **/
        virtual JacobianMethod getJacobianMethod() const;

    private:
        void resizeVectors();
};

} // namespace wolf

#endif
