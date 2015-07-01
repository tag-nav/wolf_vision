/*
 * cost_function_sparse.h
 *
 *  Created on: Jun 25, 2015
 *      Author: jvallve
 */

#ifndef TRUNK_SRC_SOLVER_COST_FUNCTION_SPARSE_BASE_H_
#define TRUNK_SRC_SOLVER_COST_FUNCTION_SPARSE_BASE_H_

//wolf includes
#include "wolf.h"
#include "cost_function_base.h"

// CERES JET
#include "ceres/jet.h"

template <class ConstraintT,
          const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE = 0,
                unsigned int BLOCK_2_SIZE = 0,
                unsigned int BLOCK_3_SIZE = 0,
                unsigned int BLOCK_4_SIZE = 0,
                unsigned int BLOCK_5_SIZE = 0,
                unsigned int BLOCK_6_SIZE = 0,
                unsigned int BLOCK_7_SIZE = 0,
                unsigned int BLOCK_8_SIZE = 0,
                unsigned int BLOCK_9_SIZE = 0>
class CostFunctionSparseBase : CostFunctionBase
{
        typedef ceres::Jet<WolfScalar, BLOCK_0_SIZE +
                                       BLOCK_1_SIZE +
                                       BLOCK_2_SIZE +
                                       BLOCK_3_SIZE +
                                       BLOCK_4_SIZE +
                                       BLOCK_5_SIZE +
                                       BLOCK_6_SIZE +
                                       BLOCK_7_SIZE +
                                       BLOCK_8_SIZE +
                                       BLOCK_9_SIZE> WolfJet;
    protected:
        ConstraintT* constraint_ptr_;
        WolfJet jets_0_[BLOCK_0_SIZE];
        WolfJet jets_1_[BLOCK_1_SIZE];
        WolfJet jets_2_[BLOCK_2_SIZE];
        WolfJet jets_3_[BLOCK_3_SIZE];
        WolfJet jets_4_[BLOCK_4_SIZE];
        WolfJet jets_5_[BLOCK_5_SIZE];
        WolfJet jets_6_[BLOCK_6_SIZE];
        WolfJet jets_7_[BLOCK_7_SIZE];
        WolfJet jets_8_[BLOCK_8_SIZE];
        WolfJet jets_9_[BLOCK_9_SIZE];
        WolfJet residuals_jet_[MEASUREMENT_SIZE];

    public:

        /** \brief Constructor with constraint pointer
         *
         * Constructor with constraint pointer
         *
         */
        CostFunctionSparseBase(ConstraintT* _constraint_ptr);

        /** \brief Default destructor
         *
         * Default destructor
         *
         */
        virtual ~CostFunctionSparseBase();

        /** \brief Evaluate residuals and jacobians of the constraint in the current x
         *
         * Evaluate residuals and jacobians of the constraint in the current x
         *
         */
        virtual void evaluateResidualJacobians();

    protected:

        /** \brief Calls the functor of the constraint evaluating jets
         *
         * Calls the functor of the constraint evaluating jets
         *
         */
        virtual void callFunctor() = 0;

        /** \brief Initialize the infinitesimal part of jets
         *
         * Initialize the infinitesimal part of jets with zeros and ones
         *
         */
        void initializeJets();

        /** \brief Gets the evaluation point
         *
         * Gets the evaluation point from the state
         *
         */
        void evaluateX();
};

template <class ConstraintT,
          const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE,
                unsigned int BLOCK_3_SIZE,
                unsigned int BLOCK_4_SIZE,
                unsigned int BLOCK_5_SIZE,
                unsigned int BLOCK_6_SIZE,
                unsigned int BLOCK_7_SIZE,
                unsigned int BLOCK_8_SIZE,
                unsigned int BLOCK_9_SIZE>
CostFunctionSparseBase<ConstraintT,
                   MEASUREMENT_SIZE,
                   BLOCK_0_SIZE,
                   BLOCK_1_SIZE,
                   BLOCK_2_SIZE,
                   BLOCK_3_SIZE,
                   BLOCK_4_SIZE,
                   BLOCK_5_SIZE,
                   BLOCK_6_SIZE,
                   BLOCK_7_SIZE,
                   BLOCK_8_SIZE,
                   BLOCK_9_SIZE>::CostFunctionSparseBase(ConstraintT* _constraint_ptr) :
    CostFunctionBase(MEASUREMENT_SIZE, BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, BLOCK_4_SIZE, BLOCK_5_SIZE, BLOCK_6_SIZE, BLOCK_7_SIZE, BLOCK_8_SIZE,BLOCK_9_SIZE),
    constraint_ptr_(_constraint_ptr)
{
    initializeJets();
}

template <class ConstraintT,
          const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE,
                unsigned int BLOCK_3_SIZE,
                unsigned int BLOCK_4_SIZE,
                unsigned int BLOCK_5_SIZE,
                unsigned int BLOCK_6_SIZE,
                unsigned int BLOCK_7_SIZE,
                unsigned int BLOCK_8_SIZE,
                unsigned int BLOCK_9_SIZE>
CostFunctionSparseBase<ConstraintT,
                   MEASUREMENT_SIZE,
                   BLOCK_0_SIZE,
                   BLOCK_1_SIZE,
                   BLOCK_2_SIZE,
                   BLOCK_3_SIZE,
                   BLOCK_4_SIZE,
                   BLOCK_5_SIZE,
                   BLOCK_6_SIZE,
                   BLOCK_7_SIZE,
                   BLOCK_8_SIZE,
                   BLOCK_9_SIZE>::~CostFunctionSparseBase()
{

}

template <class ConstraintT,
          const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE,
                unsigned int BLOCK_3_SIZE,
                unsigned int BLOCK_4_SIZE,
                unsigned int BLOCK_5_SIZE,
                unsigned int BLOCK_6_SIZE,
                unsigned int BLOCK_7_SIZE,
                unsigned int BLOCK_8_SIZE,
                unsigned int BLOCK_9_SIZE>
void CostFunctionSparseBase<ConstraintT,
                   MEASUREMENT_SIZE,
                   BLOCK_0_SIZE,
                   BLOCK_1_SIZE,
                   BLOCK_2_SIZE,
                   BLOCK_3_SIZE,
                   BLOCK_4_SIZE,
                   BLOCK_5_SIZE,
                   BLOCK_6_SIZE,
                   BLOCK_7_SIZE,
                   BLOCK_8_SIZE,
                   BLOCK_9_SIZE>::evaluateResidualJacobians()
{
    evaluateX();

    callFunctor();

    // fill the jacobian matrices
    int jacobian_location = 0;
    for (int i = 0; i<n_blocks_; i++)
    {
        for (int row = 0; row < MEASUREMENT_SIZE; row++)
            jacobians_.at(i)->row(row) = residuals_jet_[row].v.segment(jacobian_location, block_sizes_.at(i));
        jacobian_location += block_sizes_.at(i);
        std::cout << "filled jacobian " << i << ":" << std::endl << (*jacobians_.at(i)) << std::endl;
    }

    // fill the residual vector
    for (int i = 0; i < MEASUREMENT_SIZE; i++)
        residual_(i) = residuals_jet_[i].a;
}

template <class ConstraintT,
const unsigned int MEASUREMENT_SIZE,
      unsigned int BLOCK_0_SIZE,
      unsigned int BLOCK_1_SIZE,
      unsigned int BLOCK_2_SIZE,
      unsigned int BLOCK_3_SIZE,
      unsigned int BLOCK_4_SIZE,
      unsigned int BLOCK_5_SIZE,
      unsigned int BLOCK_6_SIZE,
      unsigned int BLOCK_7_SIZE,
      unsigned int BLOCK_8_SIZE,
      unsigned int BLOCK_9_SIZE>
 void CostFunctionSparseBase<ConstraintT,
                         MEASUREMENT_SIZE,
                         BLOCK_0_SIZE,
                         BLOCK_1_SIZE,
                         BLOCK_2_SIZE,
                         BLOCK_3_SIZE,
                         BLOCK_4_SIZE,
                         BLOCK_5_SIZE,
                         BLOCK_6_SIZE,
                         BLOCK_7_SIZE,
                         BLOCK_8_SIZE,
                         BLOCK_9_SIZE>::initializeJets()
{
    int last_jet_idx = 0;
    // JET 0
    for (int i = 0; i < BLOCK_0_SIZE; i++)
        jets_0_[i] = WolfJet(0, last_jet_idx++);
    // JET 1
    for (int i = 0; i < BLOCK_1_SIZE; i++)
        jets_1_[i] = WolfJet(0, last_jet_idx++);
    // JET 2
    for (int i = 0; i < BLOCK_2_SIZE; i++)
        jets_2_[i] = WolfJet(0, last_jet_idx++);
    // JET 3
    for (int i = 0; i < BLOCK_3_SIZE; i++)
        jets_3_[i] = WolfJet(0, last_jet_idx++);
    // JET 4
    for (int i = 0; i < BLOCK_4_SIZE; i++)
        jets_4_[i] = WolfJet(0, last_jet_idx++);
    // JET 5
    for (int i = 0; i < BLOCK_5_SIZE; i++)
        jets_5_[i] = WolfJet(0, last_jet_idx++);
    // JET 6
    for (int i = 0; i < BLOCK_6_SIZE; i++)
        jets_6_[i] = WolfJet(0, last_jet_idx++);
    // JET 7
    for (int i = 0; i < BLOCK_7_SIZE; i++)
        jets_7_[i] = WolfJet(0, last_jet_idx++);
    // JET 8
    for (int i = 0; i < BLOCK_8_SIZE; i++)
        jets_8_[i] = WolfJet(0, last_jet_idx++);
    // JET 9
    for (int i = 0; i < BLOCK_9_SIZE; i++)
        jets_9_[i] = WolfJet(0, last_jet_idx++);
}

template <class ConstraintT,
const unsigned int MEASUREMENT_SIZE,
      unsigned int BLOCK_0_SIZE,
      unsigned int BLOCK_1_SIZE,
      unsigned int BLOCK_2_SIZE,
      unsigned int BLOCK_3_SIZE,
      unsigned int BLOCK_4_SIZE,
      unsigned int BLOCK_5_SIZE,
      unsigned int BLOCK_6_SIZE,
      unsigned int BLOCK_7_SIZE,
      unsigned int BLOCK_8_SIZE,
      unsigned int BLOCK_9_SIZE>
 void CostFunctionSparseBase<ConstraintT,
                         MEASUREMENT_SIZE,
                         BLOCK_0_SIZE,
                         BLOCK_1_SIZE,
                         BLOCK_2_SIZE,
                         BLOCK_3_SIZE,
                         BLOCK_4_SIZE,
                         BLOCK_5_SIZE,
                         BLOCK_6_SIZE,
                         BLOCK_7_SIZE,
                         BLOCK_8_SIZE,
                         BLOCK_9_SIZE>::evaluateX()
{
    // JET 0
    for (int i = 0; i < BLOCK_0_SIZE; i++)
        jets_0_[i].a = *(constraint_ptr_->getStateBlockPtrVector().at(0)+i);
    // JET 1
    for (int i = 0; i < BLOCK_1_SIZE; i++)
        jets_1_[i].a = *(constraint_ptr_->getStateBlockPtrVector().at(1)+i);
    // JET 2
    for (int i = 0; i < BLOCK_2_SIZE; i++)
        jets_2_[i].a = *(constraint_ptr_->getStateBlockPtrVector().at(2)+i);
    // JET 3
    for (int i = 0; i < BLOCK_3_SIZE; i++)
        jets_3_[i].a = *(constraint_ptr_->getStateBlockPtrVector().at(3)+i);
    // JET 4
    for (int i = 0; i < BLOCK_4_SIZE; i++)
        jets_4_[i].a = *(constraint_ptr_->getStateBlockPtrVector().at(4)+i);
    // JET 5
    for (int i = 0; i < BLOCK_5_SIZE; i++)
        jets_5_[i].a = *(constraint_ptr_->getStateBlockPtrVector().at(5)+i);
    // JET 6
    for (int i = 0; i < BLOCK_6_SIZE; i++)
        jets_6_[i].a = *(constraint_ptr_->getStateBlockPtrVector().at(6)+i);
    // JET 7
    for (int i = 0; i < BLOCK_7_SIZE; i++)
        jets_7_[i].a = *(constraint_ptr_->getStateBlockPtrVector().at(7)+i);
    // JET 8
    for (int i = 0; i < BLOCK_8_SIZE; i++)
        jets_8_[i].a = *(constraint_ptr_->getStateBlockPtrVector().at(8)+i);
    // JET 9
    for (int i = 0; i < BLOCK_9_SIZE; i++)
        jets_9_[i].a = *(constraint_ptr_->getStateBlockPtrVector().at(9)+i);
}

#endif /* TRUNK_SRC_SOLVER_COST_FUNCTION_SPARSE_BASE_H_ */
