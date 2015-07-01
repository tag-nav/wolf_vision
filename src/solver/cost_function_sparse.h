#ifndef TRUNK_SRC_SOLVER_COST_FUNCTION_SPARSE_H_
#define TRUNK_SRC_SOLVER_COST_FUNCTION_SPARSE_H_

//wolf includes
#include "wolf.h"
#include "cost_function_sparse_base.h"

// CERES JET
#include "ceres/jet.h"

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
class CostFunctionSparse : CostFunctionSparseBase<ConstraintT,
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
                                                        BLOCK_9_SIZE>
{
    public:
        CostFunctionSparse(ConstraintT* _constraint_ptr) :
            CostFunctionSparse<ConstraintT,
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
                                BLOCK_9_SIZE>(_constraint_ptr)
        {

        }

        void callFunctor()
        {
                (*this->constraint_ptr_)(this->jets_0_, this->jets_1_, this->jets_2_, this->jets_3_, this->jets_4_, this->jets_5_, this->jets_6_, this->jets_7_, this->jets_8_ , this->jets_9_ ,this->residuals_jet_);
        }

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
                unsigned int BLOCK_8_SIZE>
class CostFunctionSparse<ConstraintT,
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
                                    0> : CostFunctionSparseBase<ConstraintT,
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
                                                        0>
{
    public:
        CostFunctionSparse(ConstraintT* _constraint_ptr) :
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
                                0>(_constraint_ptr)
        {

        }

        void callFunctor()
        {
                (*this->constraint_ptr_)(this->jets_0_, this->jets_1_, this->jets_2_, this->jets_3_, this->jets_4_, this->jets_5_, this->jets_6_, this->jets_7_, this->jets_8_ ,this->residuals_jet_);
        }
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
                unsigned int BLOCK_7_SIZE>
class CostFunctionSparse<ConstraintT,
                                    MEASUREMENT_SIZE,
                                    BLOCK_0_SIZE,
                                    BLOCK_1_SIZE,
                                    BLOCK_2_SIZE,
                                    BLOCK_3_SIZE,
                                    BLOCK_4_SIZE,
                                    BLOCK_5_SIZE,
                                    BLOCK_6_SIZE,
                                    BLOCK_7_SIZE,
                                    0,
                                    0> : CostFunctionSparseBase<ConstraintT,
                                                        MEASUREMENT_SIZE,
                                                        BLOCK_0_SIZE,
                                                        BLOCK_1_SIZE,
                                                        BLOCK_2_SIZE,
                                                        BLOCK_3_SIZE,
                                                        BLOCK_4_SIZE,
                                                        BLOCK_5_SIZE,
                                                        BLOCK_6_SIZE,
                                                        BLOCK_7_SIZE,
                                                        0,
                                                        0>
{
    public:
        CostFunctionSparse(ConstraintT* _constraint_ptr) :
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
                                0,
                                0>(_constraint_ptr)
        {

        }

        void callFunctor()
        {
                (*this->constraint_ptr_)(this->jets_0_, this->jets_1_, this->jets_2_, this->jets_3_, this->jets_4_, this->jets_5_, this->jets_6_, this->jets_7_,this->residuals_jet_);
        }
};

template <class ConstraintT,
          const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE,
                unsigned int BLOCK_3_SIZE,
                unsigned int BLOCK_4_SIZE,
                unsigned int BLOCK_5_SIZE,
                unsigned int BLOCK_6_SIZE>
class CostFunctionSparse<ConstraintT,
                                    MEASUREMENT_SIZE,
                                    BLOCK_0_SIZE,
                                    BLOCK_1_SIZE,
                                    BLOCK_2_SIZE,
                                    BLOCK_3_SIZE,
                                    BLOCK_4_SIZE,
                                    BLOCK_5_SIZE,
                                    BLOCK_6_SIZE,
                                    0,
                                    0,
                                    0> : CostFunctionSparseBase<ConstraintT,
                                                        MEASUREMENT_SIZE,
                                                        BLOCK_0_SIZE,
                                                        BLOCK_1_SIZE,
                                                        BLOCK_2_SIZE,
                                                        BLOCK_3_SIZE,
                                                        BLOCK_4_SIZE,
                                                        BLOCK_5_SIZE,
                                                        BLOCK_6_SIZE,
                                                        0,
                                                        0,
                                                        0>
{
    public:
        CostFunctionSparse(ConstraintT* _constraint_ptr) :
            CostFunctionSparseBase<ConstraintT,
                                MEASUREMENT_SIZE,
                                BLOCK_0_SIZE,
                                BLOCK_1_SIZE,
                                BLOCK_2_SIZE,
                                BLOCK_3_SIZE,
                                BLOCK_4_SIZE,
                                BLOCK_5_SIZE,
                                BLOCK_6_SIZE,
                                0,
                                0,
                                0>(_constraint_ptr)
        {

        }

        void callFunctor()
        {
                (*this->constraint_ptr_)(this->jets_0_, this->jets_1_, this->jets_2_, this->jets_3_, this->jets_4_, this->jets_5_, this->jets_6_,this->residuals_jet_);
        }
};

template <class ConstraintT,
          const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE,
                unsigned int BLOCK_3_SIZE,
                unsigned int BLOCK_4_SIZE,
                unsigned int BLOCK_5_SIZE>
class CostFunctionSparse<ConstraintT,
                                    MEASUREMENT_SIZE,
                                    BLOCK_0_SIZE,
                                    BLOCK_1_SIZE,
                                    BLOCK_2_SIZE,
                                    BLOCK_3_SIZE,
                                    BLOCK_4_SIZE,
                                    BLOCK_5_SIZE,
                                    0,
                                    0,
                                    0,
                                    0> : CostFunctionSparseBase<ConstraintT,
                                                        MEASUREMENT_SIZE,
                                                        BLOCK_0_SIZE,
                                                        BLOCK_1_SIZE,
                                                        BLOCK_2_SIZE,
                                                        BLOCK_3_SIZE,
                                                        BLOCK_4_SIZE,
                                                        BLOCK_5_SIZE,
                                                        0,
                                                        0,
                                                        0,
                                                        0>
{
    public:
        CostFunctionSparse(ConstraintT* _constraint_ptr) :
            CostFunctionSparseBase<ConstraintT,
                                MEASUREMENT_SIZE,
                                BLOCK_0_SIZE,
                                BLOCK_1_SIZE,
                                BLOCK_2_SIZE,
                                BLOCK_3_SIZE,
                                BLOCK_4_SIZE,
                                BLOCK_5_SIZE,
                                0,
                                0,
                                0,
                                0>(_constraint_ptr)
        {

        }

        void callFunctor()
        {
                (*this->constraint_ptr_)(this->jets_0_, this->jets_1_, this->jets_2_, this->jets_3_, this->jets_4_, this->jets_5_,this->residuals_jet_);
        }
};

template <class ConstraintT,
          const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE,
                unsigned int BLOCK_3_SIZE,
                unsigned int BLOCK_4_SIZE>
class CostFunctionSparse<ConstraintT,
                                    MEASUREMENT_SIZE,
                                    BLOCK_0_SIZE,
                                    BLOCK_1_SIZE,
                                    BLOCK_2_SIZE,
                                    BLOCK_3_SIZE,
                                    BLOCK_4_SIZE,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0> : CostFunctionSparseBase<ConstraintT,
                                                        MEASUREMENT_SIZE,
                                                        BLOCK_0_SIZE,
                                                        BLOCK_1_SIZE,
                                                        BLOCK_2_SIZE,
                                                        BLOCK_3_SIZE,
                                                        BLOCK_4_SIZE,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        0>
{
    public:
        CostFunctionSparse(ConstraintT* _constraint_ptr) :
            CostFunctionSparseBase<ConstraintT,
                                MEASUREMENT_SIZE,
                                BLOCK_0_SIZE,
                                BLOCK_1_SIZE,
                                BLOCK_2_SIZE,
                                BLOCK_3_SIZE,
                                BLOCK_4_SIZE,
                                0,
                                0,
                                0,
                                0,
                                0>(_constraint_ptr)
        {

        }

        void callFunctor()
        {
                (*this->constraint_ptr_)(this->jets_0_, this->jets_1_, this->jets_2_, this->jets_3_, this->jets_4_,this->residuals_jet_);
        }
};

template <class ConstraintT,
          const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE,
                unsigned int BLOCK_3_SIZE>
class CostFunctionSparse<ConstraintT,
                                    MEASUREMENT_SIZE,
                                    BLOCK_0_SIZE,
                                    BLOCK_1_SIZE,
                                    BLOCK_2_SIZE,
                                    BLOCK_3_SIZE,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0> : CostFunctionSparseBase<ConstraintT,
                                                        MEASUREMENT_SIZE,
                                                        BLOCK_0_SIZE,
                                                        BLOCK_1_SIZE,
                                                        BLOCK_2_SIZE,
                                                        BLOCK_3_SIZE,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        0>
{
    public:
        CostFunctionSparse(ConstraintT* _constraint_ptr) :
            CostFunctionSparseBase<ConstraintT,
                                MEASUREMENT_SIZE,
                                BLOCK_0_SIZE,
                                BLOCK_1_SIZE,
                                BLOCK_2_SIZE,
                                BLOCK_3_SIZE,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0>(_constraint_ptr)
        {

        }

        void callFunctor()
        {
                (*this->constraint_ptr_)(this->jets_0_, this->jets_1_, this->jets_2_, this->jets_3_,this->residuals_jet_);
        }
};

template <class ConstraintT,
          const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE>
class CostFunctionSparse<ConstraintT,
                                    MEASUREMENT_SIZE,
                                    BLOCK_0_SIZE,
                                    BLOCK_1_SIZE,
                                    BLOCK_2_SIZE,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0> : CostFunctionSparseBase<ConstraintT,
                                                        MEASUREMENT_SIZE,
                                                        BLOCK_0_SIZE,
                                                        BLOCK_1_SIZE,
                                                        BLOCK_2_SIZE,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        0>
{
    public:
        CostFunctionSparse(ConstraintT* _constraint_ptr) :
            CostFunctionSparseBase<ConstraintT,
                                MEASUREMENT_SIZE,
                                BLOCK_0_SIZE,
                                BLOCK_1_SIZE,
                                BLOCK_2_SIZE,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0>(_constraint_ptr)
        {

        }

        void callFunctor()
        {
                (*this->constraint_ptr_)(this->jets_0_, this->jets_1_, this->jets_2_,this->residuals_jet_);
        }
};

template <class ConstraintT,
          const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE>
class CostFunctionSparse<ConstraintT,
                                    MEASUREMENT_SIZE,
                                    BLOCK_0_SIZE,
                                    BLOCK_1_SIZE,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0> : CostFunctionSparseBase<ConstraintT,
                                                        MEASUREMENT_SIZE,
                                                        BLOCK_0_SIZE,
                                                        BLOCK_1_SIZE,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        0>
{
    public:
        CostFunctionSparse(ConstraintT* _constraint_ptr) :
            CostFunctionSparseBase<ConstraintT,
                                MEASUREMENT_SIZE,
                                BLOCK_0_SIZE,
                                BLOCK_1_SIZE,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0>(_constraint_ptr)
        {

        }

        void callFunctor()
        {
                (*this->constraint_ptr_)(this->jets_0_, this->jets_1_,this->residuals_jet_);
        }
};

template <class ConstraintT,
          const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE>
class CostFunctionSparse<ConstraintT,
                                    MEASUREMENT_SIZE,
                                    BLOCK_0_SIZE,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0> : CostFunctionSparseBase<ConstraintT,
                                                        MEASUREMENT_SIZE,
                                                        BLOCK_0_SIZE,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        0,
                                                        0>
{
    public:
        CostFunctionSparse(ConstraintT* _constraint_ptr) :
            CostFunctionSparseBase<ConstraintT,
                                MEASUREMENT_SIZE,
                                BLOCK_0_SIZE,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0>(_constraint_ptr)
        {

        }

        void callFunctor()
        {
                (*this->constraint_ptr_)(this->jets_0_,this->residuals_jet_);
        }
};

#endif /* TRUNK_SRC_SOLVER_COST_FUNCTION_SPARSE_H_ */
