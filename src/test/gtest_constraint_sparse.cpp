/**
 * \file gtest_constraint_sparse.cpp
 *
 *  Created on: Apr 25, 2017
 *      \author: jsola
 */


#include "utils_gtest.h"

#include "constraint_sparse.h"

using namespace wolf;

// Dummy class for avoiding the pure virtual compilation error
template <JacobianMethod J>
class ConstraintSparseObject : public ConstraintSparse<1, 1>
{
    public:
        ConstraintSparseObject(ConstraintType _tp, bool _apply_loss_function, ConstraintStatus _status, StateBlockPtr _sb) :
                ConstraintSparse<1, 1>(_tp, _apply_loss_function, _status, _sb)
        {
            //
        }
        virtual ~ConstraintSparseObject(){}

        virtual JacobianMethod getJacobianMethod() const
        {
            return J;
        }
};

TEST(ConstraintSparseAnalytic, Constructor)
{
    ConstraintSparseObject<JAC_ANALYTIC> ctr_analytic(CTR_AHP, false, CTR_ACTIVE, std::make_shared<StateBlock>(1));
    ASSERT_EQ(ctr_analytic.getJacobianMethod(),     JAC_ANALYTIC);
    ASSERT_EQ(ctr_analytic.getApplyLossFunction(),  false);
    ASSERT_EQ(ctr_analytic.getStatus(),             CTR_ACTIVE);
    ASSERT_EQ(ctr_analytic.getSize(),               1);

    ConstraintSparseObject<JAC_AUTO> ctr_auto(CTR_AHP, false, CTR_ACTIVE, std::make_shared<StateBlock>(1));
    ASSERT_EQ(ctr_auto.getJacobianMethod(), JAC_AUTO);

    ConstraintSparseObject<JAC_NUMERIC> ctr_numeric(CTR_AHP, false, CTR_ACTIVE, std::make_shared<StateBlock>(1));
    ASSERT_EQ(ctr_numeric.getJacobianMethod(), JAC_NUMERIC);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

