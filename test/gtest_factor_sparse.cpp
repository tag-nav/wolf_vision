/**
 * \file gtest_factor_sparse.cpp
 *
 *  Created on: Apr 25, 2017
 *      \author: jsola
 */

#include "utils_gtest.h"

#include "factor_sparse.h"

using namespace wolf;

// Dummy class for avoiding the pure virtual compilation error
template <JacobianMethod J>
class FactorSparseObject : public FactorSparse<1, 2, 1>
{
    public:
        FactorSparseObject(FactorType _tp, bool _apply_loss_function, FactorStatus _status, StateBlockPtr _xy, StateBlockPtr _theta) :
                FactorSparse<1, 2, 1>(_tp, _apply_loss_function, _status, _xy, _theta)
        {
            //
        }
        virtual ~FactorSparseObject(){}

        virtual JacobianMethod getJacobianMethod() const
        {
            return J;
        }
};

TEST(FactorSparseAnalytic, Constructor)
{
    FactorSparseObject<JAC_ANALYTIC> fac_analytic(FAC_AHP, false, FAC_ACTIVE, std::make_shared<StateBlock>(2), std::make_shared<StateBlock>(1));
    ASSERT_EQ(fac_analytic.getJacobianMethod(),     JAC_ANALYTIC);
    ASSERT_EQ(fac_analytic.getApplyLossFunction(),  false);
    ASSERT_EQ(fac_analytic.getStatus(),             FAC_ACTIVE);
    ASSERT_EQ(fac_analytic.getSize(),               1);

    FactorSparseObject<JAC_AUTO> fac_auto(FAC_AHP, false, FAC_ACTIVE, std::make_shared<StateBlock>(2), std::make_shared<StateBlock>(1));
    ASSERT_EQ(fac_auto.getJacobianMethod(), JAC_AUTO);

    FactorSparseObject<JAC_NUMERIC> fac_numeric(FAC_AHP, false, FAC_ACTIVE, std::make_shared<StateBlock>(2), std::make_shared<StateBlock>(1));
    ASSERT_EQ(fac_numeric.getJacobianMethod(), JAC_NUMERIC);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

