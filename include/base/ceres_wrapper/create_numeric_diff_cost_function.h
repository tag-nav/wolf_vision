/*
 * create_numeric_diff_cost_function.h
 *
 *  Created on: Apr 5, 2016
 *      Author: jvallve
 */

#ifndef SRC_CERES_WRAPPER_CREATE_NUMERIC_DIFF_COST_FUNCTION_H_
#define SRC_CERES_WRAPPER_CREATE_NUMERIC_DIFF_COST_FUNCTION_H_

#include "ceres/cost_function.h"
#include "ceres/numeric_diff_cost_function.h"

// Constraints
#include "base/constraint/constraint_odom_2D.h"
#include "base/constraint/constraint_base.h"

namespace wolf {

// Wolf ceres auto_diff creator
template <class T>
std::shared_ptr<ceres::NumericDiffCostFunction<T, ceres::CENTRAL, T::residualSize,
                                               T::block0Size,T::block1Size,T::block2Size,T::block3Size,T::block4Size,
                                               T::block5Size,T::block6Size,T::block7Size,T::block8Size,T::block9Size> > createNumericDiffCostFunctionCeres(ConstraintBasePtr _constraint_ptr)
{
    return std::make_shared<ceres::NumericDiffCostFunction<T, ceres::CENTRAL, T::residualSize,
                                                           T::block0Size,T::block1Size,T::block2Size,T::block3Size,T::block4Size,
                                                           T::block5Size,T::block6Size,T::block7Size,T::block8Size,T::block9Size> >(std::static_pointer_cast<T>(_constraint_ptr).get());
};

inline std::shared_ptr<ceres::CostFunction> createNumericDiffCostFunction(ConstraintBasePtr _ctr_ptr)
{
//    switch (_ctr_ptr->getTypeId())
//    {
//        // just for testing
//        case CTR_ODOM_2D:
//            return createNumericDiffCostFunctionCeres<ConstraintOdom2D>(_ctr_ptr);
//
//        /* For adding a new constraint, add the #include and a case:
//        case CTR_ENUM:
//            return createNumericDiffCostFunctionCeres<ConstraintType>(_ctr_ptr);
//         */
//
//        default:
            throw std::invalid_argument( "Unknown constraint type! Please add it in the file: ceres_wrapper/create_Numeric_diff_cost_function.h" );
//    }
}

} // namespace wolf

#endif /* SRC_CERES_WRAPPER_CREATE_NUMERIC_DIFF_COST_FUNCTION_H_ */
