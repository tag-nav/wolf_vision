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

// Factors
#include "base/factor/factor_odom_2D.h"
#include "base/factor/factor_base.h"

namespace wolf {

// Wolf ceres auto_diff creator
template <class T>
std::shared_ptr<ceres::NumericDiffCostFunction<T, ceres::CENTRAL, T::residualSize,
                                               T::block0Size,T::block1Size,T::block2Size,T::block3Size,T::block4Size,
                                               T::block5Size,T::block6Size,T::block7Size,T::block8Size,T::block9Size> > createNumericDiffCostFunctionCeres(FactorBasePtr _factor_ptr)
{
    return std::make_shared<ceres::NumericDiffCostFunction<T, ceres::CENTRAL, T::residualSize,
                                                           T::block0Size,T::block1Size,T::block2Size,T::block3Size,T::block4Size,
                                                           T::block5Size,T::block6Size,T::block7Size,T::block8Size,T::block9Size> >(std::static_pointer_cast<T>(_factor_ptr).get());
};

inline std::shared_ptr<ceres::CostFunction> createNumericDiffCostFunction(FactorBasePtr _fac_ptr)
{
//    switch (_fac_ptr->getTypeId())
//    {
//        // just for testing
//        case FAC_ODOM_2D:
//            return createNumericDiffCostFunctionCeres<FactorOdom2D>(_fac_ptr);
//
//        /* For adding a new factor, add the #include and a case:
//        case FAC_ENUM:
//            return createNumericDiffCostFunctionCeres<FactorType>(_fac_ptr);
//         */
//
//        default:
            throw std::invalid_argument( "Unknown factor type! Please add it in the file: ceres_wrapper/create_Numeric_diff_cost_function.h" );
//    }
}

} // namespace wolf

#endif /* SRC_CERES_WRAPPER_CREATE_NUMERIC_DIFF_COST_FUNCTION_H_ */
