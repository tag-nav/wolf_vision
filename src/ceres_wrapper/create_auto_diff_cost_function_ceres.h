/*
 * create_auto_diff_cost_function_ceres.h
 *
 *  Created on: Apr 5, 2016
 *      Author: jvallve
 */

#ifndef SRC_CERES_WRAPPER_CREATE_AUTO_DIFF_COST_FUNCTION_CERES_H_
#define SRC_CERES_WRAPPER_CREATE_AUTO_DIFF_COST_FUNCTION_CERES_H_

#include "ceres/autodiff_cost_function.h"

template <class CtrType>
ceres::AutoDiffCostFunction<CtrType,
                            CtrType::measurementSize,
                            CtrType::block0Size,CtrType::block1Size,
                            CtrType::block2Size,CtrType::block3Size,
                            CtrType::block4Size,CtrType::block5Size,
                            CtrType::block6Size,CtrType::block7Size,
                            CtrType::block8Size,CtrType::block9Size>* createAutoDiffCostFunctionCeres(ConstraintBase* _constraint_ptr)
{
    static_assert(CtrType::measurementSize != 0,"Measurement size cannot be null!");
    static_assert(!(CtrType::block0Size == 0 ||
                   (CtrType::block1Size > 0 && CtrType::block0Size == 0) ||
                   (CtrType::block2Size > 0 && (CtrType::block0Size == 0 || CtrType::block1Size == 0)) ||
                   (CtrType::block3Size > 0 && (CtrType::block0Size == 0 || CtrType::block1Size == 0 || CtrType::block2Size == 0)) ||
                   (CtrType::block4Size > 0 && (CtrType::block0Size == 0 || CtrType::block1Size == 0 || CtrType::block2Size == 0 || CtrType::block3Size == 0)) ||
                   (CtrType::block5Size > 0 && (CtrType::block0Size == 0 || CtrType::block1Size == 0 || CtrType::block2Size == 0 || CtrType::block3Size == 0 || CtrType::block4Size == 0)) ||
                   (CtrType::block6Size > 0 && (CtrType::block0Size == 0 || CtrType::block1Size == 0 || CtrType::block2Size == 0 || CtrType::block3Size == 0 || CtrType::block4Size == 0 || CtrType::block5Size == 0)) ||
                   (CtrType::block7Size > 0 && (CtrType::block0Size == 0 || CtrType::block1Size == 0 || CtrType::block2Size == 0 || CtrType::block3Size == 0 || CtrType::block4Size == 0 || CtrType::block5Size == 0 || CtrType::block6Size == 0)) ||
                   (CtrType::block8Size > 0 && (CtrType::block0Size == 0 || CtrType::block1Size == 0 || CtrType::block2Size == 0 || CtrType::block3Size == 0 || CtrType::block4Size == 0 || CtrType::block5Size == 0 || CtrType::block6Size == 0 || CtrType::block7Size == 0)) ||
                   (CtrType::block9Size > 0 && (CtrType::block0Size == 0 || CtrType::block1Size == 0 || CtrType::block2Size == 0 || CtrType::block3Size == 0 || CtrType::block4Size == 0 || CtrType::block5Size == 0 || CtrType::block6Size == 0 || CtrType::block7Size == 0 || CtrType::block8Size == 0))),
                  "bad block sizes numbers!");

    return new ceres::AutoDiffCostFunction<CtrType, CtrType::measurementSize,
                                           CtrType::block0Size,CtrType::block1Size,CtrType::block2Size,CtrType::block3Size,CtrType::block4Size,
                                           CtrType::block5Size,CtrType::block6Size,CtrType::block7Size,CtrType::block8Size,CtrType::block9Size>((CtrType*)_constraint_ptr);
};


#endif /* SRC_CERES_WRAPPER_CREATE_AUTO_DIFF_COST_FUNCTION_CERES_H_ */
