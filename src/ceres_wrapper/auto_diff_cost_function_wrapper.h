#ifndef TRUNK_SRC_AUTODIFF_COST_FUNCTION_WRAPPER_H_
#define TRUNK_SRC_AUTODIFF_COST_FUNCTION_WRAPPER_H_

// WOLF
#include "auto_diff_cost_function_wrapper_base.h"

template <class CtrType>
class AutoDiffCostFunctionWrapper : public AutoDiffCostFunctionWrapperBase<CtrType,
                                                                           CtrType::measurementSize,
                                                                           CtrType::block0Size,CtrType::block1Size,
                                                                           CtrType::block2Size,CtrType::block3Size,
                                                                           CtrType::block4Size,CtrType::block5Size,
                                                                           CtrType::block6Size,CtrType::block7Size,
                                                                           CtrType::block8Size,CtrType::block9Size>
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

    public:

        AutoDiffCostFunctionWrapper(ConstraintBase* _constraint_ptr) :
            AutoDiffCostFunctionWrapperBase<CtrType, CtrType::measurementSize,
                                            CtrType::block0Size,CtrType::block1Size,CtrType::block2Size,CtrType::block3Size,CtrType::block4Size,
                                            CtrType::block5Size,CtrType::block6Size,CtrType::block7Size,CtrType::block8Size,CtrType::block9Size>((CtrType*)_constraint_ptr)
        {
            //
        };

        virtual ~AutoDiffCostFunctionWrapper()
        {

        };

};

#endif /* TRUNK_SRC_AUTODIFF_COST_FUNCTION_WRAPPER_H_ */
