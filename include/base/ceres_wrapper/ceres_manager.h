#ifndef CERES_MANAGER_H_
#define CERES_MANAGER_H_

//Ceres includes
#include "ceres/jet.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

//wolf includes
#include "base/solver/solver_manager.h"
#include "base/ceres_wrapper/cost_function_wrapper.h"
#include "local_parametrization_wrapper.h"
#include "base/ceres_wrapper/create_numeric_diff_cost_function.h"

namespace ceres {
typedef std::shared_ptr<CostFunction>  CostFunctionPtr;
}

namespace wolf {

WOLF_PTR_TYPEDEFS(CeresManager);

/** \brief Ceres manager for WOLF
 *
 */

class CeresManager : public SolverManager
{
    protected:

        std::map<ConstraintBasePtr, ceres::ResidualBlockId> ctr_2_residual_idx_;
        std::map<ConstraintBasePtr, ceres::CostFunctionPtr> ctr_2_costfunction_;

        std::map<StateBlockPtr, LocalParametrizationWrapperPtr> state_blocks_local_param_;

        ceres::Solver::Options ceres_options_;
        ceres::Solver::Summary summary_;
        std::unique_ptr<ceres::Problem> ceres_problem_;
        std::unique_ptr<ceres::Covariance> covariance_;

    public:

        CeresManager(const ProblemPtr& _wolf_problem,
                     const ceres::Solver::Options& _ceres_options
                     = ceres::Solver::Options());

        ~CeresManager();

        ceres::Solver::Summary getSummary();

        virtual void computeCovariances(CovarianceBlocksToBeComputed _blocks
                                        = CovarianceBlocksToBeComputed::ROBOT_LANDMARKS) override;

        virtual void computeCovariances(const StateBlockList& st_list) override;

        ceres::Solver::Options& getSolverOptions();

        void check();

    private:

        std::string solveImpl(const ReportVerbosity report_level) override;

        void addConstraint(const ConstraintBasePtr& ctr_ptr) override;

        void removeConstraint(const ConstraintBasePtr& ctr_ptr) override;

        void addStateBlock(const StateBlockPtr& state_ptr) override;

        void removeStateBlock(const StateBlockPtr& state_ptr) override;

        void updateStateBlockStatus(const StateBlockPtr& state_ptr) override;

        void updateStateBlockLocalParametrization(const StateBlockPtr& state_ptr) override;

        ceres::CostFunctionPtr createCostFunction(const ConstraintBasePtr& _ctr_ptr);
};

inline ceres::Solver::Summary CeresManager::getSummary()
{
    return summary_;
}

inline ceres::Solver::Options& CeresManager::getSolverOptions()
{
    return ceres_options_;
}

} // namespace wolf

#endif
