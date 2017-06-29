#ifndef CERES_MANAGER_H_
#define CERES_MANAGER_H_

//Ceres includes
#include "ceres/jet.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

//wolf includes
#include "solver_manager.h"
#include "cost_function_wrapper.h"
#include "local_parametrization_wrapper.h"
#include "create_numeric_diff_cost_function.h"

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
		ceres::Problem* ceres_problem_;
		ceres::Solver::Options ceres_options_;
		ceres::Covariance* covariance_;
		ceres::Solver::Summary summary_;

	public:
        CeresManager(ProblemPtr _wolf_problem, const ceres::Solver::Options& _ceres_options = ceres::Solver::Options());

		~CeresManager();

		virtual std::string solve(const unsigned int& _report_level);

        ceres::Solver::Summary getSummary();

        virtual void computeCovariances(CovarianceBlocksToBeComputed _blocks = ROBOT_LANDMARKS);

        virtual void computeCovariances(const StateBlockList& st_list);

        ceres::Solver::Options& getSolverOptions();

	private:

        virtual void addConstraint(ConstraintBasePtr _ctr_ptr);

        virtual void removeConstraint(ConstraintBasePtr _ctr_ptr);

        virtual void addStateBlock(StateBlockPtr _st_ptr);

        virtual void removeStateBlock(StateBlockPtr _st_ptr);

		virtual void updateStateBlockStatus(StateBlockPtr _st_ptr);

		ceres::CostFunctionPtr createCostFunction(ConstraintBasePtr _ctr_ptr);
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
