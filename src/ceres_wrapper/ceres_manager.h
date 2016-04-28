#ifndef CERES_MANAGER_H_
#define CERES_MANAGER_H_

//Ceres includes
#include "ceres/jet.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

//wolf includes
#include "cost_function_wrapper.h"
#include "local_parametrization_wrapper.h"
#include "../wolf.h"
#include "../state_block.h"
#include "create_auto_diff_cost_function.h"
#include "create_numeric_diff_cost_function.h"

namespace wolf {



/** \brief Enumeration of covariance blocks to be computed
 *
 * Enumeration of covariance blocks to be computed
 *
 */
typedef enum
{
    ALL, ///< All blocks and all cross-covariances
    ALL_MARGINALS, ///< All marginals
    ROBOT_LANDMARKS ///< marginals of landmarks and current robot pose plus cross covariances of current robot and all landmarks
} CovarianceBlocksToBeComputed;

/** \brief Ceres manager for WOLF
 *
 */

class CeresManager
{
	protected:
		std::map<unsigned int, ceres::ResidualBlockId> id_2_residual_idx_;
        std::map<unsigned int, ceres::CostFunction*> id_2_costfunction_;
		ceres::Problem* ceres_problem_;
		ceres::Covariance* covariance_;
		Problem* wolf_problem_;
		bool use_wolf_auto_diff_;

	public:
		CeresManager(Problem* _wolf_problem, ceres::Problem::Options _options, const bool _use_wolf_cost_functions = true);

		~CeresManager();

		ceres::Solver::Summary solve(const ceres::Solver::Options& _ceres_options);

		void computeCovariances(CovarianceBlocksToBeComputed _blocks = ROBOT_LANDMARKS);

		void update(const bool _apply_loss_function = false);

		void addConstraint(ConstraintBase* _corr_ptr, const bool _apply_loss);

		void removeConstraint(const unsigned int& _corr_idx);

		void addStateBlock(StateBlock* _st_ptr);

		void removeStateBlock(double* _st_ptr);

		void removeAllStateBlocks();

		void updateStateBlockStatus(StateBlock* _st_ptr);

		ceres::CostFunction* createCostFunction(ConstraintBase* _corrPtr);
};

} // namespace wolf

#endif
