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
		WolfProblem* wolf_problem_;

	public:
		CeresManager(WolfProblem* _wolf_problem, ceres::Problem::Options _options);

		~CeresManager();

		ceres::Solver::Summary solve(const ceres::Solver::Options& _ceres_options);

		void computeCovariances(CovarianceBlocksToBeComputed _blocks = ROBOT_LANDMARKS);

		void update(const bool _self_auto_diff = true, const bool _apply_loss_function = false);

		void addConstraint(ConstraintBase* _corr_ptr, const bool _self_auto_diff, const bool _apply_loss);

		void removeConstraint(const unsigned int& _corr_idx);

		void addStateBlock(StateBlock* _st_ptr);

		void removeStateBlock(double* _st_ptr);

		void removeAllStateBlocks();

		void updateStateBlockStatus(StateBlock* _st_ptr);

		ceres::CostFunction* createCostFunction(ConstraintBase* _corrPtr, const bool _self_auto_diff);
};

#endif
