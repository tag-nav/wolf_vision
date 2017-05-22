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

WOLF_PTR_TYPEDEFS(CeresManager);

/** \brief Ceres manager for WOLF
 *
 */

class CeresManager
{
	protected:
		std::map<unsigned int, ceres::ResidualBlockId> id_2_residual_idx_;
        std::map<unsigned int, ceres::CostFunction*> id_2_costfunction_;
		ceres::Problem* ceres_problem_;
		ceres::Solver::Options ceres_options_;
		ceres::Covariance* covariance_;
		ProblemPtr wolf_problem_;
		bool use_wolf_auto_diff_;

	public:
        CeresManager(ProblemPtr _wolf_problem, const ceres::Solver::Options& _ceres_options = ceres::Solver::Options(), const bool _use_wolf_auto_diff = true);

		~CeresManager();

		ceres::Solver::Summary solve();

		void computeCovariances(CovarianceBlocksToBeComputed _blocks = ROBOT_LANDMARKS);

        ceres::Solver::Options& getSolverOptions();

        void setUseWolfAutoDiff(bool _use_wolf_auto_diff);

        std::vector<Eigen::MatrixXs> computeJacobian(const unsigned int& _ctr_id, const std::vector<Scalar*>& _states) const;

	private:

		void update();

		void addConstraint(ConstraintBasePtr _corr_ptr, unsigned int _id);

		void removeConstraint(const unsigned int& _corr_idx);

		void addStateBlock(StateBlockPtr _st_ptr);

		void removeStateBlock(double* _st_ptr);

		void removeAllStateBlocks();

		void updateStateBlockStatus(StateBlockPtr _st_ptr);

		ceres::CostFunction* createCostFunction(ConstraintBasePtr _corrPtr);
};

inline ceres::Solver::Options& CeresManager::getSolverOptions()
{
    return ceres_options_;
}

inline void CeresManager::setUseWolfAutoDiff(bool _use_wolf_auto_diff)
{
    use_wolf_auto_diff_ = _use_wolf_auto_diff;
}

} // namespace wolf

#endif
