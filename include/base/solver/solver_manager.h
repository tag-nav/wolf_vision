#ifndef _WOLF_SOLVER_MANAGER_H_
#define _WOLF_SOLVER_MANAGER_H_

//wolf includes
#include "base/wolf.h"
#include "base/state_block.h"
#include "base/factor/factor_base.h"

namespace wolf {

WOLF_PTR_TYPEDEFS(SolverManager)

/**
 * \brief Solver manager for WOLF
 */
class SolverManager
{
public:

  /** \brief Enumeration of covariance blocks to be computed
   *
   * Enumeration of covariance blocks to be computed
   *
   */
  enum class CovarianceBlocksToBeComputed : std::size_t
  {
    ALL, ///< All blocks and all cross-covariances
    ALL_MARGINALS, ///< All marginals
    ROBOT_LANDMARKS ///< marginals of landmarks and current robot pose plus cross covariances of current robot and all landmarks
  };

  /**
   * \brief Enumeration for the verbosity of the solver report.
   */
  enum class ReportVerbosity : std::size_t
  {
    QUIET = 0,
    BRIEF,
    FULL
  };

protected:

  ProblemPtr wolf_problem_;

public:

  SolverManager(const ProblemPtr& wolf_problem);

  virtual ~SolverManager() = default;

  std::string solve(const ReportVerbosity report_level = ReportVerbosity::QUIET);

  virtual void computeCovariances(const CovarianceBlocksToBeComputed blocks) = 0;

  virtual void computeCovariances(const StateBlockPtrList& st_list) = 0;

  virtual bool hasConverged() = 0;

  virtual SizeStd iterations() = 0;

  virtual Scalar initialCost() = 0;

  virtual Scalar finalCost() = 0;

  virtual void update();

  ProblemPtr getProblem();

protected:

  std::map<StateBlockPtr, Eigen::VectorXs> state_blocks_;

  virtual Eigen::VectorXs& getAssociatedMemBlock(const StateBlockPtr& state_ptr);
  virtual Scalar* getAssociatedMemBlockPtr(const StateBlockPtr& state_ptr);

  virtual std::string solveImpl(const ReportVerbosity report_level) = 0;

  virtual void addFactor(const FactorBasePtr& fac_ptr) = 0;

  virtual void removeFactor(const FactorBasePtr& fac_ptr) = 0;

  virtual void addStateBlock(const StateBlockPtr& state_ptr) = 0;

  virtual void removeStateBlock(const StateBlockPtr& state_ptr) = 0;

  virtual void updateStateBlockStatus(const StateBlockPtr& state_ptr) = 0;

  virtual void updateStateBlockLocalParametrization(const StateBlockPtr& state_ptr) = 0;
};

} // namespace wolf

#endif /* _WOLF_SOLVER_MANAGER_H_ */
