#ifndef _WOLF_SOLVER_MANAGER_H_
#define _WOLF_SOLVER_MANAGER_H_

//wolf includes
#include "base/wolf.h"
#include "base/state_block.h"
#include "base/constraint/constraint_base.h"

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

  virtual void computeCovariances(const StateBlockList& st_list) = 0;

  virtual void update();

  ProblemPtr getProblemPtr();

protected:

  std::map<StateBlockPtr, Eigen::VectorXs> state_blocks_;

  virtual Eigen::VectorXs& getAssociatedMemBlock(const StateBlockPtr& state_ptr);
  virtual Scalar* getAssociatedMemBlockPtr(const StateBlockPtr& state_ptr);

  virtual std::string solveImpl(const ReportVerbosity report_level) = 0;

  virtual void addConstraint(const ConstraintBasePtr& ctr_ptr) = 0;

  virtual void removeConstraint(const ConstraintBasePtr& ctr_ptr) = 0;

  virtual void addStateBlock(const StateBlockPtr& state_ptr) = 0;

  virtual void removeStateBlock(const StateBlockPtr& state_ptr) = 0;

  virtual void updateStateBlockStatus(const StateBlockPtr& state_ptr) = 0;

  virtual void updateStateBlockLocalParametrization(const StateBlockPtr& state_ptr) = 0;
};

} // namespace wolf

#endif /* _WOLF_SOLVER_MANAGER_H_ */
