/*
 * qr_manager.h
 *
 *  Created on: Jun 7, 2017
 *      Author: jvallve
 */

#ifndef SRC_CERES_WRAPPER_QR_MANAGER_H_
#define SRC_CERES_WRAPPER_QR_MANAGER_H_

#include "solver_manager.h"
#include "sparse_utils.h"

namespace wolf
{

class QRManager : public SolverManager
{
    protected:
        Eigen::SparseQR<Eigen::SparseMatrixs, Eigen::COLAMDOrdering<int>> solver_;//Eigen::NaturalOrdering<int>
        Eigen::SparseMatrixs A_;
        Eigen::VectorXs b_;
        std::map<StateBlockPtr, unsigned int> sb_2_col_;
        std::map<ConstraintBasePtr, unsigned int> ctr_2_row_;
        bool any_state_block_removed_;
        unsigned int new_state_blocks_;
        unsigned int N_batch_;
        bool pending_changes_;

    public:

        QRManager(ProblemPtr _wolf_problem, const unsigned int& _N_batch);

        virtual ~QRManager();

        virtual std::string solve(const unsigned int& _report_level);

        virtual void computeCovariances(CovarianceBlocksToBeComputed _blocks = ROBOT_LANDMARKS);

        virtual void computeCovariances(const StateBlockList& _sb_list);

    private:

        bool computeDecomposition();

        virtual void addConstraint(ConstraintBasePtr _ctr_ptr);

        virtual void removeConstraint(ConstraintBasePtr _ctr_ptr);

        virtual void addStateBlock(StateBlockPtr _st_ptr);

        virtual void removeStateBlock(StateBlockPtr _st_ptr);

        virtual void updateStateBlockStatus(StateBlockPtr _st_ptr);

        void relinearizeConstraint(ConstraintBasePtr _ctr_ptr);
};

} /* namespace wolf */

#endif /* SRC_CERES_WRAPPER_QR_MANAGER_H_ */
