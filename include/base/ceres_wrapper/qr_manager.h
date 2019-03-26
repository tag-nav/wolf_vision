/*
 * qr_manager.h
 *
 *  Created on: Jun 7, 2017
 *      Author: jvallve
 */

#ifndef SRC_CERES_WRAPPER_QR_MANAGER_H_
#define SRC_CERES_WRAPPER_QR_MANAGER_H_

#include "base/solver/solver_manager.h"
#include "base/solver_suitesparse/sparse_utils.h"

namespace wolf
{

class QRManager : public SolverManager
{
    protected:
        Eigen::SparseQR<Eigen::SparseMatrixs, Eigen::COLAMDOrdering<int>> solver_;
        Eigen::SparseQR<Eigen::SparseMatrixs, Eigen::NaturalOrdering<int>> covariance_solver_;
        Eigen::SparseMatrixs A_;
        Eigen::VectorXs b_;
        std::map<StateBlockPtr, unsigned int> sb_2_col_;
        std::map<FactorBasePtr, unsigned int> ctr_2_row_;
        bool any_state_block_removed_;
        unsigned int new_state_blocks_;
        unsigned int N_batch_;
        bool pending_changes_;

    public:

        QRManager(ProblemPtr _wolf_problem, const unsigned int& _N_batch);

        virtual ~QRManager();

        virtual std::string solve(const unsigned int& _report_level);

        virtual void computeCovariances(CovarianceBlocksToBeComputed _blocks = ROBOT_LANDMARKS);

        virtual void computeCovariances(const StateBlockPtrList& _sb_list);

    private:

        bool computeDecomposition();

        virtual void addFactor(FactorBasePtr _ctr_ptr);

        virtual void removeFactor(FactorBasePtr _ctr_ptr);

        virtual void addStateBlock(StateBlockPtr _st_ptr);

        virtual void removeStateBlock(StateBlockPtr _st_ptr);

        virtual void updateStateBlockStatus(StateBlockPtr _st_ptr);

        void relinearizeFactor(FactorBasePtr _ctr_ptr);
};

} /* namespace wolf */

#endif /* SRC_CERES_WRAPPER_QR_MANAGER_H_ */
