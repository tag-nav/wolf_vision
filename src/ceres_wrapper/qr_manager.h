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
        Eigen::SparseQR<Eigen::SparseMatrixs, Eigen::NaturalOrdering<int>> solver_;
        Eigen::SparseMatrixs A_;
        std::map<StateBlockPtr, unsigned int> sb_2_col_;
        std::map<ConstraintBasePtr, unsigned int> ctr_2_row_;

    public:

        QRManager(ProblemPtr _wolf_problem);

        virtual ~QRManager();

        virtual std::string solve(const unsigned int& _report_level);

        virtual void computeCovariances(CovarianceBlocksToBeComputed _blocks = ROBOT_LANDMARKS);

        virtual void computeCovariances(const StateBlockList& st_list);

    private:

        virtual void addConstraint(ConstraintBasePtr _ctr_ptr);

        virtual void removeConstraint(ConstraintBasePtr _ctr_ptr);

        virtual void addStateBlock(StateBlockPtr _st_ptr);

        virtual void removeStateBlock(StateBlockPtr _st_ptr);

        virtual void updateStateBlockStatus(StateBlockPtr _st_ptr);
};

} /* namespace wolf */

#endif /* SRC_CERES_WRAPPER_QR_MANAGER_H_ */
