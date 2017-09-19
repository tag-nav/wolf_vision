/*
 * qr_manager.cpp
 *
 *  Created on: Jun 7, 2017
 *      Author: jvallve
 */

#include "qr_manager.h"

namespace wolf {

QRManager::QRManager(ProblemPtr _wolf_problem, const unsigned int& _N_batch) :
        SolverManager(_wolf_problem),
        A_(), // empty matrix
        b_(),
        any_state_block_removed_(false),
        new_state_blocks_(0),
        N_batch_(_N_batch),
        pending_changes_(false)
{
    //
}

QRManager::~QRManager()
{
    sb_2_col_.clear();
    ctr_2_row_.clear();
}

std::string QRManager::solve(const unsigned int& _report_level)
{
    // check for update notifications
    update();

    // Decomposition
    if (!computeDecomposition())
        return std::string("decomposition failed\n");

    // Solve
    Eigen::VectorXs x_incr = solver_.solve(-b_);
    b_.setZero();

    // update state blocks
    for (auto sb_pair : sb_2_col_)
        sb_pair.first->setState(sb_pair.first->getState() + x_incr.segment(sb_pair.second, sb_pair.first->getSize()));

    if (_report_level == 1)
        return std::string("Success!\n");
    else if (_report_level == 2)
        return std::string("Success!\n");

    return std::string();
}

void QRManager::computeCovariances(CovarianceBlocksToBeComputed _blocks)
{
    // TODO
}

void QRManager::computeCovariances(const StateBlockList& _sb_list)
{
    //std::cout << "computing covariances.." << std::endl;
    update();
    computeDecomposition();

//    std::cout << "R is " << solver_.matrixR().rows() << "x" << solver_.matrixR().cols() << std::endl;
//    std::cout << Eigen::MatrixXs(solver_.matrixR()) << std::endl;

    covariance_solver_.compute(solver_.matrixR().topRows(solver_.matrixR().cols()));

    Eigen::SparseMatrix<Scalar, Eigen::ColMajor> I(A_.cols(),A_.cols());
    I.setIdentity();
    Eigen::SparseMatrix<Scalar, Eigen::ColMajor> iR = covariance_solver_.solve(I);
    Eigen::MatrixXs Sigma_full = solver_.colsPermutation() * iR * iR.transpose() * solver_.colsPermutation().transpose();

//    std::cout << "A' A = \n" << Eigen::MatrixXs(A_.transpose() * A_)<< std::endl;
//    std::cout << "P iR iR' P' = \n" << Eigen::MatrixXs(P * iR * iR.transpose() * P.transpose()) << std::endl;
//    std::cout << "Sigma * Lambda = \n" << Eigen::MatrixXs(Sigma_full * A_.transpose() * A_) << std::endl;
//    std::cout << "Permutation: \n" << solver_.colsPermutation() << std::endl;
//    std::cout << "Sigma = \n" << Sigma_full << std::endl;

    // STORE DESIRED COVARIANCES
    for (auto sb_row = _sb_list.begin(); sb_row != _sb_list.end(); sb_row++)
        for (auto sb_col = sb_row; sb_col!=_sb_list.end(); sb_col++)
        {
            //std::cout << "cov state block " << sb_col->get() << std::endl;
            assert(sb_2_col_.find(*sb_col) != sb_2_col_.end() && "state block not found");
            //std::cout << "block: " << sb_2_col_[*sb_row] << "," << sb_2_col_[*sb_col] << std::endl << Sigma_full.block(sb_2_col_[*sb_row], sb_2_col_[*sb_col], (*sb_row)->getSize(), (*sb_col)->getSize()) << std::endl;
            wolf_problem_->addCovarianceBlock(*sb_row, *sb_col, Sigma_full.block(sb_2_col_[*sb_row], sb_2_col_[*sb_col], (*sb_row)->getSize(), (*sb_col)->getSize()));
        }
}

bool QRManager::computeDecomposition()
{
    if (pending_changes_)
    {
        // Rebuild problem
        if (any_state_block_removed_)
        {
            // rebuild maps
            unsigned int state_size = 0;
            for (auto sb_pair : sb_2_col_)
            {
                sb_2_col_[sb_pair.first] = state_size;
                state_size += sb_pair.first->getSize();
            }

            unsigned int meas_size = 0;
            for (auto ctr_pair : ctr_2_row_)
            {
                ctr_2_row_[ctr_pair.first] = meas_size;
                meas_size += ctr_pair.first->getSize();
            }

            // resize and setZero A, b
            A_.resize(meas_size,state_size);
            b_.resize(meas_size);
        }

        if (any_state_block_removed_ || new_state_blocks_ >= N_batch_)
        {
            // relinearize all constraints
            for (auto ctr_pair : ctr_2_row_)
                relinearizeConstraint(ctr_pair.first);

            any_state_block_removed_ = false;
            new_state_blocks_ = 0;
        }

        // Decomposition
        solver_.compute(A_);
        if (solver_.info() != Eigen::Success)
            return false;
    }

    pending_changes_ = false;

    return true;
}

void QRManager::addConstraint(ConstraintBasePtr _ctr_ptr)
{
    //std::cout << "add constraint " << _ctr_ptr->id() << std::endl;
    assert(ctr_2_row_.find(_ctr_ptr) == ctr_2_row_.end() && "adding existing constraint");
    ctr_2_row_[_ctr_ptr] = A_.rows();
    A_.conservativeResize(A_.rows() + _ctr_ptr->getSize(), A_.cols());
    b_.conservativeResize(b_.size() + _ctr_ptr->getSize());

    assert(A_.rows() >= ctr_2_row_[_ctr_ptr] + _ctr_ptr->getSize() - 1 && "bad A number of rows");
    assert(b_.rows() >= ctr_2_row_[_ctr_ptr] + _ctr_ptr->getSize() - 1 && "bad b number of rows");

    relinearizeConstraint(_ctr_ptr);

    pending_changes_ = true;
}


void QRManager::removeConstraint(ConstraintBasePtr _ctr_ptr)
{
    //std::cout << "remove constraint " << _ctr_ptr->id() << std::endl;
    assert(ctr_2_row_.find(_ctr_ptr) != ctr_2_row_.end() && "removing unknown constraint");
    eraseBlockRow(A_, ctr_2_row_[_ctr_ptr], _ctr_ptr->getSize());
    b_.segment(ctr_2_row_[_ctr_ptr], _ctr_ptr->getSize()).setZero();
    ctr_2_row_.erase(_ctr_ptr);
    pending_changes_ = true;
}

void QRManager::addStateBlock(StateBlockPtr _st_ptr)
{
    //std::cout << "add state block " << _st_ptr.get() << std::endl;
    assert(sb_2_col_.find(_st_ptr) == sb_2_col_.end() && "adding existing state block");
    sb_2_col_[_st_ptr] = A_.cols();
    A_.conservativeResize(A_.rows(), A_.cols() + _st_ptr->getSize());

    new_state_blocks_++;
    pending_changes_ = true;
}

void QRManager::removeStateBlock(StateBlockPtr _st_ptr)
{
    //std::cout << "remove state block " << _st_ptr.get() << std::endl;
    assert(sb_2_col_.find(_st_ptr) != sb_2_col_.end() && "removing unknown state block");
    eraseBlockCol(A_, sb_2_col_[_st_ptr], _st_ptr->getSize());

    // flag to rebuild problem
    any_state_block_removed_ = true;
    // TODO: insert identity while problem is not re-built?

    sb_2_col_.erase(_st_ptr);
    pending_changes_ = true;
}

void QRManager::updateStateBlockStatus(StateBlockPtr _st_ptr)
{
    //std::cout << "update state block " << _st_ptr.get() << std::endl;
    if (_st_ptr->isFixed())
    {
        if (sb_2_col_.find(_st_ptr) != sb_2_col_.end())
            removeStateBlock(_st_ptr);
    }
    else
        if (sb_2_col_.find(_st_ptr) == sb_2_col_.end())
            addStateBlock(_st_ptr);
}

void QRManager::relinearizeConstraint(ConstraintBasePtr _ctr_ptr)
{
    // evaluate constraint
    std::vector<const Scalar*> ctr_states_ptr;
    for (auto sb : _ctr_ptr->getStateBlockPtrVector())
        ctr_states_ptr.push_back(sb->getPtr());
    Eigen::VectorXs residual(_ctr_ptr->getSize());
    std::vector<Eigen::MatrixXs> jacobians;
    _ctr_ptr->evaluate(ctr_states_ptr,residual,jacobians);

    // Fill jacobians
    Eigen::SparseMatrixs A_block_row(_ctr_ptr->getSize(), A_.cols());
    for (auto i = 0; i < jacobians.size(); i++)
        if (!_ctr_ptr->getStateBlockPtrVector()[i]->isFixed())
        {
            assert(sb_2_col_.find(_ctr_ptr->getStateBlockPtrVector()[i]) != sb_2_col_.end() && "constraint involving a state block not added");
            assert(A_.cols() >= sb_2_col_[_ctr_ptr->getStateBlockPtrVector()[i]] + jacobians[i].cols() - 1 && "bad A number of cols");
            // insert since A_block_row has just been created so it's empty for sure
            insertSparseBlock(jacobians[i], A_block_row, 0, sb_2_col_[_ctr_ptr->getStateBlockPtrVector()[i]]);
        }
    assignBlockRow(A_, A_block_row, ctr_2_row_[_ctr_ptr]);

    // Fill residual
    b_.segment(ctr_2_row_[_ctr_ptr], _ctr_ptr->getSize()) = residual;
}

} /* namespace wolf */
