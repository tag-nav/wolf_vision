/*
 * qr_manager.cpp
 *
 *  Created on: Jun 7, 2017
 *      Author: jvallve
 */

#include "qr_manager.h"

namespace wolf {

QRManager::QRManager(ProblemPtr _wolf_problem) :
        SolverManager(_wolf_problem),
        A_(), // empty matrix
        b_()
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
    update();

    // Rebuild problem
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

    A_.resize(meas_size,state_size);
    b_.resize(meas_size);

    for (auto ctr_pair : ctr_2_row_)
        relinearizeConstraint(ctr_pair.first);

    // Decomposition
    solver_.compute(A_);
    if (solver_.info() != Eigen::Success)
        return std::string("decomposition failed\n");

    // Solve
    Eigen::VectorXs x_incr = solver_.solve(-b_);
    b_.setZero();

    // update state blocks
    for (auto sb_pair : sb_2_col_)
        sb_pair.first->setState(sb_pair.first->getState() + x_incr.segment(sb_pair.second, sb_pair.first->getSize()));

    if (_report_level == 0)
        return std::string();
    else if (_report_level == 1)
        return std::string("Success!\n");
    else if (_report_level == 2)
        return std::string("Success!\n");
    else
        throw std::runtime_error("bad report level value.");
}

void QRManager::computeCovariances(CovarianceBlocksToBeComputed _blocks)
{
    // TODO
    /*Eigen::SparseMatrixs Lambda_bl = wolf_bl->computeInfoMatrix();
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<Scalar> > solver;
    solver.compute(Lambda_bl.transpose());
    Eigen::SparseMatrix<Scalar> I(Lambda_bl.rows(),Lambda_bl.rows());
    I.setIdentity();
    Eigen::MatrixXs Sigma_bl_full = solver.solve(I);
    Eigen::MatrixXs Sigma_bl2(state_size,state_size);

    for (auto sb_row : sb_remaining)
        for (auto sb_col : sb_remaining)
            Sigma_bl2.block(sb_2_idx_prun[sb_row], sb_2_idx_prun[sb_col], sb_row->getSize(), sb_col->getSize()) = Sigma_bl_full.block(sb_2_idx_full[sb_row], sb_2_idx_full[sb_col], sb_row->getSize(), sb_col->getSize());
    */
}

void QRManager::computeCovariances(const StateBlockList& st_list)
{
    //TODO
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
}


void QRManager::removeConstraint(ConstraintBasePtr _ctr_ptr)
{
    //std::cout << "remove constraint " << _ctr_ptr->id() << std::endl;
    assert(ctr_2_row_.find(_ctr_ptr) != ctr_2_row_.end() && "removing unknown constraint");
    eraseBlockRow(A_, ctr_2_row_[_ctr_ptr], _ctr_ptr->getSize());
    b_.segment(ctr_2_row_[_ctr_ptr], _ctr_ptr->getSize()).setZero();
    ctr_2_row_.erase(_ctr_ptr);
}

void QRManager::addStateBlock(StateBlockPtr _st_ptr)
{
    //std::cout << "add state block " << _st_ptr.get() << std::endl;
    assert(sb_2_col_.find(_st_ptr) == sb_2_col_.end() && "adding existing state block");
    sb_2_col_[_st_ptr] = A_.cols();
    A_.conservativeResize(A_.rows(), A_.cols() + _st_ptr->getSize());
    b_.conservativeResize(b_.size() + _st_ptr->getSize());
}

void QRManager::removeStateBlock(StateBlockPtr _st_ptr)
{
    //std::cout << "remove state block " << _st_ptr.get() << std::endl;
    assert(sb_2_col_.find(_st_ptr) != sb_2_col_.end() && "removing unknown state block");
    eraseBlockCol(A_, sb_2_col_[_st_ptr], _st_ptr->getSize());
    // TODO: insert identity while problem is not re-built?
    sb_2_col_.erase(_st_ptr); // to be erased when problem rebuild?
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
    for (auto i = 0; i < jacobians.size(); i++)
        if (!_ctr_ptr->getStateBlockPtrVector()[i]->isFixed())
        {
            assert(sb_2_col_.find(_ctr_ptr->getStateBlockPtrVector()[i]) != sb_2_col_.end() && "constraint involving a state block not added");
            assert(A_.cols() >= sb_2_col_[_ctr_ptr->getStateBlockPtrVector()[i]] + jacobians[i].cols() - 1 && "bad A number of cols");
            // insert since this row have been created few lines before and it's empty for sure
            insertSparseBlock(jacobians[i], A_, ctr_2_row_[_ctr_ptr], sb_2_col_[_ctr_ptr->getStateBlockPtrVector()[i]]);
        }

    // Fill residual
    b_.segment(ctr_2_row_[_ctr_ptr], _ctr_ptr->getSize()) = residual;
}

} /* namespace wolf */
