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
        A_(0, 0)
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
    //TODO
    return std::string(0);
}

void QRManager::computeCovariances(CovarianceBlocksToBeComputed _blocks)
{
    // TODO
}

void QRManager::computeCovariances(const StateBlockList& st_list)
{
    //TODO
}

void QRManager::addConstraint(ConstraintBasePtr _ctr_ptr)
{
    assert(ctr_2_row_.find(_ctr_ptr) == ctr_2_row_.end() && "adding existing constraint");
    ctr_2_row_[_ctr_ptr] = A_.rows();
    A_.conservativeResize(A_.rows() + _ctr_ptr->getSize(), A_.cols());
    // TODO: add Jacobians to A
    // TODO: assert state blocks in sb_2_cols
}

void QRManager::removeConstraint(ConstraintBasePtr _ctr_ptr)
{
    assert(ctr_2_row_.find(_ctr_ptr) != ctr_2_row_.end() && "removing unknown constraint");
    ctr_2_row_.erase(_ctr_ptr);
    eraseBlockRow(A_, ctr_2_row_[_ctr_ptr], _ctr_ptr->getSize());
}

void QRManager::addStateBlock(StateBlockPtr _st_ptr)
{
    assert(sb_2_col_.find(_st_ptr) == sb_2_col_.end() && "adding existing state block");
    if (!_st_ptr->isFixed())
    {
        sb_2_col_[_st_ptr] = A_.cols();
        A_.conservativeResize(A_.rows(), A_.cols() + _st_ptr->getSize());
    }
}

void QRManager::removeStateBlock(StateBlockPtr _st_ptr)
{
    assert(sb_2_col_.find(_st_ptr) != sb_2_col_.end() && "removing unknown state block");
    sb_2_col_.erase(_st_ptr);
    eraseBlockCol(A_, sb_2_col_[_st_ptr], _st_ptr->getSize());
    // TODO: insert identity while problem is not re-built?
}

void QRManager::updateStateBlockStatus(StateBlockPtr _st_ptr)
{
    if (_st_ptr->isFixed())
        removeStateBlock(_st_ptr);
    else
        addStateBlock(_st_ptr);
}

} /* namespace wolf */
