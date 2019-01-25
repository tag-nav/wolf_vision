/*
 * sparse_utils.h
 *
 *  Created on: Jul 2, 2015
 *      Author: jvallve
 */

#ifndef SPARSE_UTILS_H_
#define SPARSE_UTILS_H_

// eigen includes
#include <eigen3/Eigen/Sparse>

namespace wolf
{

void eraseBlockRow(Eigen::SparseMatrixs& A, const unsigned int& _row, const unsigned int& _n_rows)
{
    A.prune([](int i, int, Scalar) { return i >= _row && i < _row + _n_rows; });
}

void eraseBlockCol(Eigen::SparseMatrixs& A, const unsigned int& _col, const unsigned int& _n_cols)
{
    A.prune([](int, int j, Scalar) { return j >= _col && j < _col + _n_cols; });
}

void addSparseBlock(const Eigen::MatrixXs& ins, Eigen::SparseMatrixs& original, const unsigned int& row, const unsigned int& col)
{
    for (auto ins_row = 0; ins_row < ins.rows(); ins_row++)
        for (auto ins_col = 0; ins_col < ins.cols(); ins_col++)
            original.coeffRef(row+ins_row, col+ins_col) += ins(ins_row,ins_col);

    original.makeCompressed();
}

void insertSparseBlock(const Eigen::MatrixXs& ins, Eigen::SparseMatrixs& original, const unsigned int& row, const unsigned int& col)
{
    for (auto ins_row = 0; ins_row < ins.rows(); ins_row++)
        for (auto ins_col = 0; ins_col < ins.cols(); ins_col++)
            original.insert(row+ins_row, col+ins_col) = ins(ins_row,ins_col);

    original.makeCompressed();
}

}
#endif /* SPARSE_UTILS_H_ */
