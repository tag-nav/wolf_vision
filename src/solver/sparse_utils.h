/*
 * sparse_utils.h
 *
 *  Created on: Jul 2, 2015
 *      Author: jvallve
 */

#ifndef TRUNK_SRC_SOLVER_SPARSE_UTILS_H_
#define TRUNK_SRC_SOLVER_SPARSE_UTILS_H_

// eigen includes
#include <eigen3/Eigen/Sparse>

class SparseBlockPruning
{
    public:
        unsigned int col, row, Nrows, Ncols;
        SparseBlockPruning(unsigned int _col, unsigned int _row, unsigned int _Nrows, unsigned int _Ncols) :
                col(_col),
                row(_row),
                Nrows(_Nrows),
                Ncols(_Ncols)
        {
            //
        }
        bool operator()(unsigned int i, unsigned int j, double) const
        {
            return (i < row || i > row + Nrows-1) || (j < col || j > col + Ncols-1);
        }
};

void eraseSparseBlock(Eigen::SparseMatrix<double>& original, const unsigned int& row, const unsigned int& col, const unsigned int& Nrows, const unsigned int& Ncols)
{
    // prune all non-zero elements that not satisfy the 'keep' operand
    // elements that are not in the block rows or are not in the block columns should be kept

    SparseBlockPruning bp(row, col, Nrows, Ncols);
    original.prune(bp);
}

void addSparseBlock(const Eigen::MatrixXd& ins, Eigen::SparseMatrix<double>& original, const unsigned int& row, const unsigned int& col)
{
  for (unsigned int r=0; r<ins.rows(); ++r)
      for (unsigned int c = 0; c < ins.cols(); c++)
          if (ins(r,c) != 0)
              original.coeffRef(r + row, c + col) += ins(r,c);
}

#endif /* TRUNK_SRC_SOLVER_SPARSE_UTILS_H_ */
