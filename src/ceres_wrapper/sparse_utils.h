/*
 * sparse_utils.h
 *
 *  Created on: Jul 2, 2015
 *      Author: jvallve
 */

#ifndef SPARSE_UTILS_H_
#define SPARSE_UTILS_H_

// eigen includes
//#include <eigen3/Eigen/Sparse>

namespace wolf
{

void eraseBlockRow(Eigen::SparseMatrix<Scalar, Eigen::RowMajor>& A, const unsigned int& _row, const unsigned int& _n_rows)
{
    A.middleRows(_row,_n_rows) = Eigen::SparseMatrixs(_n_rows,A.cols());
}

void eraseBlockRow(Eigen::SparseMatrix<Scalar, Eigen::ColMajor>& A, const unsigned int& _row, const unsigned int& _n_rows)
{
    A.prune([&](int i, int, Scalar) { return i >= _row && i < _row + _n_rows; });
}

void eraseBlockCol(Eigen::SparseMatrix<Scalar, Eigen::ColMajor>& A, const unsigned int& _col, const unsigned int& _n_cols)
{
    A.middleCols(_col,_n_cols) = Eigen::SparseMatrixs(A.rows(),_n_cols);
}

void eraseBlockCol(Eigen::SparseMatrix<Scalar, Eigen::RowMajor>& A, const unsigned int& _col, const unsigned int& _n_cols)
{
    A.prune([&](int, int j, Scalar) { return j >= _col && j < _col + _n_cols; });
}

template<int _Options, typename _StorageIndex>
void addSparseBlock(const Eigen::MatrixXs& ins, Eigen::SparseMatrix<Scalar,_Options,_StorageIndex>& original, const unsigned int& row, const unsigned int& col)
{
    for (auto ins_row = 0; ins_row < ins.rows(); ins_row++)
        for (auto ins_col = 0; ins_col < ins.cols(); ins_col++)
            original.coeffRef(row+ins_row, col+ins_col) += ins(ins_row,ins_col);

    original.makeCompressed();
}

template<int _Options, typename _StorageIndex>
void insertSparseBlock(const Eigen::MatrixXs& ins, Eigen::SparseMatrix<Scalar,_Options,_StorageIndex>& original, const unsigned int& row, const unsigned int& col)
{
    for (auto ins_row = 0; ins_row < ins.rows(); ins_row++)
        for (auto ins_col = 0; ins_col < ins.cols(); ins_col++)
            original.insert(row+ins_row, col+ins_col) = ins(ins_row,ins_col);

    original.makeCompressed();
}

void assignBlockRow(Eigen::SparseMatrix<Scalar, Eigen::RowMajor>& A, const Eigen::SparseMatrix<Scalar, Eigen::RowMajor>& ins, const unsigned int& _row)
{
    assert(A.rows() >= _row + ins.rows() && A.cols() == ins.cols());
    A.middleRows(_row, ins.rows()) = ins;
}

Eigen::SparseMatrixs createBlockDiagonal(const std::vector<Eigen::MatrixXs>& _diag_blocs)
{
    unsigned int dim = _diag_blocs.front().rows();
    unsigned int size = dim * _diag_blocs.size();

    Eigen::SparseMatrixs M(size,size);

    unsigned int pos = 0;
    for (const Eigen::MatrixXs& omega_k : _diag_blocs)
    {
        insertSparseBlock(omega_k, M, pos, pos);
        pos += dim;
    }

    return M;
}

template<int _Options, typename _StorageIndex>
void getDiagonalBlocks(const Eigen::SparseMatrix<Scalar,_Options,_StorageIndex>& _M, std::vector<Eigen::MatrixXs>& diag_, const unsigned int& dim)
{
    assert(_M.rows() % dim == 0 && "number of rows must be multiple of dimension");
    diag_.clear();
    for (auto i = 0; i < _M.rows(); i += dim)
        diag_.push_back(Eigen::MatrixXs(_M.block(i,i,dim,dim)));
}

} // namespace wolf
#endif /* SPARSE_UTILS_H_ */
