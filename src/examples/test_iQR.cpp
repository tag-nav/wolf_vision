/*
 * test_iQR.cpp
 *
 *  Created on: Jun 17, 2015
 *      Author: jvallve
 */

/*
 * test_ccolamd_blocks.cpp
 *
 *  Created on: Jun 12, 2015
 *      Author: jvallve
 */


//std includes
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <memory>
#include <random>
#include <typeinfo>
#include <ctime>
#include <queue>

// eigen includes
#include <eigen3/Eigen/OrderingMethods>
#include <eigen3/Eigen/SparseQR>
#include <Eigen/SPQRSupport>

// ccolamd
#include "solver/ccolamd_ordering.h"

using namespace Eigen;

void erase_sparse_block(SparseMatrix<double>& original, const unsigned int& row, const unsigned int& Nrows, const unsigned int& col, const unsigned int& Ncols)
{
  for (uint i = row; i < row + Nrows; i++)
    for (uint j = col; j < row + Ncols; j++)
      original.coeffRef(i,j) = 0.0;

  original.makeCompressed();
}

void add_sparse_block(const MatrixXd& ins, SparseMatrix<double>& original, const unsigned int& row, const unsigned int& col)
{
  for (uint r=0; r<ins.rows(); ++r)
      for (uint c = 0; c < ins.cols(); c++)
          if (ins(r,c) != 0)
              original.coeffRef(r + row, c + col) += ins(r,c);
}

//void add_sparse_row(const MatrixXd& ins, SparseMatrix<double>& original, const unsigned int& row, const unsigned int& col)
//{
//  for (uint r=0; r<ins.rows(); ++r)
//      for (uint c = 0; c < ins.cols(); c++)
//          if (ins(r,c) != 0)
//              original.coeffRef(r + row, c + col) += ins(r,c);
//}

void permutation_2_block_permutation(const PermutationMatrix<Dynamic, Dynamic, int> &perm_nodes, PermutationMatrix<Dynamic, Dynamic, int> &perm_variables, const int dim)
{
    ArrayXXi idx(dim, perm_nodes.indices().rows());
    idx.row(0) = dim * perm_nodes.indices().transpose();

    for (unsigned int i = 1; i<dim; i++)
        idx.row(i) = idx.row(i-1) + 1;
    Map<ArrayXi> idx_blocks(idx.data(), dim*perm_nodes.indices().rows(), 1);
    perm_variables.indices() = idx_blocks;
}

void augment_permutation(PermutationMatrix<Dynamic, Dynamic, int> &perm, const int new_size)
{
    int old_size = perm.indices().size();
    int dim = new_size - old_size;
    VectorXi new_indices(new_size);
    new_indices.head(old_size)= perm.indices();
    new_indices.tail(dim) = ArrayXi::LinSpaced(dim, old_size, new_size-1);
    perm.resize(new_size);
    perm.indices() = new_indices;
}


//main
int main(int argc, char *argv[])
{
    if (argc != 3 || atoi(argv[1]) < 1|| atoi(argv[2]) < 1)
    {
        std::cout << "Please call me with: [./test_iQR SIZE DIM], where:" << std::endl;
        std::cout << "     - SIZE: integer size of the problem" << std::endl;
        std::cout << "     - DIM: integer dimension of the nodes" << std::endl;
        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
        return -1;
    }
    int size = atoi(argv[1]);
    int dim = atoi(argv[2]);

    // Problem variables
    SparseQR < SparseMatrix<double>, NaturalOrdering<int>> solver, solver2;

    MatrixXd omega = MatrixXd::Constant(dim, dim, 0.1) + MatrixXd::Identity(dim, dim);
    SparseMatrix<double> A(0,0),
                         A_ordered(0,0);
    VectorXd b,
             b_ordered,
             x_ordered,
             x;
    int n_measurements = 0;
    int n_nodes = 0;

    // ordering variables
    //SparseMatrix<int> A_nodes(0,0);
    SparseMatrix<int> A_nodes_ordered(0,0);
    PermutationMatrix<Dynamic, Dynamic, int> acc_permutation_nodes_matrix(0);

    CCOLAMDOrdering<int> ordering, partial_ordering;
    VectorXi nodes_ordering_constraints;

    // results variables
    clock_t t1, t2, t3, t4;
    double time1=0, time2=0, time3=0, time4=0;

    std::cout << "STARTING INCREMENTAL QR TEST" << std::endl << std::endl;

    // GENERATING MEASUREMENTS
    std::vector<std::vector<int>> measurements;
    for (unsigned int i = 0; i < size; i++)
    {
        std::vector<int> meas(0);
        if (i == 0) //prior
        {
            meas.push_back(0);
            measurements.push_back(meas);
            meas.clear();
        }
        else //odometry
        {
            meas.push_back(i-1);
            meas.push_back(i);
            measurements.push_back(meas);
            meas.clear();
        }
        if (i > size / 2) // loop closures
        {
            meas.push_back(0);
            meas.push_back(i);
            measurements.push_back(meas);
            meas.clear();
        }
    }

    // INCREMENTAL LOOP
    for (unsigned int i = 0; i < measurements.size(); i++)
    {
        std::cout << "========================= MEASUREMENT " << i << ":" << std::endl;
        std::vector<int> measurement = measurements.at(i);

        // AUGMENT THE PROBLEM ----------------------------
        n_measurements++;
        while (n_nodes < measurement.back()+1)
        {
            n_nodes++;
            // Resize accumulated permutations
            augment_permutation(acc_permutation_nodes_matrix, n_nodes);

            // Resize state
            x.conservativeResize(n_nodes*dim);
            x_ordered.conservativeResize(n_nodes*dim);
        }
        A.conservativeResize(n_measurements*dim,n_nodes*dim);
        A_ordered.conservativeResize(n_measurements*dim,n_nodes*dim);
        b.conservativeResize(n_measurements*dim);
        //A_nodes.conservativeResize(n_measurements,n_nodes);
        A_nodes_ordered.conservativeResize(n_measurements,n_nodes);

        // ADD MEASUREMENTS
        int min_ordered_node = n_nodes;
        for (unsigned int j = 0; j < measurement.size(); j++)
        {
            int ordered_node = acc_permutation_nodes_matrix.indices()(measurement.at(j));

            add_sparse_block(2*omega, A, A.rows()-dim, measurement.at(j) * dim);
            add_sparse_block(2*omega, A_ordered, A_ordered.rows()-dim, ordered_node * dim);

            //A_nodes.coeffRef(A_nodes.rows()-1, measurement.at(j)) = 1;
            A_nodes_ordered.coeffRef(A_nodes_ordered.rows()-1, ordered_node) = 1;

            b.segment(b.size() - dim, dim) = VectorXd::LinSpaced(dim, b.size()-dim, b.size()-1);
            // store minimum ordered node
            if (min_ordered_node > ordered_node)
                min_ordered_node = ordered_node;
        }

//        std::cout << "Solving Ax = b" << std::endl;
//        std::cout << "A_nodes: " << std::endl << MatrixXi::Identity(A_nodes.rows(), A_nodes.rows()) * A_nodes  << std::endl << std::endl;
//        std::cout << "A_nodes_ordered: " << std::endl << MatrixXi::Identity(A_nodes_ordered.rows(), A_nodes_ordered.rows()) * A_nodes_ordered  << std::endl << std::endl;
//        std::cout << "A: " << std::endl << MatrixXd::Identity(A.rows(), A.rows()) * A  << std::endl << std::endl;
//        std::cout << "A_ordered: " << std::endl << MatrixXd::Identity(A.rows(), A.rows()) * A_ordered  << std::endl << std::endl;
//        std::cout << "b: " << std::endl << b.transpose() << std::endl << std::endl;

        // BLOCK REORDERING ------------------------------------
        t1 = clock();
        int subproblem_nodes = n_nodes - min_ordered_node;
        if (n_nodes > 1 && subproblem_nodes > 2) // only reordering when involved nodes in the measurement are not the two last ones
        {
            // SUBPROBLEM ORDERING (from first node variable to last one)
            std::cout << "ordering partial problem: " << min_ordered_node << " to "<< n_nodes - 1 << std::endl;
            t3 = clock();
            SparseMatrix<int> sub_A_nodes_ordered = A_nodes_ordered.rightCols(subproblem_nodes);

            // partial ordering constraints
            VectorXi nodes_partial_ordering_constraints = sub_A_nodes_ordered.bottomRows(1).transpose();

            // computing nodes partial ordering
            A_nodes_ordered.makeCompressed();
            PermutationMatrix<Dynamic, Dynamic, int> partial_permutation_nodes_matrix(subproblem_nodes);
            partial_ordering(sub_A_nodes_ordered, partial_permutation_nodes_matrix, nodes_partial_ordering_constraints.data());

            // node ordering to variable ordering
            PermutationMatrix<Dynamic, Dynamic, int> partial_permutation_matrix(A_ordered.cols());
            permutation_2_block_permutation(partial_permutation_nodes_matrix, partial_permutation_matrix , dim);

            // apply partial orderings
            A_nodes_ordered.rightCols(subproblem_nodes) = (A_nodes_ordered.rightCols(subproblem_nodes) * partial_permutation_nodes_matrix.transpose()).sparseView();
            A_ordered.rightCols(subproblem_nodes * dim) = (A_ordered.rightCols(subproblem_nodes * dim) * partial_permutation_matrix.transpose()).sparseView();

            // ACCUMULATING PERMUTATIONS
            PermutationMatrix<Dynamic, Dynamic, int> permutation_nodes_matrix(VectorXi::LinSpaced(n_nodes, 0, n_nodes - 1)); // identity permutation
            permutation_nodes_matrix.indices().tail(subproblem_nodes) = partial_permutation_nodes_matrix.indices() + VectorXi::Constant(subproblem_nodes, n_nodes - subproblem_nodes);
            acc_permutation_nodes_matrix = permutation_nodes_matrix * acc_permutation_nodes_matrix;

            time3 += ((double) clock() - t3) / CLOCKS_PER_SEC;

//            std::cout << "partial_permutation_nodes_matrix: " << std::endl << partial_permutation_nodes_matrix.indices().transpose()  << std::endl << std::endl;
//            std::cout << "permutation_nodes_matrix:    " << std::endl << permutation_nodes_matrix.indices().transpose()  << std::endl << std::endl;

            //std::cout << "A_nodes_ordered  " << std::endl << MatrixXi::Identity(A_nodes_ordered.rows(), A_nodes_ordered.rows()) * A_nodes_ordered  << std::endl << std::endl;

        }
//        std::cout << "incrementally ordered A Block structure: " << std::endl << MatrixXi::Identity(A_nodes_ordered.rows(), A_nodes_ordered.rows()) * A_nodes_ordered  << std::endl << std::endl;

        //std::cout << "A: " << std::endl << MatrixXd::Identity(A.rows(), A.rows()) * A << std::endl << std::endl;
        //std::cout << "ordered A: " << std::endl << MatrixXd::Identity(A.rows(), A.rows()) * A_ordered << std::endl << std::endl;
        //std::cout << "b: " << std::endl << b.transpose() << std::endl << std::endl;

        // solving
        A_nodes_ordered.makeCompressed();
        A_ordered.makeCompressed();
        solver.compute(A_ordered);
        if (solver.info() != Success)
        {
            std::cout << "decomposition failed" << std::endl;
            return 0;
        }
        //std::cout << "R: " << std::endl << solver.matrixR() << std::endl;
        x_ordered = solver.solve(b);
        PermutationMatrix<Dynamic, Dynamic, int> acc_permutation_matrix(A_ordered.cols());
        permutation_2_block_permutation(acc_permutation_nodes_matrix, acc_permutation_matrix , dim);
        x_ordered = acc_permutation_matrix.inverse() * x_ordered;
        time1 += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // WITHOUT ORDERING
        t2 = clock();
        A.makeCompressed();
        solver2.compute(A);
        if (solver2.info() != Success)
        {
            std::cout << "decomposition failed" << std::endl;
            return 0;
        }
        //std::cout << "no ordering? " << solver2.colsPermutation().indices().transpose() << std::endl;
        x = solver2.solve(b);
        time2 += ((double) clock() - t2) / CLOCKS_PER_SEC;

        // RESULTS ------------------------------------
        std::cout << "========================= RESULTS " << i << ":" << std::endl;
        std::cout << "NO REORDERING:    solved in " << time2*1e3 << " ms | " << solver2.matrixR().nonZeros() << " nonzeros in R"<< std::endl;
        std::cout << "BLOCK REORDERING: solved in " << time1*1e3 << " ms | " << solver.matrixR().nonZeros() << " nonzeros in R"<< std::endl;
        //std::cout << "x1 = " << x.transpose() << std::endl;
        std::cout << "x = " << x.transpose() << std::endl;
        std::cout << "x(ordering) = " << x_ordered.transpose() << std::endl;
        std::cout << "x - x(ordering)= " << (x-x_ordered).transpose() << std::endl;
    }
}






