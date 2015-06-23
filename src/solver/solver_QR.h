/*
 * solver_QR.h
 *
 *  Created on: Jun 22, 2015
 *      Author: jvallve
 */

#ifndef TRUNK_SRC_SOLVER_SOLVER_QR_H_
#define TRUNK_SRC_SOLVER_SOLVER_QR_H_

using namespace Eigen;

class SolverQR
{
    protected:
        SparseQR < SparseMatrix<double>, NaturalOrdering<int>> solver_;
        SparseMatrix<double> A, A_ordered, R;
        VectorXd b, x, x_ordered, x_ordered_partial;
        int n_measurements = 0;
        int n_nodes = 0;

        // ordering variables
        SparseMatrix<int> A_nodes_ordered;
        PermutationMatrix<Dynamic, Dynamic, int> acc_permutation_nodes_matrix;

        CCOLAMDOrdering<int> ordering, partial_ordering;
        VectorXi nodes_ordering_constraints;

    private:
};


#endif /* TRUNK_SRC_SOLVER_SOLVER_QR_H_ */
