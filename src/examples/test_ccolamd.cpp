/*
 * test_ccolamd.cpp
 *
 *  Created on: Jun 11, 2015
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
#include <eigen3/Eigen/CholmodSupport>

// ccolamd
#include "ccolamd.h"

using namespace Eigen;

template<typename Index>
class CCOLAMDOrdering
{
    public:
        typedef PermutationMatrix<Dynamic, Dynamic, Index> PermutationType;
        typedef Matrix<Index, Dynamic, 1> IndexVector;

        template<typename MatrixType>
        void operator()(const MatrixType& mat, PermutationType& perm)
        {
            Index m = mat.rows();
            Index n = mat.cols();
            Index nnz = mat.nonZeros();

            std::cout << "m" << m << std::endl;
            std::cout << "n" << n << std::endl;
            std::cout << "nnz" << nnz << std::endl;

            Index cmember[n];
            // Get the recommended value of Alen to be used by colamd
            Index Alen = ccolamd_recommended(nnz, m, n);
            std::cout << "Alen" << Alen << std::endl;
            // Set the default parameters
            double knobs[CCOLAMD_KNOBS];
            Index stats[CCOLAMD_STATS];
            ccolamd_set_defaults(knobs);

            IndexVector p(n + 1), A(Alen);
            for (Index i = 0; i <= n; i++)
                p(i) = mat.outerIndexPtr()[i];
            for (Index i = 0; i < nnz; i++)
                A(i) = mat.innerIndexPtr()[i];

            std::cout << "p " << p.transpose() << std::endl;
            // Call Colamd routine to compute the ordering
            Index info = ccolamd(m, n, Alen, A.data(), p.data(), knobs, stats, NULL);
            ccolamd_report (stats) ;
            if (!info)
                std::cout << "CCOLAMD failed " << std::endl;
            //eigen_assert(info && "COLAMD failed ");

            perm.resize(n);
            std::cout << "p " << p.transpose() << std::endl;
            std::cout << "perm.rows() " << perm.rows() << std::endl;
            for (Index i = 0; i < n; i++)
                perm.indices()(p(i)) = i;
        }
};
//int ccolamd         /* returns (1) if successful, (0) otherwise*/
//(               /* A and p arguments are modified on output */
//    int n_row,          /* number of rows in A */
//    int n_col,          /* number of columns in A */
//    int Alen,           /* size of the array A */
//    int A [ ],          /* row indices of A, of size Alen */
//    int p [ ],          /* column pointers of A, of size n_col+1 */
//    double knobs [CCOLAMD_KNOBS],/* parameter settings for ccolamd */
//    int stats [CCOLAMD_STATS],  /* ccolamd output statistics and error codes */
//    int cmember [ ]     /* Constraint set of A, of size n_col */
//) ;



//main
int main(int argc, char *argv[])
{
    if (argc != 2 || atoi(argv[1]) < 1)
    {
        std::cout << "Please call me with: [./test_ccolamd SIZE], where:" << std::endl;
        std::cout << "     - SIZE: integer size of the problem" << std::endl;
        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
        return -1;
    }
    int size = atoi(argv[1]); //ordering enabled

    SparseMatrix<double> A(size, size), Aordered(size, size);
    CholmodSupernodalLLT < SparseMatrix<double> > solver;
    PermutationMatrix<Dynamic, Dynamic, int> perm(size);
    CCOLAMDOrdering<int> ordering;
    VectorXd b(size), bordered(size), xordered(size), x(size);
    ;
    clock_t t1, t2;
    double time1, time2;

    // BUILD THE PROBLEM ----------------------------
    //Fill A & b
    A.insert(0, 0) = 5;
    b(0) = 1;
    for (unsigned int i = 1; i < size; i++)
    {
        A.insert(i, i) = 5;
        A.insert(i, i - 1) = 1;
        A.insert(i - 1, i) = 1;
        b(i) = i + 1;
    }
    A.insert(size - 1, 0) = 1;
    A.insert(0, size - 1) = 1;

    std::cout << "Solving Ax = b:" << std::endl << "A = " << std::endl << A << std::endl << std::endl;
    std::cout << "b = " << std::endl << b.transpose() << std::endl << std::endl;

    // SOLVING WITHOUT REORDERING ------------------------------------
    // solve Ax = b
    t1 = clock();
    solver.compute(A);
    if (solver.info() != Success)
    {
        std::cout << "decomposition failed" << std::endl;
        return 0;
    }
    x = solver.solve(b);
    time1 = ((double) clock() - t1) / CLOCKS_PER_SEC;
    std::cout << "solved in " << time1 << "seconds" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;

    // SOLVING AFTER REORDERING ------------------------------------
    // ordering
    t2 = clock();
    A.makeCompressed();

    ordering(A, perm);
    std::cout << "Reordering using AMD:" << std::endl;
    std::cout << "perm = " << std::endl << perm.indices().transpose() << std::endl << std::endl;

    bordered = perm * b;
    Aordered = A.twistedBy(perm);
    std::cout << "reordered A = " << std::endl << Aordered * MatrixXd::Identity(size, size) << std::endl << std::endl;
    std::cout << "reordered b = " << std::endl << bordered.transpose() << std::endl << std::endl;

    // solve Ax = b
    solver.compute(Aordered);
    if (solver.info() != Success)
    {
        std::cout << "decomposition failed" << std::endl;
        return 0;
    }
    xordered = solver.solve(bordered);
    time2 = ((double) clock() - t2) / CLOCKS_PER_SEC;
    std::cout << "solved in " << time2 << "seconds" << std::endl;
    std::cout << "x = " << (perm.inverse() * xordered).transpose() << std::endl;
    std::cout << "x = " << x.transpose() << " (solution without reordering)" << std::endl;
}

