/*
 * test_ccolamd.cpp
 *
 *  Created on: Jun 11, 2015
 *      Author: jvallve
 */

typedef int IndexType;

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
#define EIGEN_DEFAULT_DENSE_INDEX_TYPE IndexType
#include <eigen3/Eigen/OrderingMethods>
#include <eigen3/Eigen/CholmodSupport>
#include <eigen3/Eigen/SparseLU>

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
        void operator()(const MatrixType& mat, PermutationType& perm, Index* cmember = nullptr)
        {
            Index m = mat.rows();
            Index n = mat.cols();
            Index nnz = mat.nonZeros();

            // Get the recommended value of Alen to be used by colamd
            Index Alen = ccolamd_recommended(nnz, m, n);
            // Set the default parameters
            double knobs[CCOLAMD_KNOBS];
            Index stats[CCOLAMD_STATS];
            ccolamd_set_defaults(knobs);

            IndexVector p(n + 1), A(Alen);
            for (Index i = 0; i <= n; i++)
                p(i) = mat.outerIndexPtr()[i];
            for (Index i = 0; i < nnz; i++)
                A(i) = mat.innerIndexPtr()[i];

            // Call CColamd routine to compute the ordering
            Index info = compute_ccolamd(m, n, Alen, A.data(), p.data(), knobs, stats, cmember);
            if (!info)
                assert(info && "COLAMD failed ");

            perm.resize(n);
            for (Index i = 0; i < n; i++)
                perm.indices()(p(i)) = i;
        }

    private:
        int compute_ccolamd(int &m, int &n, int &Alen, int* A, int* p, double* knobs, int* stats, int* cmember)
        {
            int info = ccolamd(m, n, Alen, A, p, knobs, stats, cmember);
            //ccolamd_report (stats) ;
            return info;
        }

        long int compute_ccolamd(long int &m, long int &n, long int &Alen, long int* A, long int* p, double* knobs, long int* stats, long int* cmember)
        {
            long int info = ccolamd_l(m, n, Alen, A, p, knobs, stats, cmember);
            //ccolamd_l_report (stats) ;
            return info;
        }
};

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
    IndexType size = atoi(argv[1]); //ordering enabled

    SparseMatrix<double, ColMajor, IndexType> A(size, size), Aordered(size, size);
    CholmodSupernodalLLT < SparseMatrix<double, ColMajor, IndexType> > solver;
    PermutationMatrix<Dynamic, Dynamic, IndexType> perm(size);
    CCOLAMDOrdering<IndexType> ordering;
    Matrix<IndexType, Dynamic, 1> ordering_constraints(size);
    VectorXd b(size), bordered(size), xordered(size), x(size);
    clock_t t1, t2, t3;
    double time1, time2, time3;

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
    A.insert(size - 1, 0) = 2;
    A.insert(0, size - 1) = 2;

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
    // ordering constraints
    ordering_constraints(size-1) = 1;
    ordering_constraints(0) = 1;
    // ordering
    t2 = clock();
    A.makeCompressed();

    std::cout << "Reordering using CCOLAMD:" << std::endl;
    std::cout << "ordering_constraints = " << std::endl << ordering_constraints.transpose() << std::endl << std::endl;
    ordering(A, perm, ordering_constraints.data());
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

    // SOLVING AND REORDERING ------------------------------------
    t3 = clock();
    SparseLU<SparseMatrix<double, ColMajor, IndexType>, CCOLAMDOrdering<IndexType> > solver2;
    solver2.compute(A);
    if (solver2.info() != Success)
    {
        std::cout << "decomposition failed" << std::endl;
        return 0;
    }
    x = solver2.solve(b);
    time3 = ((double) clock() - t3) / CLOCKS_PER_SEC;
    std::cout << "solved in " << time3 << "seconds" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;
}

