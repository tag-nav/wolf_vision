/*
 * ccolamd_ordering.h
 *
 *  Created on: Jun 12, 2015
 *      Author: jvallve
 */

#ifndef TRUNK_SRC_WOLF_SOLVER_CCOLAMD_ORDERING_H_
#define TRUNK_SRC_WOLF_SOLVER_CCOLAMD_ORDERING_H_



//std includes
#include <iostream>

// Eigen includes
#include <eigen3/Eigen/OrderingMethods>
#include <eigen3/Eigen/CholmodSupport>
#include <eigen3/Eigen/SparseLU>

// ccolamd
#include "ccolamd.h"

namespace Eigen{

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
//            std::cout << "p = " << std::endl << p.transpose() << std::endl;
//            std::cout << "A = " << std::endl << A.head(nnz).transpose() << std::endl;

            // Call CColamd routine to compute the ordering
            Index info = compute_ccolamd(m, n, Alen, A.data(), p.data(), knobs, stats, cmember);
            if (!info)
                assert(info && "CCOLAMD failed ");

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
}


#endif /* TRUNK_SRC_WOLF_SOLVER_CCOLAMD_ORDERING_H_ */
