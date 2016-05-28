/**
 * \file test_matrix_prod.cpp
 *
 *  Created on: May 26, 2016
 *      \author: jsola
 */

#include "eigen3/Eigen/Dense"

//std includes
#include <ctime>
#include <iostream>

/**
 * We multiply matrices and see how long it takes.
 * We compare different combinations of row-major and column-major to see which one is the fastest.
 * We can select the matrix size.
 */
int main()
{
    using namespace Eigen;

    int N = 100000;
    const int S = 6;
    Matrix<double, 16, S-3+1> results;
    clock_t t0;

    {
        Matrix<double, Dynamic, Dynamic, RowMajor> R1, R2, Ro;
        Matrix<double, Dynamic, Dynamic, ColMajor> C1, C2, Co;

        for (int s = 3; s <= S; s++)
        {

            R1.setRandom(s, s);
            R2.setRandom(s, s);
            C1.setRandom(s, s);
            C2.setRandom(s, s);

            std::cout << "Timings for dynamic matrix product. R: row major matrix. C: column major matrix. " << s << "x"
                    << s << " matrices." << std::endl;


            t0 = clock();
            for (int i = 0; i < N; i++)
                Ro = R1 * R2;
            std::cout << "Time Ro = R * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

            t0 = clock();
            for (int i = 0; i < N; i++)
                Ro = R1 * C2;
            std::cout << "Time Ro = R * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

            t0 = clock();
            for (int i = 0; i < N; i++)
                Ro = C1 * R2;
            std::cout << "Time Ro = C * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

            t0 = clock();
            for (int i = 0; i < N; i++)
                Ro = C1 * C2;
            std::cout << "Time Ro = C * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

            t0 = clock();
            for (int i = 0; i < N; i++)
                Co = R1 * R2;
            std::cout << "Time Co = R * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

            t0 = clock();
            for (int i = 0; i < N; i++)
                Co = R1 * C2;
            std::cout << "Time Co = R * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

            t0 = clock();
            for (int i = 0; i < N; i++)
                Co = C1 * R2;
            std::cout << "Time Co = C * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

            t0 = clock();
            for (int i = 0; i < N; i++)
                Co = C1 * C2;
            std::cout << "Time Co = C * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6
                    << "us <-- this is the Eigen default!" << std::endl;
        }
    }
    {
        const int s = 3;
        std::cout << "Timings for static matrix product. R: row major matrix. C: column major matrix. " << s << "x"
                << s << " matrices." << std::endl;
        Matrix<double, s, s, RowMajor> R1, R2, Ro;
        Matrix<double, s, s, ColMajor> C1, C2, Co;
        t0 = clock();
        for (int i = 0; i < N; i++)
            Ro = R1 * R2;
        std::cout << "Time Ro = R * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Ro = R1 * C2;
        std::cout << "Time Ro = R * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Ro = C1 * R2;
        std::cout << "Time Ro = C * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Ro = C1 * C2;
        std::cout << "Time Ro = C * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Co = R1 * R2;
        std::cout << "Time Co = R * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Co = R1 * C2;
        std::cout << "Time Co = R * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Co = C1 * R2;
        std::cout << "Time Co = C * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Co = C1 * C2;
        std::cout << "Time Co = C * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6
                << "us <-- this is the Eigen default!" << std::endl;
    }
    {
        const int s = 4;
        std::cout << "Timings for static matrix product. R: row major matrix. C: column major matrix. " << s << "x"
                << s << " matrices." << std::endl;
        Matrix<double, s, s, RowMajor> R1, R2, Ro;
        Matrix<double, s, s, ColMajor> C1, C2, Co;
        t0 = clock();
        for (int i = 0; i < N; i++)
            Ro = R1 * R2;
        std::cout << "Time Ro = R * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Ro = R1 * C2;
        std::cout << "Time Ro = R * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Ro = C1 * R2;
        std::cout << "Time Ro = C * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Ro = C1 * C2;
        std::cout << "Time Ro = C * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Co = R1 * R2;
        std::cout << "Time Co = R * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Co = R1 * C2;
        std::cout << "Time Co = R * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Co = C1 * R2;
        std::cout << "Time Co = C * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Co = C1 * C2;
        std::cout << "Time Co = C * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6
                << "us <-- this is the Eigen default!" << std::endl;
    }
    {
        const int s = 5;
        std::cout << "Timings for static matrix product. R: row major matrix. C: column major matrix. " << s << "x"
                << s << " matrices." << std::endl;
        Matrix<double, s, s, RowMajor> R1, R2, Ro;
        Matrix<double, s, s, ColMajor> C1, C2, Co;
        t0 = clock();
        for (int i = 0; i < N; i++)
            Ro = R1 * R2;
        std::cout << "Time Ro = R * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Ro = R1 * C2;
        std::cout << "Time Ro = R * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Ro = C1 * R2;
        std::cout << "Time Ro = C * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Ro = C1 * C2;
        std::cout << "Time Ro = C * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Co = R1 * R2;
        std::cout << "Time Co = R * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Co = R1 * C2;
        std::cout << "Time Co = R * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Co = C1 * R2;
        std::cout << "Time Co = C * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Co = C1 * C2;
        std::cout << "Time Co = C * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6
                << "us <-- this is the Eigen default!" << std::endl;
    }
    {
        const int s = 6;
        std::cout << "Timings for static matrix product. R: row major matrix. C: column major matrix. " << s << "x"
                << s << " matrices." << std::endl;
        Matrix<double, s, s, RowMajor> R1, R2, Ro;
        Matrix<double, s, s, ColMajor> C1, C2, Co;
        t0 = clock();
        for (int i = 0; i < N; i++)
            Ro = R1 * R2;
        std::cout << "Time Ro = R * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Ro = R1 * C2;
        std::cout << "Time Ro = R * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Ro = C1 * R2;
        std::cout << "Time Ro = C * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Ro = C1 * C2;
        std::cout << "Time Ro = C * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Co = R1 * R2;
        std::cout << "Time Co = R * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Co = R1 * C2;
        std::cout << "Time Co = R * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Co = C1 * R2;
        std::cout << "Time Co = C * R: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6 << "us" << std::endl;

        t0 = clock();
        for (int i = 0; i < N; i++)
            Co = C1 * C2;
        std::cout << "Time Co = C * C: " << ((double)clock() - t0) / CLOCKS_PER_SEC / N * 1e6
                << "us <-- this is the Eigen default!" << std::endl;
    }

    return 0;
}

