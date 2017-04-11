/*
 * gtest_feature_base.cpp
 *
 *  Created on: Apr 10, 2017
 *      Author: jsola
 */

#include "feature_base.h"

#include "utils_gtest.h"


using namespace wolf;
using namespace Eigen;

TEST(FeatureBase, Constructor)
{
    Vector3s m; m << 1,2,3;
    Matrix3s M; M.setRandom(); M = M*M.transpose();
    Matrix3s I = M.inverse();
    Eigen::LLT<Eigen::MatrixXs> llt_of_info(I); // compute the Cholesky decomposition of A
    Eigen::MatrixXs U = llt_of_info.matrixU();
    Eigen::MatrixXs L = llt_of_info.matrixL();

    FeatureBasePtr f(std::make_shared<FeatureBase>("DUMMY", m, M));

    ASSERT_EIGEN_APPROX(f->getMeasurement(), m, 1e-6);
    ASSERT_EIGEN_APPROX(f->getMeasurementCovariance(), M, 1e-6);
    ASSERT_EIGEN_APPROX(f->getMeasurementSquareRootInformationUpper().transpose() * f->getMeasurementSquareRootInformationUpper(), U.transpose()*U, 1e-6);
    ASSERT_EIGEN_APPROX(f->getMeasurementSquareRootInformationUpper(), U, 1e-6);
}

TEST(FeatureBase, SetMeasurement)
{
    Vector3s m; m << 1,2,3;
    Matrix3s M; M.setRandom(); M = M*M.transpose();
    Matrix3s I = M.inverse();
    Eigen::LLT<Eigen::MatrixXs> llt_of_info(I); // compute the Cholesky decomposition of A
    Eigen::MatrixXs U = llt_of_info.matrixU();
    Eigen::MatrixXs L = llt_of_info.matrixL();

    FeatureBasePtr f(std::make_shared<FeatureBase>("DUMMY", Vector3s::Zero(), Matrix3s::Identity()));

    f->setMeasurement(m);

    ASSERT_EIGEN_APPROX(f->getMeasurement(), m, 1e-6);
}

TEST(FeatureBase, SetMeasurementCovariance)
{
    Vector3s m; m << 1,2,3;
    Matrix3s M; M.setRandom(); M = M*M.transpose();
    Matrix3s I = M.inverse();
    Eigen::LLT<Eigen::MatrixXs> llt_of_info(I); // compute the Cholesky decomposition of A
    Eigen::MatrixXs U = llt_of_info.matrixU();
    Eigen::MatrixXs L = llt_of_info.matrixL();

    FeatureBasePtr f(std::make_shared<FeatureBase>("DUMMY", Vector3s::Zero(), Matrix3s::Identity()));

    f->setMeasurementCovariance(M);

    ASSERT_EIGEN_APPROX(f->getMeasurementCovariance(), M, 1e-6);
    ASSERT_EIGEN_APPROX(f->getMeasurementSquareRootInformationUpper().transpose() * f->getMeasurementSquareRootInformationUpper(), U.transpose()*U, 1e-6);
    ASSERT_EIGEN_APPROX(f->getMeasurementSquareRootInformationUpper(), U, 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

