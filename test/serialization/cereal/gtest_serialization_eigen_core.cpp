/*
 * gtest_intrinsics_odom2d_serialization.cpp
 *
 *  Created on: Jul 16, 2017
 *      Author: Jeremie Deray
 */

#include "../../utils_gtest.h"

#include "../../../serialization/cereal/serialization_eigen_core.h"

#include "../../../serialization/cereal/io.h"

#include <cereal/types/memory.hpp>
#include <fstream>

class WolfTestCerealSerializationEigen : public testing::Test
{
public:

  WolfTestCerealSerializationEigen()
  {
    nb_ = f_mat_t::Random();

    dnb_ = d_mat_t::Random(10, 10);
  }

  const std::string path_to_io = "/tmp/";
  const std::string filename   = "serialization_eigen";

  const std::vector<std::string> exts = {".bin"/*, ".xml", ".json"*/};

  using f_mat_t = Eigen::Matrix<double, 5, 5>;

  using d_mat_t = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

  f_mat_t nb_;

  d_mat_t dnb_;
};

TEST_F(WolfTestCerealSerializationEigen,
       CerealSerializationEigenFixedMat)
{
  for (const auto ext : exts)
  {
    const std::string full_path = path_to_io + filename + ext;

    ASSERT_NO_THROW( wolf::save( full_path, nb_ ) )
        << "Failed on saving " << full_path;

    WolfTestCerealSerializationEigen::f_mat_t nb_load;

    ASSERT_NO_THROW( wolf::load( full_path, nb_load ) )
        << "Failed on loading " << full_path;

    EXPECT_EQ(nb_load, nb_) << full_path;
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationEigen::"
         "CerealSerializationEigenFixedMat !\n");
}

TEST_F(WolfTestCerealSerializationEigen,
       CerealSerializationEigenDynamicMat)
{
  for (const auto ext : exts)
  {
    const std::string full_path = path_to_io + filename + ext;

    ASSERT_NO_THROW( wolf::save( full_path, dnb_ ) )
        << "Failed on saving " << full_path;

    WolfTestCerealSerializationEigen::d_mat_t dnb_load;

    ASSERT_NO_THROW( wolf::load( full_path, dnb_load ) )
        << "Failed on loading " << full_path;

    EXPECT_EQ(dnb_load, dnb_) << full_path;
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationEigen::"
         "CerealSerializationEigenDynamicMat !\n");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
