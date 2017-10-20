/*
 * gtest_intrinsics_odom2d_serialization.cpp
 *
 *  Created on: Jul 16, 2017
 *      Author: Jeremie Deray
 */

#include "../../utils_gtest.h"

#include "../../../serialization/cereal/serialization_processor_odom2d_params.h"

#include "../../../serialization/cereal/io.h"

#include <cereal/types/memory.hpp>
#include <fstream>

class WolfTestCerealSerializationProcessorParamsOdom2D : public testing::Test
{
public:

  WolfTestCerealSerializationProcessorParamsOdom2D()
  {
    nb_.name = "NAME";
    nb_.type = "ODOM 2D";

    nb_.dist_traveled_th_            = 0.17;
    nb_.theta_traveled_th_           = 0.3;
    nb_.cov_det_th_                  = 0.4;
    nb_.elapsed_time_th_             = 1.5;
    nb_.unmeasured_perturbation_std_ = 1e-5;
  }

  const std::string path_to_io = "/tmp/";
  const std::string filename   = "serialization_processor_odom2d_params";
  const std::string ptr_ext    = "_ptr";

  const std::vector<std::string> exts = {".bin", ".xml", ".json"};

  wolf::ProcessorParamsOdom2D nb_;
};

TEST_F(WolfTestCerealSerializationProcessorParamsOdom2D,
       CerealSerializationProcessorParamsOdom2D)
{
  for (const auto ext : exts)
  {
    const std::string full_path = path_to_io + filename + ext;

    ASSERT_NO_THROW( wolf::save( full_path, nb_ ) )
        << "Failed on saving " << full_path;

    wolf::ProcessorParamsOdom2D nb_load;

    ASSERT_NO_THROW( wolf::load( full_path, nb_load ) )
        << "Failed on loading " << full_path;

    ASSERT_EQ(nb_load.type,               nb_.type)               << full_path;
    ASSERT_EQ(nb_load.name,               nb_.name)               << full_path;
    ASSERT_EQ(nb_load.dist_traveled_th_,  nb_.dist_traveled_th_)  << full_path;
    ASSERT_EQ(nb_load.theta_traveled_th_, nb_.theta_traveled_th_) << full_path;
    ASSERT_EQ(nb_load.cov_det_th_,        nb_.cov_det_th_)        << full_path;
    ASSERT_EQ(nb_load.unmeasured_perturbation_std_,
              nb_.unmeasured_perturbation_std_)                   << full_path;

    /// Testing BasePtr

    const std::string ptr_full_path = path_to_io + filename + ptr_ext + ext;

    {
      wolf::ProcessorParamsBasePtr nb =
          std::make_shared<wolf::ProcessorParamsOdom2D>(nb_);

      ASSERT_NO_THROW( wolf::save( ptr_full_path, nb ) )
          << "Failed on saving " << ptr_full_path;
    }

    {
      wolf::ProcessorParamsBasePtr nb;

      ASSERT_NO_THROW( wolf::load( ptr_full_path, nb ) )
          << "Failed on loading " << ptr_full_path;

      wolf::ProcessorParamsOdom2DPtr nb_cast =
          std::dynamic_pointer_cast<wolf::ProcessorParamsOdom2D>(nb);

      ASSERT_TRUE(nb_cast != nullptr)
          << "Failed on casting " << ptr_full_path;

      ASSERT_EQ(nb_cast->type,               nb_.type)               << full_path;
      ASSERT_EQ(nb_cast->name,               nb_.name)               << full_path;
      ASSERT_EQ(nb_cast->dist_traveled_th_,  nb_.dist_traveled_th_)  << full_path;
      ASSERT_EQ(nb_cast->theta_traveled_th_, nb_.theta_traveled_th_) << full_path;
      ASSERT_EQ(nb_cast->cov_det_th_,        nb_.cov_det_th_)        << full_path;
      ASSERT_EQ(nb_cast->unmeasured_perturbation_std_,
                nb_.unmeasured_perturbation_std_)                    << full_path;
    }
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationProcessorParamsOdom2D::"
         "CerealSerializationProcessorParamsOdom2D !\n");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
