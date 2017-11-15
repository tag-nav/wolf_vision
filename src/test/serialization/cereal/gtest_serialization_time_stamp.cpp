/*
 * gtest_intrinsics_odom2d_serialization.cpp
 *
 *  Created on: Jul 16, 2017
 *      Author: Jeremie Deray
 */

#include "../../utils_gtest.h"

#include "../../../serialization/cereal/serialization_time_stamp.h"

#include "../../../serialization/cereal/io.h"

#include <cereal/types/memory.hpp>
#include <fstream>

class WolfTestCerealSerializationTimeStamp : public testing::Test
{
public:

  WolfTestCerealSerializationTimeStamp()
  {
    nb_.setToNow();
  }

  const std::string path_to_io = "/tmp/";
  const std::string filename   = "serialization_time_stamp";
  const std::string ptr_ext    = "_ptr";

  const std::vector<std::string> exts = {".bin", ".xml", ".json"};

  wolf::TimeStamp nb_;
};

TEST_F(WolfTestCerealSerializationTimeStamp,
       CerealSerializationTimeStamp)
{
  for (const auto ext : exts)
  {
    const std::string full_path = path_to_io + filename + ext;

    ASSERT_NO_THROW( wolf::save( full_path, nb_ ) )
        << "Failed on saving " << full_path;

    wolf::TimeStamp nb_load;

    ASSERT_NO_THROW( wolf::load( full_path, nb_load ) )
        << "Failed on loading " << full_path;

    ASSERT_EQ(nb_load, nb_) << full_path;
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationTimeStamp::"
         "CerealSerializationTimeStamp !\n");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
