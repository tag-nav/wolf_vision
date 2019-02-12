/*
 * gtest_intrinsics_odom2d_serialization.cpp
 *
 *  Created on: Jul 16, 2017
 *      Author: Jeremie Deray
 */

#include "../../utils_gtest.h"

#include "../../../serialization/cereal/io.h"
#include "../../../serialization/cereal/serialization_sensor_odom2d_intrinsic.h"

#include "../../../serialization/cereal/archives.h"

#include <cereal/types/memory.hpp>
#include <fstream>

namespace wolf {

using IntrinsicsOdom2DPtr = std::shared_ptr<IntrinsicsOdom2D>;

}

class WolfTestCerealSerializationSaveLoad : public testing::Test
{
public:

  WolfTestCerealSerializationSaveLoad()
  {
    //
  }

  const std::string path_to_io = "/tmp/";

  decltype(std::declval<wolf::IntrinsicsOdom2D>().type) nb_type = "TYPE";
  decltype(std::declval<wolf::IntrinsicsOdom2D>().name) nb_name = "NAME";
  decltype(std::declval<wolf::IntrinsicsOdom2D>().k_disp_to_disp) nb_k_disp_to_disp = 0.54;
  decltype(std::declval<wolf::IntrinsicsOdom2D>().k_rot_to_rot) nb_k_rot_to_rot   = 0.18;
};

TEST_F(WolfTestCerealSerializationSaveLoad, CerealSerializationSaveLoadExtension)
{
  const std::string xml  = "/test/filename.xml";
  const std::string bin  = "/test/filename.bin";
  const std::string json = "/test/filename.json";

  ASSERT_EQ(wolf::serialization::extension(xml),  ".xml");
  ASSERT_EQ(wolf::serialization::extension(bin),  ".bin");
  ASSERT_EQ(wolf::serialization::extension(json), ".json");
}

TEST_F(WolfTestCerealSerializationSaveLoad,
       CerealSerializationSaveLoadXML)
{
  const std::string filename = path_to_io + "save_load_serialization.xml";

  {
    wolf::IntrinsicsOdom2D nb;
    nb.type = nb_type;
    nb.name = nb_name;
    nb.k_disp_to_disp = nb_k_disp_to_disp;
    nb.k_rot_to_rot   = nb_k_rot_to_rot;

    ASSERT_NO_THROW( wolf::save( filename, nb ) );
  }

  {
    wolf::IntrinsicsOdom2D nb;

    ASSERT_NO_THROW( wolf::load( filename, nb ) );

    ASSERT_EQ(nb.type, nb_type);
    ASSERT_EQ(nb.name, nb_name);
    ASSERT_EQ(nb.k_disp_to_disp, nb_k_disp_to_disp);
    ASSERT_EQ(nb.k_rot_to_rot,   nb_k_rot_to_rot);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationSaveLoad::"
         "CerealSerializationSaveLoadXML !\n");
}

TEST_F(WolfTestCerealSerializationSaveLoad,
       CerealSerializationSaveLoadJSON)
{
  const std::string filename = path_to_io + "save_load_serialization.json";

  {
    wolf::IntrinsicsOdom2D nb;
    nb.type = nb_type;
    nb.name = nb_name;
    nb.k_disp_to_disp = nb_k_disp_to_disp;
    nb.k_rot_to_rot   = nb_k_rot_to_rot;

    ASSERT_NO_THROW( wolf::save( filename, nb ) );
  }

  {
    wolf::IntrinsicsOdom2D nb;

    ASSERT_NO_THROW( wolf::load( filename, nb ) );

    ASSERT_EQ(nb.type, nb_type);
    ASSERT_EQ(nb.name, nb_name);
    ASSERT_EQ(nb.k_disp_to_disp, nb_k_disp_to_disp);
    ASSERT_EQ(nb.k_rot_to_rot,   nb_k_rot_to_rot);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationSaveLoad::"
         "CerealSerializationSaveLoadJSON !\n");
}

TEST_F(WolfTestCerealSerializationSaveLoad,
       CerealSerializationSaveLoadBinary)
{
  const std::string filename = path_to_io + "save_load_serialization.bin";

  {
    wolf::IntrinsicsOdom2D nb;
    nb.type = nb_type;
    nb.name = nb_name;
    nb.k_disp_to_disp = nb_k_disp_to_disp;
    nb.k_rot_to_rot   = nb_k_rot_to_rot;

    ASSERT_NO_THROW( wolf::save( filename, nb ) );
  }

  {
    wolf::IntrinsicsOdom2D nb;

    ASSERT_NO_THROW( wolf::load( filename, nb ) );

    ASSERT_EQ(nb.type, nb_type);
    ASSERT_EQ(nb.name, nb_name);
    ASSERT_EQ(nb.k_disp_to_disp, nb_k_disp_to_disp);
    ASSERT_EQ(nb.k_rot_to_rot,   nb_k_rot_to_rot);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationSaveLoad::"
         "CerealSerializationSaveLoadBinary !\n");
}

TEST_F(WolfTestCerealSerializationSaveLoad,
       CerealSerializationSaveLoadNoExt)
{
  const std::string filename = path_to_io + "save_load_serialization_no_ext";

  {
    wolf::IntrinsicsOdom2D nb;
    nb.type = nb_type;
    nb.name = nb_name;
    nb.k_disp_to_disp = nb_k_disp_to_disp;
    nb.k_rot_to_rot   = nb_k_rot_to_rot;

    ASSERT_NO_THROW( wolf::save( filename, nb ) );
  }

  {
    wolf::IntrinsicsOdom2D nb;

    ASSERT_NO_THROW( wolf::load( filename, nb ) );

    ASSERT_EQ(nb.type, nb_type);
    ASSERT_EQ(nb.name, nb_name);
    ASSERT_EQ(nb.k_disp_to_disp, nb_k_disp_to_disp);
    ASSERT_EQ(nb.k_rot_to_rot,   nb_k_rot_to_rot);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationSaveLoad::"
         "CerealSerializationSaveLoadNoExt !\n");
}

TEST_F(WolfTestCerealSerializationSaveLoad,
       CerealSerializationSaveLoadUnknownExt)
{
  const std::string filename = path_to_io + "save_load_serialization.foo";

  {
    wolf::IntrinsicsOdom2D nb;

    ASSERT_THROW( wolf::save( filename, nb ), std::runtime_error );
  }

  {
    wolf::IntrinsicsOdom2D nb;

    ASSERT_THROW( wolf::load( filename, nb ), std::runtime_error );
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationSaveLoad::"
         "CerealSerializationSaveLoadUnknownExt !\n");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
