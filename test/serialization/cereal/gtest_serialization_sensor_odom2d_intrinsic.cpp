/*
 * gtest_intrinsics_odom2d_serialization.cpp
 *
 *  Created on: Jul 16, 2017
 *      Author: Jeremie Deray
 */

#include "../../utils_gtest.h"

#include "../../../serialization/cereal/serialization_sensor_odom2d_intrinsic.h"

#include "../../../serialization/cereal/archives.h"

#include <cereal/types/memory.hpp>
#include <fstream>

namespace wolf {

using IntrinsicsOdom2DPtr = std::shared_ptr<IntrinsicsOdom2D>;

}

class WolfTestCerealSerializationIntrinsicsOdom2D : public testing::Test
{
public:

  WolfTestCerealSerializationIntrinsicsOdom2D()
  {
    nb_.k_disp_to_disp = 0.54;
    nb_.k_rot_to_rot = 0.18;
    nb_.name = "NAME";
    nb_.type = "TYPE";
  }

  const std::string path_to_io = "/tmp/";

  wolf::IntrinsicsOdom2D nb_;
};

TEST_F(WolfTestCerealSerializationIntrinsicsOdom2D,
       CerealSerializationIntrinsicsOdom2DXML)
{
  {
    wolf::IntrinsicsOdom2D nb;
    nb.type = nb_.type;
    nb.name = nb_.name;
    nb.k_disp_to_disp = nb_.k_disp_to_disp;
    nb.k_rot_to_rot   = nb_.k_rot_to_rot;

    std::ofstream os(path_to_io + "intrinsics_odom2d_serialization.xml");
    cereal::XMLOutputArchive xml_archive(os);

    ASSERT_NO_THROW( xml_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "intrinsics_odom2d_serialization.xml");
    cereal::XMLInputArchive xml_archive(is);

    wolf::IntrinsicsOdom2D nb;

    ASSERT_NO_THROW( xml_archive( nb ) );

    ASSERT_EQ(nb.type, nb_.type);
    ASSERT_EQ(nb.name, nb_.name);
    ASSERT_EQ(nb.k_disp_to_disp, nb_.k_disp_to_disp);
    ASSERT_EQ(nb.k_rot_to_rot,   nb_.k_rot_to_rot);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationIntrinsicsOdom2D::"
         "CerealSerializationIntrinsicsOdom2DXML !\n");
}

TEST_F(WolfTestCerealSerializationIntrinsicsOdom2D,
       CerealSerializationIntrinsicsOdom2DPtrXML)
{
  {
    wolf::IntrinsicsBasePtr nb = std::make_shared<wolf::IntrinsicsOdom2D>(nb_);

    std::ofstream os(path_to_io + "intrinsics_odom2d_ptr_serialization.xml");
    cereal::XMLOutputArchive xml_archive(os);

    ASSERT_NO_THROW( xml_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "intrinsics_odom2d_ptr_serialization.xml");
    cereal::XMLInputArchive xml_archive(is);

    wolf::IntrinsicsBasePtr nb;

    ASSERT_NO_THROW( xml_archive( nb ) );

    wolf::IntrinsicsOdom2DPtr nb_cast =
        std::dynamic_pointer_cast<wolf::IntrinsicsOdom2D>(nb);

    ASSERT_TRUE(nb_cast != nullptr);

    ASSERT_EQ(nb_cast->type, nb_.type);
    ASSERT_EQ(nb_cast->name, nb_.name);
    ASSERT_EQ(nb_cast->k_disp_to_disp, nb_.k_disp_to_disp);
    ASSERT_EQ(nb_cast->k_rot_to_rot,   nb_.k_rot_to_rot);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationIntrinsicsOdom2D::"
         "CerealSerializationIntrinsicsOdom2DPtrXML !\n");
}

TEST_F(WolfTestCerealSerializationIntrinsicsOdom2D,
       CerealSerializationIntrinsicsOdom2DJSON)
{
  {
    wolf::IntrinsicsOdom2D nb;
    nb.type = nb_.type;
    nb.name = nb_.name;
    nb.k_disp_to_disp = nb_.k_disp_to_disp;
    nb.k_rot_to_rot   = nb_.k_rot_to_rot;

    std::ofstream os(path_to_io + "intrinsics_odom2d_serialization.json");
    cereal::JSONOutputArchive json_archive(os);

    ASSERT_NO_THROW( json_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "intrinsics_odom2d_serialization.json");
    cereal::JSONInputArchive json_archive(is);

    wolf::IntrinsicsOdom2D nb;

    ASSERT_NO_THROW( json_archive( nb ) );

    ASSERT_EQ(nb.type, nb_.type);
    ASSERT_EQ(nb.name, nb_.name);
    ASSERT_EQ(nb.k_disp_to_disp, nb_.k_disp_to_disp);
    ASSERT_EQ(nb.k_rot_to_rot,   nb_.k_rot_to_rot);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationIntrinsicsOdom2D::"
         "CerealSerializationIntrinsicsOdom2DJSON !\n");
}

TEST_F(WolfTestCerealSerializationIntrinsicsOdom2D,
       CerealSerializationIntrinsicsOdom2DPtrJSON)
{
  {
    wolf::IntrinsicsBasePtr nb = std::make_shared<wolf::IntrinsicsOdom2D>(nb_);

    std::ofstream os(path_to_io + "intrinsics_odom2d_ptr_serialization.json");
    cereal::JSONOutputArchive json_archive(os);

    ASSERT_NO_THROW( json_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "intrinsics_odom2d_ptr_serialization.json");
    cereal::JSONInputArchive json_archive(is);

    wolf::IntrinsicsBasePtr nb;

    ASSERT_NO_THROW( json_archive( nb ) );

    wolf::IntrinsicsOdom2DPtr nb_cast =
        std::dynamic_pointer_cast<wolf::IntrinsicsOdom2D>(nb);

    ASSERT_TRUE(nb_cast != nullptr);

    ASSERT_EQ(nb_cast->type, nb_.type);
    ASSERT_EQ(nb_cast->name, nb_.name);
    ASSERT_EQ(nb_cast->k_disp_to_disp, nb_.k_disp_to_disp);
    ASSERT_EQ(nb_cast->k_rot_to_rot,   nb_.k_rot_to_rot);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationIntrinsicsOdom2D::"
         "CerealSerializationIntrinsicsOdom2DPtrJSON !\n");
}

TEST_F(WolfTestCerealSerializationIntrinsicsOdom2D,
       CerealSerializationIntrinsicsOdom2DBinary)
{
  {
    wolf::IntrinsicsOdom2D nb;
    nb.type = nb_.type;
    nb.name = nb_.name;
    nb.k_disp_to_disp = nb_.k_disp_to_disp;
    nb.k_rot_to_rot   = nb_.k_rot_to_rot;

    std::ofstream os(path_to_io + "intrinsics_odom2d_serialization.bin");
    cereal::BinaryOutputArchive bin_archive(os);

    ASSERT_NO_THROW( bin_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "intrinsics_odom2d_serialization.bin");
    cereal::BinaryInputArchive bin_archive(is);

    wolf::IntrinsicsOdom2D nb;

    ASSERT_NO_THROW( bin_archive( nb ) );

    ASSERT_EQ(nb.type, nb_.type);
    ASSERT_EQ(nb.name, nb_.name);
    ASSERT_EQ(nb.k_disp_to_disp, nb_.k_disp_to_disp);
    ASSERT_EQ(nb.k_rot_to_rot,   nb_.k_rot_to_rot);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationIntrinsicsOdom2D::"
         "CerealSerializationIntrinsicsOdom2DBinary !\n");
}

TEST_F(WolfTestCerealSerializationIntrinsicsOdom2D, CerealSerializationIntrinsicsOdom2DPtrBinary)
{
  {
    wolf::IntrinsicsBasePtr nb = std::make_shared<wolf::IntrinsicsOdom2D>(nb_);

    std::ofstream os(path_to_io + "intrinsics_odom2d_ptr_serialization.bin");
    cereal::BinaryOutputArchive bin_archive(os);

    ASSERT_NO_THROW( bin_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "intrinsics_odom2d_ptr_serialization.bin");
    cereal::BinaryInputArchive bin_archive(is);

    wolf::IntrinsicsBasePtr nb;

    ASSERT_NO_THROW( bin_archive( nb ) );

    wolf::IntrinsicsOdom2DPtr nb_cast =
        std::dynamic_pointer_cast<wolf::IntrinsicsOdom2D>(nb);

    ASSERT_TRUE(nb_cast != nullptr);

    ASSERT_EQ(nb_cast->type, nb_.type);
    ASSERT_EQ(nb_cast->name, nb_.name);
    ASSERT_EQ(nb_cast->k_disp_to_disp, nb_.k_disp_to_disp);
    ASSERT_EQ(nb_cast->k_rot_to_rot,   nb_.k_rot_to_rot);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationIntrinsicsOdom2D::"
         "CerealSerializationIntrinsicsOdom2DPtrBinary !\n");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
