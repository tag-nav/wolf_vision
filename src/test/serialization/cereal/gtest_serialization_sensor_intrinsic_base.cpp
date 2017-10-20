/*
 * gtest_intrinsics_base_serialization.cpp
 *
 *  Created on: Jul 16, 2017
 *      Author: Jeremie Deray
 */

#include "../../utils_gtest.h"

#include "../../../serialization/cereal/serialization_sensor_intrinsic_base.h"

#include "../../../serialization/cereal/archives.h"

#include <cereal/types/memory.hpp>
#include <fstream>

class WolfTestCerealSerializationIntrinsicsBase : public testing::Test
{
public:

  WolfTestCerealSerializationIntrinsicsBase()
  {
    //
  }

  const std::string path_to_io = "/tmp/";

  decltype(std::declval<wolf::IntrinsicsBase>().type) nb_type = "TYPE";
  decltype(std::declval<wolf::IntrinsicsBase>().name) nb_name = "NAME";
};

TEST_F(WolfTestCerealSerializationIntrinsicsBase,
       CerealSerializationIntrinsicsBaseXML)
{
  {
    wolf::IntrinsicsBase nb;
    nb.type = nb_type;
    nb.name = nb_name;

    std::ofstream os(path_to_io + "intrinsics_base_serialization.xml");
    cereal::XMLOutputArchive xml_archive(os);

    ASSERT_NO_THROW( xml_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "intrinsics_base_serialization.xml");
    cereal::XMLInputArchive xml_archive(is);

    wolf::IntrinsicsBase nb;

    ASSERT_NO_THROW( xml_archive( nb ) );

    ASSERT_EQ(nb.type, nb_type);
    ASSERT_EQ(nb.name, nb_name);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationIntrinsicsBase::"
         "CerealSerializationIntrinsicsBaseXML !\n");
}

TEST_F(WolfTestCerealSerializationIntrinsicsBase,
       CerealSerializationIntrinsicsBasePtrXML)
{
  {
    wolf::IntrinsicsBasePtr nb = std::make_shared<wolf::IntrinsicsBase>();
    nb->name = nb_name;
    nb->type = nb_type;

    std::ofstream os(path_to_io + "intrinsics_base_ptr_serialization.xml");
    cereal::XMLOutputArchive xml_archive(os);

    ASSERT_NO_THROW( xml_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "intrinsics_base_ptr_serialization.xml");
    cereal::XMLInputArchive xml_archive(is);

    wolf::IntrinsicsBasePtr nb;

    ASSERT_NO_THROW( xml_archive( nb ) );

    ASSERT_EQ(nb->type, nb_type);
    ASSERT_EQ(nb->name, nb_name);

    ASSERT_TRUE(
          std::dynamic_pointer_cast<
          wolf::IntrinsicsBase>(nb) != nullptr);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationIntrinsicsBase::"
         "CerealSerializationIntrinsicsBasePtrXML !\n");
}

TEST_F(WolfTestCerealSerializationIntrinsicsBase,
       CerealSerializationIntrinsicsBaseJSON)
{
  {
    wolf::IntrinsicsBase nb;
    nb.type = nb_type;
    nb.name = nb_name;

    std::ofstream os(path_to_io + "intrinsics_base_serialization.json");
    cereal::JSONOutputArchive json_archive(os);

    ASSERT_NO_THROW( json_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "intrinsics_base_serialization.json");
    cereal::JSONInputArchive json_archive(is);

    wolf::IntrinsicsBase nb;

    ASSERT_NO_THROW( json_archive( nb ) );

    ASSERT_EQ(nb.type, nb_type);
    ASSERT_EQ(nb.name, nb_name);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationIntrinsicsBase::"
         "CerealSerializationIntrinsicsBaseJSON !\n");
}

TEST_F(WolfTestCerealSerializationIntrinsicsBase,
       CerealSerializationIntrinsicsBasePtrJSON)
{
  {
    wolf::IntrinsicsBasePtr nb = std::make_shared<wolf::IntrinsicsBase>();
    nb->name = nb_name;
    nb->type = nb_type;

    std::ofstream os(path_to_io + "intrinsics_base_ptr_serialization.json");
    cereal::JSONOutputArchive json_archive(os);

    ASSERT_NO_THROW( json_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "intrinsics_base_ptr_serialization.json");
    cereal::JSONInputArchive json_archive(is);

    wolf::IntrinsicsBasePtr nb;

    ASSERT_NO_THROW( json_archive( nb ) );

    ASSERT_EQ(nb->type, nb_type);
    ASSERT_EQ(nb->name, nb_name);

    ASSERT_TRUE(
          std::dynamic_pointer_cast<
          wolf::IntrinsicsBase>(nb) != nullptr);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationIntrinsicsBase::"
         "CerealSerializationIntrinsicsBasePtrJSON !\n");
}

TEST_F(WolfTestCerealSerializationIntrinsicsBase,
       CerealSerializationIntrinsicsBaseBinary)
{
  {
    wolf::IntrinsicsBase nb;
    nb.type = nb_type;
    nb.name = nb_name;

    std::ofstream os(path_to_io + "intrinsics_base_serialization.bin");
    cereal::BinaryOutputArchive bin_archive(os);

    ASSERT_NO_THROW( bin_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "intrinsics_base_serialization.bin");
    cereal::BinaryInputArchive bin_archive(is);

    wolf::IntrinsicsBase nb;

    ASSERT_NO_THROW( bin_archive( nb ) );

    ASSERT_EQ(nb.type, nb_type);
    ASSERT_EQ(nb.name, nb_name);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationIntrinsicsBase::"
         "CerealSerializationIntrinsicsBaseBinary !\n");
}

TEST_F(WolfTestCerealSerializationIntrinsicsBase, CerealSerializationIntrinsicsBasePtrBinary)
{
  {
    wolf::IntrinsicsBasePtr nb = std::make_shared<wolf::IntrinsicsBase>();
    nb->name = nb_name;
    nb->type = nb_type;

    std::ofstream os(path_to_io + "intrinsics_base_ptr_serialization.bin");
    cereal::BinaryOutputArchive bin_archive(os);

    ASSERT_NO_THROW( bin_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "intrinsics_base_ptr_serialization.bin");
    cereal::BinaryInputArchive bin_archive(is);

    wolf::IntrinsicsBasePtr nb;

    ASSERT_NO_THROW( bin_archive( nb ) );

    ASSERT_EQ(nb->type, nb_type);
    ASSERT_EQ(nb->name, nb_name);

    ASSERT_TRUE(
          std::dynamic_pointer_cast<
          wolf::IntrinsicsBase>(nb) != nullptr);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationIntrinsicsBase::"
         "CerealSerializationIntrinsicsBasePtrBinary !\n");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
