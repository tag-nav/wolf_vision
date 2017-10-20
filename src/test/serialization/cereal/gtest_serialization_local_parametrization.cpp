/*
 * gtest_node_base_serialization.cpp
 *
 *  Created on: Jul 16, 2017
 *      Author: Jeremie Deray
 */

#include "../../utils_gtest.h"

#include "../../../serialization/cereal/serialization_local_parametrization_quaternion.h"
#include "../../../serialization/cereal/serialization_local_parametrization_homogeneous.h"

#include "../../../serialization/cereal/archives.h"

#include <cereal/types/memory.hpp>
#include <fstream>

///////////////////////////////////////
/// LocalParametrizationHomogeneous ///
///////////////////////////////////////

const std::string path_to_io = "/tmp/";

TEST(TestSerialization, SerializationLocalParametrizationHomogeneousXML)
{
  {
    std::ofstream os(path_to_io + "local_parametrization_serialization.xml");
    cereal::XMLOutputArchive archive(os);

    wolf::LocalParametrizationHomogeneous local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_serialization.xml");
    cereal::XMLInputArchive archive(is);

    wolf::LocalParametrizationHomogeneous local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h.getGlobalSize(), 4);
    ASSERT_EQ(local_param_h.getLocalSize(),  3);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationHomogeneousXML !\n");
}

TEST(TestSerialization, SerializationLocalParametrizationHomogeneousPtrXML)
{
  using LocalParametrizationPtr = std::shared_ptr<wolf::LocalParametrizationBase> ;

  {
    std::ofstream os(path_to_io + "local_parametrization_ptr_serialization.xml");
    cereal::XMLOutputArchive archive(os);

    LocalParametrizationPtr local_param_h =
        std::make_shared<wolf::LocalParametrizationHomogeneous>();

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_ptr_serialization.xml");
    cereal::XMLInputArchive archive(is);

    LocalParametrizationPtr local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h->getGlobalSize(), 4);
    ASSERT_EQ(local_param_h->getLocalSize(),  3);

    ASSERT_TRUE(
          std::dynamic_pointer_cast<
          wolf::LocalParametrizationHomogeneous>(local_param_h) != nullptr);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationHomogeneousPtrXML !\n");
}

TEST(TestSerialization, SerializationLocalParametrizationHomogeneousJSON)
{
  {
    std::ofstream os(path_to_io + "local_parametrization_serialization.json");
    cereal::JSONOutputArchive archive(os);

    wolf::LocalParametrizationHomogeneous local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_serialization.json");
    cereal::JSONInputArchive archive(is);

    wolf::LocalParametrizationHomogeneous local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h.getGlobalSize(), 4);
    ASSERT_EQ(local_param_h.getLocalSize(),  3);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationHomogeneousJSON !\n");
}

TEST(TestSerialization, SerializationLocalParametrizationHomogeneousPtrJSON)
{
  using LocalParametrizationPtr = std::shared_ptr<wolf::LocalParametrizationBase> ;

  {
    std::ofstream os(path_to_io + "local_parametrization_ptr_serialization.json");
    cereal::JSONOutputArchive archive(os);

    LocalParametrizationPtr local_param_h =
        std::make_shared<wolf::LocalParametrizationHomogeneous>();

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_ptr_serialization.json");
    cereal::JSONInputArchive archive(is);

    LocalParametrizationPtr local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h->getGlobalSize(), 4);
    ASSERT_EQ(local_param_h->getLocalSize(),  3);

    ASSERT_TRUE(
          std::dynamic_pointer_cast<
          wolf::LocalParametrizationHomogeneous>(local_param_h) != nullptr);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationHomogeneousPtrJSON !\n");
}

TEST(TestSerialization, SerializationLocalParametrizationHomogeneousBIN)
{
  {
    std::ofstream os(path_to_io + "local_parametrization_serialization.bin");
    cereal::BinaryOutputArchive archive(os);

    wolf::LocalParametrizationHomogeneous local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_serialization.bin");
    cereal::BinaryInputArchive archive(is);

    wolf::LocalParametrizationHomogeneous local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h.getGlobalSize(), 4);
    ASSERT_EQ(local_param_h.getLocalSize(),  3);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationHomogeneousBIN !\n");
}

TEST(TestSerialization, SerializationLocalParametrizationHomogeneousPtrBIN)
{
  using LocalParametrizationPtr = std::shared_ptr<wolf::LocalParametrizationBase> ;

  {
    std::ofstream os(path_to_io + "local_parametrization_ptr_serialization.bin");
    cereal::BinaryOutputArchive archive(os);

    LocalParametrizationPtr local_param_h =
        std::make_shared<wolf::LocalParametrizationHomogeneous>();

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_ptr_serialization.bin");
    cereal::BinaryInputArchive archive(is);

    LocalParametrizationPtr local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h->getGlobalSize(), 4);
    ASSERT_EQ(local_param_h->getLocalSize(),  3);

    ASSERT_TRUE(
          std::dynamic_pointer_cast<
          wolf::LocalParametrizationHomogeneous>(local_param_h) != nullptr);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationHomogeneousPtrBIN !\n");
}


//////////////////////////////////////
/// LocalParametrizationQuaternion ///
//////////////////////////////////////

//////////////////////////////////////
///           LOCAL                ///
//////////////////////////////////////

TEST(TestSerialization, SerializationLocalParametrizationQuaternionXML)
{
  {
    std::ofstream os(path_to_io + "local_parametrization_quat_serialization.xml");
    cereal::XMLOutputArchive archive(os);

    wolf::LocalParametrizationQuaternionLocal local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_quat_serialization.xml");
    cereal::XMLInputArchive archive(is);

    wolf::LocalParametrizationQuaternionLocal local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h.getGlobalSize(), 4);
    ASSERT_EQ(local_param_h.getLocalSize(),  3);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationQuaternionXML !\n");
}

TEST(TestSerialization, SerializationLocalParametrizationQuaternionPtrXML)
{
  using LocalParametrizationPtr = std::shared_ptr<wolf::LocalParametrizationBase> ;

  {
    std::ofstream os(path_to_io + "local_parametrization_quat_ptr_serialization.xml");
    cereal::XMLOutputArchive archive(os);

    LocalParametrizationPtr local_param_h =
        std::make_shared<wolf::LocalParametrizationQuaternionLocal>();

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_quat_ptr_serialization.xml");
    cereal::XMLInputArchive archive(is);

    LocalParametrizationPtr local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h->getGlobalSize(), 4);
    ASSERT_EQ(local_param_h->getLocalSize(),  3);

    ASSERT_TRUE(
          std::dynamic_pointer_cast<
          wolf::LocalParametrizationQuaternionLocal>(local_param_h) != nullptr);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationQuaternionPtrXML !\n");
}

TEST(TestSerialization, SerializationLocalParametrizationQuaternionJSON)
{
  {
    std::ofstream os(path_to_io + "local_parametrization_quat_serialization.json");
    cereal::JSONOutputArchive archive(os);

    wolf::LocalParametrizationQuaternionLocal local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_quat_serialization.json");
    cereal::JSONInputArchive archive(is);

    wolf::LocalParametrizationQuaternionLocal local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h.getGlobalSize(), 4);
    ASSERT_EQ(local_param_h.getLocalSize(),  3);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationQuaternionJSON !\n");
}

TEST(TestSerialization, SerializationLocalParametrizationQuaternionPtrJSON)
{
  using LocalParametrizationPtr = std::shared_ptr<wolf::LocalParametrizationBase> ;

  {
    std::ofstream os(path_to_io + "local_parametrization_quat_ptr_serialization.json");
    cereal::JSONOutputArchive archive(os);

    LocalParametrizationPtr local_param_h =
        std::make_shared<wolf::LocalParametrizationQuaternionLocal>();

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_quat_ptr_serialization.json");
    cereal::JSONInputArchive archive(is);

    LocalParametrizationPtr local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h->getGlobalSize(), 4);
    ASSERT_EQ(local_param_h->getLocalSize(),  3);

    ASSERT_TRUE(
          std::dynamic_pointer_cast<
          wolf::LocalParametrizationQuaternionLocal>(local_param_h) != nullptr);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationQuaternionPtrJSON !\n");
}

TEST(TestSerialization, SerializationLocalParametrizationQuaternionBIN)
{
  {
    std::ofstream os(path_to_io + "local_parametrization_quat_serialization.bin");
    cereal::BinaryOutputArchive archive(os);

    wolf::LocalParametrizationQuaternionLocal local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_quat_serialization.bin");
    cereal::BinaryInputArchive archive(is);

    wolf::LocalParametrizationQuaternionLocal local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h.getGlobalSize(), 4);
    ASSERT_EQ(local_param_h.getLocalSize(),  3);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationQuaternionBIN !\n");
}

TEST(TestSerialization, SerializationLocalParametrizationQuaternionPtrBIN)
{
  using LocalParametrizationPtr = std::shared_ptr<wolf::LocalParametrizationBase> ;

  {
    std::ofstream os(path_to_io + "local_parametrization_quat_ptr_serialization.bin");
    cereal::BinaryOutputArchive archive(os);

    LocalParametrizationPtr local_param_h =
        std::make_shared<wolf::LocalParametrizationQuaternionLocal>();

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_quat_ptr_serialization.bin");
    cereal::BinaryInputArchive archive(is);

    LocalParametrizationPtr local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h->getGlobalSize(), 4);
    ASSERT_EQ(local_param_h->getLocalSize(),  3);

    ASSERT_TRUE(
          std::dynamic_pointer_cast<
          wolf::LocalParametrizationQuaternionLocal>(local_param_h) != nullptr);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationQuaternionPtrBIN !\n");
}

//////////////////////////////////////
///           GLOBAL               ///
//////////////////////////////////////

TEST(TestSerialization, SerializationLocalParametrizationQuaternionGlobalXML)
{
  {
    std::ofstream os(path_to_io + "local_parametrization_quatg_serialization.xml");
    cereal::XMLOutputArchive archive(os);

    wolf::LocalParametrizationQuaternionGlobal local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_quatg_serialization.xml");
    cereal::XMLInputArchive archive(is);

    wolf::LocalParametrizationQuaternionGlobal local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h.getGlobalSize(), 4);
    ASSERT_EQ(local_param_h.getLocalSize(),  3);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationQuaternionGlobalXML !\n");
}

TEST(TestSerialization, SerializationLocalParametrizationQuaternionGlobalPtrXML)
{
  using LocalParametrizationPtr = std::shared_ptr<wolf::LocalParametrizationBase> ;

  {
    std::ofstream os(path_to_io + "local_parametrization_quatg_ptr_serialization.xml");
    cereal::XMLOutputArchive archive(os);

    LocalParametrizationPtr local_param_h =
        std::make_shared<wolf::LocalParametrizationQuaternionGlobal>();

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_quatg_ptr_serialization.xml");
    cereal::XMLInputArchive archive(is);

    LocalParametrizationPtr local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h->getGlobalSize(), 4);
    ASSERT_EQ(local_param_h->getLocalSize(),  3);

    ASSERT_TRUE(
          std::dynamic_pointer_cast<
          wolf::LocalParametrizationQuaternionGlobal>(local_param_h) != nullptr);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationQuaternionGlobalPtrXML !\n");
}

TEST(TestSerialization, SerializationLocalParametrizationQuaternionGlobalJSON)
{
  {
    std::ofstream os(path_to_io + "local_parametrization_quatg_serialization.json");
    cereal::JSONOutputArchive archive(os);

    wolf::LocalParametrizationQuaternionGlobal local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_quatg_serialization.json");
    cereal::JSONInputArchive archive(is);

    wolf::LocalParametrizationQuaternionGlobal local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h.getGlobalSize(), 4);
    ASSERT_EQ(local_param_h.getLocalSize(),  3);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationQuaternionGlobalJSON !\n");
}

TEST(TestSerialization, SerializationLocalParametrizationQuaternionGlobalPtrJSON)
{
  using LocalParametrizationPtr = std::shared_ptr<wolf::LocalParametrizationBase> ;

  {
    std::ofstream os(path_to_io + "local_parametrization_quatg_ptr_serialization.json");
    cereal::JSONOutputArchive archive(os);

    LocalParametrizationPtr local_param_h =
        std::make_shared<wolf::LocalParametrizationQuaternionGlobal>();

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_quatg_ptr_serialization.json");
    cereal::JSONInputArchive archive(is);

    LocalParametrizationPtr local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h->getGlobalSize(), 4);
    ASSERT_EQ(local_param_h->getLocalSize(),  3);

    ASSERT_TRUE(
          std::dynamic_pointer_cast<
          wolf::LocalParametrizationQuaternionGlobal>(local_param_h) != nullptr);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationQuaternionGlobalPtrJSON !\n");
}

TEST(TestSerialization, SerializationLocalParametrizationQuaternionGlobalBIN)
{
  {
    std::ofstream os(path_to_io + "local_parametrization_quatg_serialization.bin");
    cereal::BinaryOutputArchive archive(os);

    wolf::LocalParametrizationQuaternionGlobal local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_quatg_serialization.bin");
    cereal::BinaryInputArchive archive(is);

    wolf::LocalParametrizationQuaternionGlobal local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h.getGlobalSize(), 4);
    ASSERT_EQ(local_param_h.getLocalSize(),  3);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationQuaternionGlobalBIN !\n");
}

TEST(TestSerialization, SerializationLocalParametrizationQuaternionGlobalPtrBIN)
{
  using LocalParametrizationPtr = std::shared_ptr<wolf::LocalParametrizationBase> ;

  {
    std::ofstream os(path_to_io + "local_parametrization_quatg_ptr_serialization.bin");
    cereal::BinaryOutputArchive archive(os);

    LocalParametrizationPtr local_param_h =
        std::make_shared<wolf::LocalParametrizationQuaternionGlobal>();

    ASSERT_NO_THROW( archive( local_param_h ) );
  }

  {
    std::ifstream is(path_to_io + "local_parametrization_quatg_ptr_serialization.bin");
    cereal::BinaryInputArchive archive(is);

    LocalParametrizationPtr local_param_h;

    ASSERT_NO_THROW( archive( local_param_h ) );

    ASSERT_EQ(local_param_h->getGlobalSize(), 4);
    ASSERT_EQ(local_param_h->getLocalSize(),  3);

    ASSERT_TRUE(
          std::dynamic_pointer_cast<
          wolf::LocalParametrizationQuaternionGlobal>(local_param_h) != nullptr);
  }

  PRINTF("All good at TestSerialization::SerializationLocalParametrizationQuaternionGlobalPtrBIN !\n");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
