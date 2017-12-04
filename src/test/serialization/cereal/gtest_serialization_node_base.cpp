/*
 * gtest_node_base_serialization.cpp
 *
 *  Created on: Jul 16, 2017
 *      Author: Jeremie Deray
 */

#include "../../utils_gtest.h"

#include "../../../serialization/cereal/serialization_node_base.h"

#include "../../../serialization/cereal/archives.h"

#include <cereal/types/memory.hpp>
#include <fstream>

class WolfTestCerealSerializationNodeBase : public testing::Test
{
public:

    WolfTestCerealSerializationNodeBase() /*:
      nb(nb_class),
      nb_ptr(std::make_shared<wolf::NodeBase>(nb_class))*/
    {
      //
    }

    const std::string path_to_io = "/tmp/";

    decltype(std::declval<wolf::NodeBase>().getClass()) nb_class = "Foo";
    decltype(std::declval<wolf::NodeBase>().getClass()) nb_type  = "Bar";
    decltype(std::declval<wolf::NodeBase>().getClass()) nb_name  = "Dummy";

    decltype(std::declval<wolf::NodeBase>().nodeId()) id;

//    wolf::NodeBase nb;
//    wolf::NodeBasePtr nb_ptr;
};

TEST_F(WolfTestCerealSerializationNodeBase, CerealSerializationNodeBaseXML)
{
  {
    // This guy has node_id = 1
    wolf::NodeBase nb(nb_class, nb_type, nb_name);

    id = nb.nodeId();

    std::ofstream os(path_to_io + "node_base_serialization.xml");
    cereal::XMLOutputArchive xml_archive(os);

    ASSERT_NO_THROW( xml_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "node_base_serialization.xml");
    cereal::XMLInputArchive xml_archive(is);

    // This guy has node_id = 2
    wolf::NodeBase nb("SuperDummy");

    ASSERT_NO_THROW( xml_archive( nb ) );

    ASSERT_EQ(nb.getClass(), nb_class);
    ASSERT_EQ(nb.getType(),  nb_type);
    ASSERT_EQ(nb.getName(),  nb_name);
    ASSERT_EQ(nb.nodeId(),   id);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationNodeBase::CerealSerializationNodeBaseXML !\n");
}

TEST_F(WolfTestCerealSerializationNodeBase, CerealSerializationNodeBasePtrXML)
{
  {
    // This guy has node_id = 3
    wolf::NodeBasePtr nb = std::make_shared<wolf::NodeBase>(nb_class, nb_type, nb_name);

    id = nb->nodeId();

    std::ofstream os(path_to_io + "node_base_ptr_serialization.xml");
    cereal::XMLOutputArchive xml_archive(os);

    ASSERT_NO_THROW( xml_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "node_base_ptr_serialization.xml");
    cereal::XMLInputArchive xml_archive(is);

    wolf::NodeBasePtr nb;

    ASSERT_NO_THROW( xml_archive( nb ) );

    ASSERT_EQ(nb->getClass(), nb_class);
    ASSERT_EQ(nb->getType(),  nb_type);
    ASSERT_EQ(nb->getName(),  nb_name);
    ASSERT_EQ(nb->nodeId(),   id);

    ASSERT_TRUE(
          std::dynamic_pointer_cast<
          wolf::NodeBase>(nb) != nullptr);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationNodeBase::CerealSerializationNodeBasePtrXML !\n");
}

TEST_F(WolfTestCerealSerializationNodeBase, CerealSerializationNodeBaseJSON)
{
  {
    wolf::NodeBase nb(nb_class, nb_type, nb_name);

    id = nb.nodeId();

    std::ofstream os(path_to_io + "node_base_serialization.json");
    cereal::JSONOutputArchive json_archive(os);

    ASSERT_NO_THROW( json_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "node_base_serialization.json");
    cereal::JSONInputArchive json_archive(is);

    wolf::NodeBase blank("This guy has node_id = 1");
    wolf::NodeBase nb("SuperDummy");

    ASSERT_NO_THROW( json_archive( nb ) );

    ASSERT_EQ(nb.getClass(), nb_class);
    ASSERT_EQ(nb.getType(),  nb_type);
    ASSERT_EQ(nb.getName(),  nb_name);
    ASSERT_EQ(nb.nodeId(),   id);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationNodeBase::CerealSerializationNodeBaseJSON !\n");
}

TEST_F(WolfTestCerealSerializationNodeBase, CerealSerializationNodeBasePtrJSON)
{
  {
    wolf::NodeBasePtr nb = std::make_shared<wolf::NodeBase>(nb_class, nb_type, nb_name);

    id = nb->nodeId();

    std::ofstream os(path_to_io + "node_base_ptr_serialization.json");
    cereal::JSONOutputArchive json_archive(os);

    ASSERT_NO_THROW( json_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "node_base_ptr_serialization.json");
    cereal::JSONInputArchive json_archive(is);

    wolf::NodeBasePtr nb;

    ASSERT_NO_THROW( json_archive( nb ) );

    ASSERT_EQ(nb->getClass(), nb_class);
    ASSERT_EQ(nb->getType(),  nb_type);
    ASSERT_EQ(nb->getName(),  nb_name);
    ASSERT_EQ(nb->nodeId(),   id);

    ASSERT_TRUE(
          std::dynamic_pointer_cast<
          wolf::NodeBase>(nb) != nullptr);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationNodeBase::CerealSerializationNodeBasePtrJSON !\n");
}

TEST_F(WolfTestCerealSerializationNodeBase, CerealSerializationNodeBaseBinary)
{
  {
    wolf::NodeBase nb(nb_class, nb_type, nb_name);

    id = nb.nodeId();

    std::ofstream os(path_to_io + "node_base_serialization.bin");
    cereal::BinaryOutputArchive bin_archive(os);

    ASSERT_NO_THROW( bin_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "node_base_serialization.bin");
    cereal::BinaryInputArchive bin_archive(is);

    wolf::NodeBase blank("This guy has node_id = 1");
    wolf::NodeBase nb("SuperDummy");

    ASSERT_NO_THROW( bin_archive( nb ) );

    ASSERT_EQ(nb.getClass(), nb_class);
    ASSERT_EQ(nb.getType(),  nb_type);
    ASSERT_EQ(nb.getName(),  nb_name);
    ASSERT_EQ(nb.nodeId(),   id);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationNodeBase::CerealSerializationNodeBaseBinary !\n");
}

TEST_F(WolfTestCerealSerializationNodeBase, CerealSerializationNodeBasePtrBinary)
{
  {
    wolf::NodeBasePtr nb = std::make_shared<wolf::NodeBase>(nb_class, nb_type, nb_name);

    id = nb->nodeId();

    std::ofstream os(path_to_io + "node_base_ptr_serialization.bin");
    cereal::BinaryOutputArchive bin_archive(os);

    ASSERT_NO_THROW( bin_archive( nb ) );
  }

  {
    std::ifstream is(path_to_io + "node_base_ptr_serialization.bin");
    cereal::BinaryInputArchive bin_archive(is);

    wolf::NodeBasePtr nb;

    ASSERT_NO_THROW( bin_archive( nb ) );

    ASSERT_EQ(nb->getClass(), nb_class);
    ASSERT_EQ(nb->getType(),  nb_type);
    ASSERT_EQ(nb->getName(),  nb_name);
    ASSERT_EQ(nb->nodeId(),   id);

    ASSERT_TRUE(
          std::dynamic_pointer_cast<
          wolf::NodeBase>(nb) != nullptr);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationNodeBase::CerealSerializationNodeBasePtrBinary !\n");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
