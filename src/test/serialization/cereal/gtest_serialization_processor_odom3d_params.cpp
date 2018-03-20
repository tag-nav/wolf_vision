/*
 * gtest_intrinsics_odom2d_serialization.cpp
 *
 *  Created on: Jul 16, 2017
 *      Author: Jeremie Deray
 */

#include "../../utils_gtest.h"

#include "../../../serialization/cereal/serialization_processor_odom3d_params.h"

#include "../../../serialization/cereal/io.h"

#include <cereal/types/memory.hpp>
#include <fstream>

class WolfTestCerealSerializationProcessorOdom3DParams : public testing::Test
{
public:

  WolfTestCerealSerializationProcessorOdom3DParams()
  {
    nb_.name = "NAME";
    //nb_.type = "ODOM 3D";

    nb_.max_time_span   = 1.5;
    nb_.max_buff_length = 55.;
    nb_.dist_traveled   = .25;
    nb_.angle_turned    = .17;
  }

  const std::string path_to_io = "/tmp/";

  wolf::ProcessorParamsOdom3D nb_;
};

TEST_F(WolfTestCerealSerializationProcessorOdom3DParams,
       CerealSerializationProcessorOdom3DParamsXML)
{
  const std::string filename(path_to_io + "params_odom3d_serialization.xml");

  wolf::ProcessorParamsOdom3D nb_save;
  nb_save.name = "NAME2";
  //nb_.type = "ODOM 3D";

  nb_save.max_time_span   = 2.5;
  nb_save.max_buff_length = 52.;
  nb_save.dist_traveled   = .24;
  nb_save.angle_turned    = .18;

  ASSERT_NO_THROW( wolf::save( filename, nb_, nb_save, 10 ) );

  {
    wolf::ProcessorParamsOdom3D nb_load;

    ASSERT_NO_THROW( wolf::load( filename, nb_load ) );

    ASSERT_EQ(nb_load.type, nb_.type);
    ASSERT_EQ(nb_load.name, nb_.name);
    ASSERT_EQ(nb_load.max_time_span,   nb_.max_time_span);
    ASSERT_EQ(nb_load.max_buff_length, nb_.max_buff_length);
    ASSERT_EQ(nb_load.dist_traveled,   nb_.dist_traveled);
    ASSERT_EQ(nb_load.angle_turned,    nb_.angle_turned);

    wolf::ProcessorParamsOdom3D nb_load0, nb_load1;
    int myint;
    ASSERT_NO_THROW( wolf::load( filename, nb_load0, nb_load1, myint ) );

    ASSERT_EQ(nb_load0.type, nb_.type);
    ASSERT_EQ(nb_load0.name, nb_.name);
    ASSERT_EQ(nb_load0.max_time_span,   nb_.max_time_span);
    ASSERT_EQ(nb_load0.max_buff_length, nb_.max_buff_length);
    ASSERT_EQ(nb_load0.dist_traveled,   nb_.dist_traveled);
    ASSERT_EQ(nb_load0.angle_turned,    nb_.angle_turned);

    ASSERT_EQ(nb_load1.type, nb_save.type);
    ASSERT_EQ(nb_load1.name, nb_save.name);
    ASSERT_EQ(nb_load1.max_time_span,   nb_save.max_time_span);
    ASSERT_EQ(nb_load1.max_buff_length, nb_save.max_buff_length);
    ASSERT_EQ(nb_load1.dist_traveled,   nb_save.dist_traveled);
    ASSERT_EQ(nb_load1.angle_turned,    nb_save.angle_turned);

    ASSERT_EQ(myint, 10);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationProcessorOdom3DParams::"
         "CerealSerializationProcessorOdom3DParamsXML !\n");
}

TEST_F(WolfTestCerealSerializationProcessorOdom3DParams,
       CerealSerializationProcessorParamsOdom3DPtrXML)
{
  const std::string filename(path_to_io + "params_odom3d_ptr_serialization.xml");

  {
    wolf::ProcessorParamsBasePtr nb =
        std::make_shared<wolf::ProcessorParamsOdom3D>(nb_);

    ASSERT_NO_THROW( wolf::save( filename, nb ) );
  }

  {
    wolf::ProcessorParamsBasePtr nb;

    ASSERT_NO_THROW( wolf::load( filename, nb ) );

    wolf::ProcessorParamsOdom3DPtr nb_cast =
        std::dynamic_pointer_cast<wolf::ProcessorParamsOdom3D>(nb);

    ASSERT_TRUE(nb_cast != nullptr);

    ASSERT_EQ(nb_cast->type, nb_.type);
    ASSERT_EQ(nb_cast->name, nb_.name);
    ASSERT_EQ(nb_cast->max_time_span,   nb_.max_time_span);
    ASSERT_EQ(nb_cast->max_buff_length, nb_.max_buff_length);
    ASSERT_EQ(nb_cast->dist_traveled,   nb_.dist_traveled);
    ASSERT_EQ(nb_cast->angle_turned,    nb_.angle_turned);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationProcessorOdom3DParams::"
         "CerealSerializationProcessorParamsOdom3DPtrXML !\n");
}

TEST_F(WolfTestCerealSerializationProcessorOdom3DParams,
       CerealSerializationProcessorOdom3DParamsJSON)
{
  const std::string filename(path_to_io + "params_odom3d_serialization.json");

  ASSERT_NO_THROW( wolf::save( filename, nb_ ) );

  wolf::ProcessorParamsOdom3D nb_load;

  ASSERT_NO_THROW( wolf::load( filename, nb_load ) );

  ASSERT_EQ(nb_load.type, nb_.type);
  ASSERT_EQ(nb_load.name, nb_.name);
  ASSERT_EQ(nb_load.max_time_span,   nb_.max_time_span);
  ASSERT_EQ(nb_load.max_buff_length, nb_.max_buff_length);
  ASSERT_EQ(nb_load.dist_traveled,   nb_.dist_traveled);
  ASSERT_EQ(nb_load.angle_turned,    nb_.angle_turned);

  PRINTF("All good at "
         "WolfTestCerealSerializationProcessorOdom3DParams::"
         "CerealSerializationProcessorOdom3DParamsJSON !\n");
}

TEST_F(WolfTestCerealSerializationProcessorOdom3DParams,
       CerealSerializationProcessorParamsOdom3DPtrJSON)
{
  const std::string filename(path_to_io + "params_odom3d_ptr_serialization.json");

  {
    wolf::ProcessorParamsBasePtr nb =
        std::make_shared<wolf::ProcessorParamsOdom3D>(nb_);

    ASSERT_NO_THROW( wolf::save( filename, nb ) );
  }

  {
    wolf::ProcessorParamsBasePtr nb;

    ASSERT_NO_THROW( wolf::load( filename, nb ) );

    wolf::ProcessorParamsOdom3DPtr nb_cast =
        std::dynamic_pointer_cast<wolf::ProcessorParamsOdom3D>(nb);

    ASSERT_TRUE(nb_cast != nullptr);

    ASSERT_EQ(nb_cast->type, nb_.type);
    ASSERT_EQ(nb_cast->name, nb_.name);
    ASSERT_EQ(nb_cast->max_time_span,   nb_.max_time_span);
    ASSERT_EQ(nb_cast->max_buff_length, nb_.max_buff_length);
    ASSERT_EQ(nb_cast->dist_traveled,   nb_.dist_traveled);
    ASSERT_EQ(nb_cast->angle_turned,    nb_.angle_turned);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationProcessorOdom3DParams::"
         "CerealSerializationProcessorParamsOdom3DPtrJSON !\n");
}

TEST_F(WolfTestCerealSerializationProcessorOdom3DParams,
       CerealSerializationProcessorOdom3DParamsBinary)
{
  const std::string filename(path_to_io + "params_odom3d_serialization.bin");

  ASSERT_NO_THROW( wolf::save( filename, nb_ ) );

  wolf::ProcessorParamsOdom3D nb_load;

  ASSERT_NO_THROW( wolf::load( filename, nb_load ) );

  ASSERT_EQ(nb_load.type, nb_.type);
  ASSERT_EQ(nb_load.name, nb_.name);
  ASSERT_EQ(nb_load.max_time_span,   nb_.max_time_span);
  ASSERT_EQ(nb_load.max_buff_length, nb_.max_buff_length);
  ASSERT_EQ(nb_load.dist_traveled,   nb_.dist_traveled);
  ASSERT_EQ(nb_load.angle_turned,    nb_.angle_turned);

  PRINTF("All good at "
         "WolfTestCerealSerializationProcessorOdom3DParams::"
         "CerealSerializationProcessorOdom3DParamsBinary !\n");
}

TEST_F(WolfTestCerealSerializationProcessorOdom3DParams,
       CerealSerializationProcessorParamsOdom3DPtrBinary)
{
  const std::string filename(path_to_io + "params_odom3d_ptr_serialization.bin");

  {
    wolf::ProcessorParamsBasePtr nb =
        std::make_shared<wolf::ProcessorParamsOdom3D>(nb_);

    ASSERT_NO_THROW( wolf::save( filename, nb ) );
  }

  {
    wolf::ProcessorParamsBasePtr nb;

    ASSERT_NO_THROW( wolf::load( filename, nb ) );

    wolf::ProcessorParamsOdom3DPtr nb_cast =
        std::dynamic_pointer_cast<wolf::ProcessorParamsOdom3D>(nb);

    ASSERT_TRUE(nb_cast != nullptr);

    ASSERT_EQ(nb_cast->type, nb_.type);
    ASSERT_EQ(nb_cast->name, nb_.name);
    ASSERT_EQ(nb_cast->max_time_span,   nb_.max_time_span);
    ASSERT_EQ(nb_cast->max_buff_length, nb_.max_buff_length);
    ASSERT_EQ(nb_cast->dist_traveled,   nb_.dist_traveled);
    ASSERT_EQ(nb_cast->angle_turned,    nb_.angle_turned);
  }

  PRINTF("All good at "
         "WolfTestCerealSerializationProcessorOdom3DParams::"
         "CerealSerializationProcessorParamsOdom3DPtrBinary !\n");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
