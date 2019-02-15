#include "utils_gtest.h"
#include "base/converter.h"
#include "base/wolf.h"
#include "../hello_plugin/parser_yaml.hpp"
#include "base/params_server.hpp"

using namespace std;
using namespace wolf;

parserYAML parse(string _file)
{
  parserYAML parser = parserYAML(_file);
  parser.parse();
  return parser;
}

TEST(ParamsServer, Default)
{
  std::string wolf_root = _WOLF_ROOT_DIR;
  auto parser = parse(wolf_root + "/test/params1.yaml");
  auto params = parser.getParams();
  paramsServer server = paramsServer(params, parser.sensorsSerialization(), parser.processorsSerialization());
  EXPECT_EQ(server.getParam<double>("should_not_exist", "2.6"), 2.6);
  EXPECT_EQ(server.getParam<bool>("my_proc_test/voting_active", "true"), false);
  EXPECT_NE(server.getParam<unsigned int>("my_proc_test/time_tolerance", "23"), 23);
}
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
