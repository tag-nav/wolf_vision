#include "utils_gtest.h"
#include "base/converter.h"
#include "../hello_plugin/parser_yaml.hpp"

using namespace std;
using namespace wolf;

parserYAML parse(string _file)
{
  parserYAML parser = parserYAML(_file);
  parser.parse();
  return parser;
}

TEST(ParserYAML, RegularParse)
{
  auto parser = parse("../test/params1.yaml");
  auto params = parser.getParams();
  // for(auto it : params)
  //   cout << it.first << " %% " << it.second << endl;
  ASSERT_EQ(params["odom/intrinsic/k_rot_to_rot"], "0.1");
  ASSERT_EQ(params["processor1/sensorname"], "odom");
}
TEST(ParserYAML, ParseMap)
{
  auto parser = parse("../test/params2.yaml");
  auto params = parser.getParams();
  ASSERT_EQ(params["processor1/mymap"], "[{k1:v1},{k2:v2},{k3:[v3,v4,v5]}]");
}
TEST(ParserYAML, JumpFile)
{
  auto parser = parse("../test/params3.yaml");
  auto params = parser.getParams();
  ASSERT_EQ(params["my_proc_test/max_buff_length"], "100");
  ASSERT_EQ(params["my_proc_test/jump/voting_active"], "false");
}
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
