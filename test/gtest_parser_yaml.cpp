#include "utils_gtest.h"
#include "base/converter.h"
#include "../hello_plugin/parser_yaml.hpp"

using namespace std;
using namespace wolf;

TEST(ParserYAML, RegularParse)
{
  string file = "";
  file = "../test/params1.yaml";
  parserYAML parser = parserYAML(file);
  parser.parse();
  auto params = parser.getParams();
  for(auto it : params)
    cout << it.first << " %% " << it.second << endl;
  ASSERT_EQ(params["odom/intrinsic/k_rot_to_rot"], "0.1");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
