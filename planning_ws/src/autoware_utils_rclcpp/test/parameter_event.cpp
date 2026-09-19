// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#include <autoware_utils_rclcpp/parameter_event.hpp>
#include <gtest/gtest.h>
using rclcpp::Parameter;
using autoware_utils::update_param;
TEST(ParameterEvent, ScalarValuesActuallyUpdate) {
  const std::vector<Parameter> values{{"module.double",1.25},{"module/int",3},{"module.bool",true},{"module.text",std::string("new")}};
  double d=0;int i=0;bool b=false;std::string s="old";
  EXPECT_TRUE(update_param(values,"module/double",d));EXPECT_DOUBLE_EQ(d,1.25);
  EXPECT_TRUE(update_param(values,"module.int",i));EXPECT_EQ(i,3);
  EXPECT_TRUE(update_param(values,"module/bool",b));EXPECT_TRUE(b);
  EXPECT_TRUE(update_param(values,"module/text",s));EXPECT_EQ(s,"new");
}
TEST(ParameterEvent, MissingAndInvalidTypeDoNotOverwrite) {
  const std::vector<Parameter> values{{"value",true}};double number=9.0;
  EXPECT_FALSE(update_param(values,"missing",number));EXPECT_DOUBLE_EQ(number,9.0);
  EXPECT_THROW(update_param(values,"value",number),std::invalid_argument);EXPECT_DOUBLE_EQ(number,9.0);
}
TEST(ParameterEvent, ArraysRoundTripAndEmptyArraysClear) {
  const std::vector<Parameter> values{{"d",std::vector<double>{1.0,2.0}},
    {"s",std::vector<std::string>{"a","b"}},{"b",std::vector<bool>{true,false}},
    {"i",std::vector<int>{1,2}},{"empty",std::vector<double>{}}};
  std::vector<double>d;std::vector<std::string>s;std::vector<bool>b;std::vector<int>i;
  ASSERT_TRUE(update_param(values,"d",d));EXPECT_EQ(d,(std::vector<double>{1,2}));
  ASSERT_TRUE(update_param(values,"s",s));EXPECT_EQ(s,(std::vector<std::string>{"a","b"}));
  ASSERT_TRUE(update_param(values,"b",b));EXPECT_EQ(b,(std::vector<bool>{true,false}));
  ASSERT_TRUE(update_param(values,"i",i));EXPECT_EQ(i,(std::vector<int>{1,2}));
  ASSERT_TRUE(update_param(values,"empty",d));EXPECT_TRUE(d.empty());
}
TEST(ParameterEvent, InvalidArrayElementPreservesWholeOldValue) {
  XmlRpc::XmlRpcValue raw;raw.setSize(2);raw[0]=1.0;raw[1]=true;
  const std::vector<Parameter> values{Parameter("array",raw)};std::vector<double> output{8,9};
  EXPECT_THROW(update_param(values,"array",output),std::invalid_argument);
  EXPECT_EQ(output,(std::vector<double>{8,9}));
}
TEST(ParameterEvent, NativePayloadIsOwnedAndFirstMatchingNameWins) {
  XmlRpc::XmlRpcValue raw(1.5);Parameter value("x",raw);raw=9.0;
  EXPECT_DOUBLE_EQ(value.get_value<double>(),1.5);
  double output=0;EXPECT_TRUE(update_param(std::vector<Parameter>{value,Parameter("x",2.0)},"x",output));
  EXPECT_DOUBLE_EQ(output,1.5);
}
int main(int argc,char ** argv) {
  ::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
