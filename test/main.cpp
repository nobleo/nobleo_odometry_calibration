#include <glog/logging.h>
#include <gtest/gtest.h>

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
