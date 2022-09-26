/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#include <gtest/gtest.h>
#include "capabilities/client.h"


class ClientTest : public capabilities::Client
{
public:
  ClientTest() : capabilities::Client("/") {}
  void usedCapabilitiesAdd(const std::string& capability_interface)
  {
    used_capabilities_.insert(capability_interface);
  }
};

TEST(CapabilitiesClient, unitTest)
{
  ClientTest c;
  try
  {
    c.waitForServices(ros::Duration(0.1), {"invalid_service"});
  }
  catch (capabilities::ServiceInvalidException& e)
  {
  }
  EXPECT_EQ(c.waitForServices(ros::Duration(0.1)), false);
  c.establishBond(ros::Duration(0.1));
  c.usedCapabilitiesAdd("some_pkg/SomeCap");
  try
  {
    c.freeCapability("some_pkg/SomeCap", ros::Duration(0.1));
  }
  catch (capabilities::ServiceNotAvailableException& e)
  {
  }
}

TEST(CapabilitiesClient, moduleTest)
{
  capabilities::Client c;
  c.waitForServices(ros::Duration(3.0));
  c.useCapability("minimal_pkg/Minimal", "minimal_pkg/minimal");
  c.useCapability("minimal_pkg/Minimal", "minimal_pkg/minimal");
  // for (int i = 0; i < 10; i++) ros::Duration(1.0).sleep(), ros::spinOnce();
  c.freeCapability("minimal_pkg/Minimal");
  try
  {
    c.freeCapability("not_a_pkg/NotACap");
  }
  catch (capabilities::CapabilityNotInUseException& e)
  {
  }
  c.shutdown();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "capabilities_client_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
