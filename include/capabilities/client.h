/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

/*
Provides a simple C++ interface for interacting with the capability server.

Typical usage::

    >>> #include <capabilities/client.h>
    >>> capabilities::Client client;
    >>> // Use the line below if the capability_server has a different name
    >>> // capabilities::Client client = capabilities::Client("/capability_server_node_name");
    >>> if (!client.waitForServices(3.0)) {  // Wait upto 3.0 seconds for the required ROS services
    ...     ROS_ERROR("capability_server, called '%s', failed to come up.", client.name_.c_str());
    ...     exit(1);
    ... }
    >>> client.useCapability('foo_pkg/Foo');
    >>> client.useCapability('foo_pkg/Foo');
    >>> client.freeCapability('foo_pkg/Foo');
    >>> client.shutdown();
*/

#ifndef CAPABILITIES_CLIENT_H
#define CAPABILITIES_CLIENT_H

#include <map>
#include <memory>
#include <set>

#include <ros/ros.h>

#include <bondcpp/bond.h>
#include <capabilities/EstablishBond.h>
#include <capabilities/FreeCapability.h>
#include <capabilities/UseCapability.h>


namespace capabilities
{

class ServiceInvalidException : public ros::Exception
{
public:
  explicit ServiceInvalidException(const std::string& service_name) :
    ros::Exception("Service '" + service_name + "' is not a valid service and cannot be waited on")
  {}
};


class ServiceNotAvailableException : public ros::Exception
{
public:
  explicit ServiceNotAvailableException(const std::string& service_name) :
    ros::Exception("Required ROS service '" + service_name + "' not available")
  {}
};


class CannotEstablishBondException : public ros::Exception
{
public:
  CannotEstablishBondException() :
    ros::Exception("Failed to establish bond.")
  {}
};


class CapabilityNotRunningException : public ros::Exception
{
public:
  explicit CapabilityNotRunningException(const std::string& interface) :
    ros::Exception("Capability interface '" + interface + "' is not running")
  {}
};


class CapabilityNotInUseException : public ros::Exception
{
public:
  explicit CapabilityNotInUseException(const std::string& interface) :
    ros::Exception("Capability interface '" + interface + "' not previously used")
  {}
};


class Client
{
public:
  explicit Client(const std::string& capability_server_node_name = "/capability_server") :
    name_(capability_server_node_name)
  {
    ros::NodeHandle nh(name_);
    services_["establish_bond"] = nh.serviceClient<capabilities::EstablishBond>("establish_bond");
    services_["free_capability"] = nh.serviceClient<capabilities::FreeCapability>("free_capability");
    services_["use_capability"] = nh.serviceClient<capabilities::UseCapability>("use_capability");
  }

  bool waitForServices(ros::Duration timeout = ros::Duration(-1), const std::set<std::string>& services = std::set<std::string>())
  {
    if (services.size())
    {
      for (auto& key : services)
      {
        if (services_.find(key) == services_.end())
          throw ServiceInvalidException(key);
        if (!_waitForService(services_[key], timeout))
          return false;
      }
    }
    else for (auto& key : services_)
    {
      if (!_waitForService(key.second, timeout))
        return false;
    }
    return true;
  }

  std::string establishBond(ros::Duration timeout = ros::Duration(-1))
  {
    ros::ServiceClient &client = services_["establish_bond"];
    if (!_waitForService(client, timeout)) return "";

    capabilities::EstablishBond srv;
    if (!client.call(srv)) return "";

    bond_id_ = srv.response.bond_id;
    bond_ = std::make_shared<bond::Bond>(name_ + "/bonds", bond_id_);
    bond_->start();

    ros::Time start_time = ros::Time::now();
    while (ros::ok() && !bond_->waitUntilFormed(ros::Duration(0.0)))
    {
      if (timeout >= ros::Duration(0))
      {
        if ((ros::Time::now() - start_time) >= timeout) return "";
      }
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }

    return bond_id_;
  }

  bool freeCapability(const std::string& capability_interface, ros::Duration timeout = ros::Duration(-1))
  {
    if (used_capabilities_.find(capability_interface) == used_capabilities_.end())
    {
      ROS_ERROR("Cannot free capability interface '%s', because it was not first used.", capability_interface.c_str());
      throw CapabilityNotInUseException(capability_interface);
    }

    ros::ServiceClient &client = services_["free_capability"];
    if (!_waitForService(client, timeout))
      throw ServiceNotAvailableException(client.getService().c_str());

    capabilities::FreeCapability srv;
    srv.request.capability = capability_interface;
    srv.request.bond_id = bond_id_;
    if (!client.call(srv))
    {
      // https://github.com/ros/ros_comm/issues/1677
      // currently not possible to check rospy.ServiceException message in C++
      // throw CapabilityNotRunningException(capability_interface);
      return false;
    }
    return true;
  }

  void shutdown()
  {
    bond_.reset();
  }

  bool useCapability(const std::string& capability_interface, const std::string& preferred_provider = "", ros::Duration timeout = ros::Duration(-1))
  {
    // If no bond has been established, establish one first
    if (!bond_)
    {
      if (establishBond(timeout) == "")
        throw CannotEstablishBondException();
    }

    ros::ServiceClient &client = services_["use_capability"];
    if (!_waitForService(client, timeout))
      throw ServiceNotAvailableException(client.getService().c_str());

    capabilities::UseCapability srv;
    srv.request.capability = capability_interface;
    srv.request.preferred_provider = preferred_provider;
    srv.request.bond_id = bond_id_;
    client.call(srv);
    used_capabilities_.insert(capability_interface);
    return true;
  }

protected:
  bool _waitForService(ros::ServiceClient &service, ros::Duration timeout)
  {
    // if (!ros::service::waitForService(service.getService(), timeout))
    if (!service.waitForExistence(timeout))
    {
      ROS_WARN("Timed out after waiting '%f' seconds for service '%s' to be available.",
               timeout.toSec(), service.getService().c_str());
      return false;
    }
    return true;
  }

public:
  const std::string name_;

protected:
  std::shared_ptr<bond::Bond> bond_;
  std::string bond_id_;
  std::set<std::string> used_capabilities_;
  std::map<std::string, ros::ServiceClient> services_;
};

}  // namespace capabilities

#endif  // CAPABILITIES_CLIENT_H
