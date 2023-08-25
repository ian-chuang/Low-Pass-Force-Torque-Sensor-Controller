///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author: Adolfo Rodriguez Tsouroukdissian

#pragma once


#include <controller_interface/controller.h>
#include <geometry_msgs/WrenchStamped.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <memory>
#include <realtime_tools/realtime_publisher.h>

// Low pass filter
#include <low_pass_force_torque_sensor_controller/low_pass_filter.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <low_pass_force_torque_sensor_controller/LowPassFilterConfig.h>

namespace low_pass_force_torque_sensor_controller
{

// this controller gets access to the ForceTorqueSensorInterface
class LowPassForceTorqueSensorController: public controller_interface::Controller<hardware_interface::ForceTorqueSensorInterface>
{
public:
  LowPassForceTorqueSensorController(){}

  virtual bool init(hardware_interface::ForceTorqueSensorInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
  virtual void stopping(const ros::Time& /*time*/);

private:

  // Force control specific dynamic reconfigure
  typedef low_pass_force_torque_sensor_controller::LowPassFilterConfig Config;

  void dynamicReconfigureCallback(Config& config, uint32_t level);

  std::shared_ptr<dynamic_reconfigure::Server<Config> > dyn_conf_server_;
  dynamic_reconfigure::Server<Config>::CallbackType callback_type_;

  std::string sensor_name_;
  hardware_interface::ForceTorqueSensorHandle sensor_;
  std::array<double, 6> wrench_;
  typedef std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> > RtPublisherPtr;
  std::vector<LowPassFilter> filters_;
  RtPublisherPtr realtime_wrench_pub_;
  RtPublisherPtr realtime_filter_pub_;
  ros::Time last_publish_time_;
  double publish_rate_;
};

}
