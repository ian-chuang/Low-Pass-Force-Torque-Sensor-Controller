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

#include "low_pass_force_torque_sensor_controller/low_pass_force_torque_sensor_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace low_pass_force_torque_sensor_controller
{

  bool LowPassForceTorqueSensorController::init(hardware_interface::ForceTorqueSensorInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
  {
    std::fill(wrench_.begin(), wrench_.end(), 0);

    if (!controller_nh.getParam("name", sensor_name_))
    {
      ROS_ERROR_STREAM("Failed to load " << controller_nh.getNamespace() + "/name"
                                         << " from parameter server");
      return false;
    }

    // get publishing period
    if (!controller_nh.getParam("publish_rate", publish_rate_))
    {
      ROS_ERROR_STREAM("Failed to load " << controller_nh.getNamespace() + "/publish_rate"
                                         << " from parameter server");
      return false;
    }

    // sensor handle
    sensor_ = hw->getHandle(sensor_name_);

    // Connect dynamic reconfigure and overwrite the default values with values
    // on the parameter server. This is done automatically if parameters with
    // the according names exist.
    callback_type_ = std::bind(
        &LowPassForceTorqueSensorController::dynamicReconfigureCallback, this, std::placeholders::_1, std::placeholders::_2);
    dyn_conf_server_.reset(new dynamic_reconfigure::Server<Config>(controller_nh));
    dyn_conf_server_->setCallback(callback_type_);

    // realtime publisher
    RtPublisherPtr wrench_pub(new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(root_nh, sensor_name_, 4));
    realtime_wrench_pub_ = wrench_pub;
    RtPublisherPtr filter_pub(new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(root_nh, sensor_name_ + "_filtered", 4));
    realtime_filter_pub_ = filter_pub;

    return true;
  }

  void LowPassForceTorqueSensorController::starting(const ros::Time &time)
  {
    last_publish_time_ = time;
  }

  void LowPassForceTorqueSensorController::update(const ros::Time &time, const ros::Duration & /*period*/)
  {
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
    {
      std::string frame_id = sensor_.getFrameId();
      wrench_[0] = sensor_.getForce()[0];
      wrench_[1] = sensor_.getForce()[1];
      wrench_[2] = sensor_.getForce()[2];
      wrench_[3] = sensor_.getTorque()[0];
      wrench_[4] = sensor_.getTorque()[1];
      wrench_[5] = sensor_.getTorque()[2];

      std::array<double, 6> filtered_wrench;
      for (size_t i = 0; i < 6; ++i)
      {
        filtered_wrench[i] = filters_[i].filter(wrench_[i]);
      }

      bool published = false;

      // try to publish
      if (realtime_wrench_pub_->trylock())
      {
        published = true;

        // populate message
        realtime_wrench_pub_->msg_.header.stamp = time;
        realtime_wrench_pub_->msg_.header.frame_id = frame_id;

        realtime_wrench_pub_->msg_.wrench.force.x = wrench_[0];
        realtime_wrench_pub_->msg_.wrench.force.y = wrench_[1];
        realtime_wrench_pub_->msg_.wrench.force.z = wrench_[2];
        realtime_wrench_pub_->msg_.wrench.torque.x = wrench_[3];
        realtime_wrench_pub_->msg_.wrench.torque.y = wrench_[4];
        realtime_wrench_pub_->msg_.wrench.torque.z = wrench_[5];

        realtime_wrench_pub_->unlockAndPublish();
      }

      // try to publish
      if (realtime_filter_pub_->trylock()) {
        published = true;

        // populate message
        realtime_filter_pub_->msg_.header.stamp = time;
        realtime_filter_pub_->msg_.header.frame_id = frame_id;

        realtime_filter_pub_->msg_.wrench.force.x = filtered_wrench[0];
        realtime_filter_pub_->msg_.wrench.force.y = filtered_wrench[1];
        realtime_filter_pub_->msg_.wrench.force.z = filtered_wrench[2];
        realtime_filter_pub_->msg_.wrench.torque.x = filtered_wrench[3];
        realtime_filter_pub_->msg_.wrench.torque.y = filtered_wrench[4];
        realtime_filter_pub_->msg_.wrench.torque.z = filtered_wrench[5];

        realtime_filter_pub_->unlockAndPublish();
      }

      if (published) {
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);
      }
    }
  }

  void LowPassForceTorqueSensorController::stopping(const ros::Time & /*time*/)
  {
  }

  void LowPassForceTorqueSensorController::dynamicReconfigureCallback(Config& config,
                                                                              uint32_t level)
  {
    // Low-pass filters for the joint positions
    filters_.clear();
    for (size_t i = 0; i < 6; ++i)
    {
      filters_.emplace_back(config.low_pass_filter_coeff);
    }
    for (size_t i = 0; i < 6; ++i)
    {
      filters_[i].reset(wrench_[i]);
    }

    ROS_INFO("%s low-pass filter coefficients changed to: %f", sensor_name_.c_str(), config.low_pass_filter_coeff);
  }

}

PLUGINLIB_EXPORT_CLASS(low_pass_force_torque_sensor_controller::LowPassForceTorqueSensorController, controller_interface::ControllerBase)
