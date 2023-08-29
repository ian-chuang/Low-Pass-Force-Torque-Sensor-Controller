# Low-Pass-Force-Torque-Sensor-Controller

The **Low-Pass-Force-Torque-Sensor-Controller** is a ROS package designed to facilitate the control of force-torque sensors using the **ros_control** framework. It includes an integrated low-pass filter to enhance the accuracy of the sensor readings. This controller is an extension of the original [ros_controllers](https://github.com/ros-controls/ros_controllers) repository, featuring specific modifications to expand its capabilities.

## Changes and Features

- Added a first-order Butterworth low-pass filter to the controller. This filter is applied to the sensor's raw force-torque data. In addition to the unfiltered wrench data, the filtered wrench data is published on the `<ft-sensor-name>_filtered` topic.
- The strength of the low-pass filter can be adjusted dynamically using the dynamic reconfigure feature. Increasing the filter coefficient value results in more noise being filtered out from the sensor readings.
- The original `force_torque_sensor_controller` published all wrenches available in the ROS hardware interface. The `low_pass_force_torque_sensor_controller` has been modified to publish a single wrench, which is specified using the `name` parameter.

## Integration

To integrate the low-pass force-torque sensor controller into your `ros_control` configuration, follow these steps:

1. Examine the example configuration and launch files provided in the following locations:
   - Configuration YAML: `config/low_pass_force_torque_sensor_controller.yaml`
   - Launch File: `launch/low_pass_force_torque_sensor_controller.launch`

2. Configuration YAML (`config/low_pass_force_torque_sensor_controller.yaml`):

```yaml
low_pass_force_torque_sensor_controller:
  type: force_torque_sensor_controller/LowPassForceTorqueSensorController  # Specify the controller type as LowPassForceTorqueSensorController
  name: wrench  # Specify the name of the force torque sensor interface to be published
  low_pass_filter_coeff: 500 # coefficient for filter strength, higher value -> more filtering 
  publish_rate: 50  # Specify the rate (in Hz) at which the controller publishes data
```

3. Launch File (`launch/low_pass_force_torque_sensor_controller.launch`):

```xml
<?xml version="1.0"?>
<launch>
  <!-- Load configuration -->
  <rosparam command="load" file="$(find low_pass_force_torque_sensor_controller)/low_pass_force_torque_sensor_controller.yaml" />
  <!-- Spawn controller -->
  <node name="low_pass_force_torque_sensor_controller_spawner" pkg="controller_manager" type="spawner" output="screen" args="low_pass_force_torque_sensor_controller" />
</launch>
```

## Dynamic Reconfigure

To modify the behavior of the controller using dynamic reconfigure:

1. Launch the ROS **rqt_gui** tool by executing `rosrun rqt_gui rqt_gui`.

2. Within **rqt_gui**, navigate to `Plugins -> Configuration -> Dynamic Reconfigure`.

3. Experiment with different values of the low-pass coefficient to observe the impact on the filtered sensor readings.

For further details and usage examples, please refer to the original [ros_controllers](https://github.com/ros-controls/ros_controllers) repository.