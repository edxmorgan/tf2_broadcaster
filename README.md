# tf2_broadcaster

Ros2_control plugin for broadcasting transformed states from sensor

## Library Details

- **Pluginlib-Library**: tf2_broadcaster
- **Plugin Type**: tf2_broadcaster/Tf2Broadcaster (implements `controller_interface::ControllerInterface`)

## Usage


### Clone the repository into the `src` folder of your ROS2 workspace:

```bash
cd ~/your_workspace/src
git clone https://github.com/edxmorgan/tf2_broadcaster
```

### Include in package.xml
```xml
<exec_depend>tf2_broadcaster</exec_depend>
```

### Add to your robot controllers yaml config file in ros2_control

```yaml
controller_manager:
  ros__parameters:
    update_rate: 150  #Hz

    tf2_broadcaster:
      type: tf2_broadcaster/Tf2Broadcaster


tf2_broadcaster:
  ros__parameters:
    sensor: alphaimu_sensor
    child_frame_id: base_link
    parent_frame_id: world
    position_x_state_interface : position.x 
    position_y_state_interface : position.y
    position_z_state_interface : position.z
    orientation_w_state_interface : orientation.w
    orientation_x_state_interface : orientation.x
    orientation_y_state_interface : orientation.y
    orientation_z_state_interface : orientation.z
```