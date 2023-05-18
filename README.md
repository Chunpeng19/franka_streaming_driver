# Franka robot joint streaming driver
The driver is used to send franka joint state and receive joint command from client.

## Requirement
1. ROS Humble
2. [libfranka](https://github.com/frankaemika/libfranka)

## Run the package
1. After requirement installation and workspace setup, clone and build the package.

2. Run the interface:
```
cd $<workspace>
source install/setup.bash
ros2 run franka_streaming_driver franka_interface robot_ip
```

3. Run the teleop example code (optional):
```
ros2 run franka_streaming_driver joint_teleop
```

## Controller insider interface
The interface is based on joint torque controller in 'libfranka', the commanded torque is

<img src="https://render.githubusercontent.com/render/math?math=\tau_{cmd} = K_p (q_d - q) + K_d (\dot{q}_d - \dot{q}) + \tau_d + \tau_{Coriolis}">

Client obtains the following joint states by subscribing to *"/franka_joint_states"* with a cutomized 'FrankaJointState' message in the package.
- float64[] position ~ joint angles (rad)
- float64[] velocity ~ joint velocity (rad/s)
- float64[] tau ~ joint torque sensor reading (Nm)
- float64[] tau_ext ~ external filtered estimated joint torque (Nm)

Client publishes the joint commands to *"/franka_joint_commands"* with a a cutomized 'FrankaJointCmd' message in the package.
- float64[] position ~ joint desired angles (rad)
- float64[] velocity ~ joint desired velocity (rad/s)
- float64[] effort ~ joint desired torque (Nm)
- float64[] kp ~ joint desired stiffness (Nm/rad)
- float64[] kd ~ joint desired damping (Nm/rad*s)