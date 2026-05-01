# super_planner

```bash
sudo apt-get install ros-noetic-mavros* ros-noetic-octomap* ros-noetic-plotjuggler* ros-noetic-joy 

## Ground car mode

SUPER can also publish commands for a four-wheel ground vehicle. Use one of the
ground-car example configs:

- `config/ground_car_ros1.yaml`
- `config/ground_car_ros2.yaml`

The switch is:

```yaml
traj_opt:
  vehicle_type: "ground_car"

fsm:
  cmd_vel_topic: "/cmd_vel"
  click_height: 0.0
```

In `ground_car` mode, the planner keeps the position, velocity, acceleration,
jerk and safe-corridor constraints, disables quadrotor-specific angular-rate
and thrust penalties, and publishes `geometry_msgs/Twist` on `cmd_vel_topic`.
The original quadrotor `PositionCommand` output remains the default when
`vehicle_type` is `quadrotor`.
sudo apt-get install libdw-dev
```


```bash
  mkdir build
  cd build/
  cmake -DBUILD_SHARED_LIBS=TRUE ..
  make
  sudo make install
```
