# ws_car_1 Touch Bumper Car

ROS 2/Gazebo project for a small 15 cm three-wheel car in a 1.5 x 1.5 meter
room. The rear wheels are motorized by a differential drive plugin. A front
contact bumper reports collisions; the controller backs up about 5 cm, turns by
a random angle, and drives forward again.

## Packages

- `touch_car_description`: robot URDF/Xacro model.
- `touch_car_gazebo`: Gazebo room world and launch file.
- `touch_car_control`: ROS 2 Python bumper reaction node.

Each package has an `INFO.txt` with a short purpose and run commands.

## Run

This project targets ROS 2 Jazzy with Gazebo Sim through `ros_gz`.

Install the Gazebo integration once:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz
```

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch touch_car_gazebo room.launch.py
```

The launch file starts Gazebo Classic, spawns the robot, and starts the bumper
driver node.
