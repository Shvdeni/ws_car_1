# ws_car_1 Camera Target Car

ROS 2/Gazebo Sim project for a small three-wheel car in a 1.5 x 1.5 meter room.
The car has a front camera, scans the room for a red sphere, drives toward it,
scores when it reaches the sphere, hides the sphere, respawns it in a new random
location, and starts scanning again.

The launch also opens a `Touch car front camera` OpenCV window. This window
shows what the car camera sees, with a detection overlay: bounding box, center
point, class, confidence, score, and controller state.

## Packages

- `touch_car_description`: robot URDF/Xacro and SDF model with differential drive, bumper, and front camera.
- `touch_car_gazebo`: Gazebo Sim room world, red sphere target, bridges, and launch file.
- `touch_car_control`: ROS 2 Python controller, detection overlay publisher, and camera viewer.

Each package has an `INFO.txt` with a short purpose and run commands.

## Run

This project targets ROS 2 Jazzy with Gazebo Sim through `ros_gz`.

Install the Gazebo integration once:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-cv-bridge
```

Then run:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch touch_car_gazebo room.launch.py
```

## Camera Topics

- Raw car camera: `/touch_car/front_camera/image`
- Annotated detection view: `/touch_car/front_camera/detections`

The viewer node subscribes to the annotated topic by default.

## Detection Model

The trained model file is installed from:

```text
src/touch_car_control/best.pt
```

If `ultralytics` is installed, the controller loads `best.pt` and uses YOLO
detections. If it is not installed, the controller falls back to an HSV red
detector so the simulation still works.
