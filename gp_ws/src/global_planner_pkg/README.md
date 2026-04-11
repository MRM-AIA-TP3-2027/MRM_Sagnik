TARS Autonomous Navigator
ROS 2 + Gazebo Simulation

---

Description of Task

This project implements an autonomous mobile robot that navigates to a target GPS coordinate inside a Gazebo simulation.

Since the GPS and IMU plugins were not functioning reliably, the system uses odometry data from the differential drive plugin as the primary position and orientation source.

The robot performs the following sequence:

* Receives a goal in GPS coordinates
* Converts GPS coordinates into ENU Cartesian coordinates
* Rotates to face the goal
* Moves toward the goal while correcting heading
* Stops when it reaches within a defined tolerance

A finite state machine controls behavior with four states:

* IDLE
* ROTATING
* DRIVING
* DONE

The system also visualizes the goal by spawning a red sphere inside Gazebo.

---

Description of ROS Nodes, Topics, and Messages

Node

tars_navigator

This is the main control node responsible for navigation logic, state transitions, and velocity commands.

---

Subscribed Topics

/tars/odom
Message Type: nav_msgs/msg/Odometry

* Provides robot position and orientation
* Used as the primary localization source
* Orientation is converted from quaternion to yaw

/goal_gps
Message Type: geometry_msgs/msg/Point

* x represents latitude
* y represents longitude
* Triggers navigation when received

---

Published Topics

/tars/cmd_vel
Message Type: geometry_msgs/msg/Twist

* Controls robot motion
* linear.x controls forward speed
* angular.z controls rotation

/tars/distance_remaining
Message Type: std_msgs/msg/Float64

* Distance between robot and goal

/tars/heading_error
Message Type: std_msgs/msg/Float64

* Angular difference between robot heading and goal direction

/tars/cross_track_error
Message Type: std_msgs/msg/Float64

* Lateral deviation from ideal path

---

Gazebo Service

/spawn_entity
Service Type: gazebo_msgs/srv/SpawnEntity

* Used to spawn a red spherical marker at the goal position

---

Control Logic

Rotation Phase

* Robot aligns with goal direction
* Uses proportional angular velocity
* Stops when heading error is within tolerance

Driving Phase

* Robot moves forward toward goal
* Angular correction applied to maintain direction
* Speed reduces near goal to avoid overshoot
* Stops when distance is below threshold

Noise Handling

* Low pass filter applied to yaw
* Deadband applied to heading error
* Speed scaling reduces oscillations

---

RQT Graph

Insert your RQT graph screenshot here

The graph should show:

* tars_navigator node
* Connections to /tars/odom
* Publishing to /tars/cmd_vel
* Error topics
* Goal input

Example structure:

/goal_gps → tars_navigator → /tars/cmd_vel
/tars/odom → tars_navigator

---

Link to YouTube Video

Add your demonstration video link here

Example:
https://youtube.com/your_video_link

The video should demonstrate:

* Robot receiving a goal
* Rotation toward goal
* Smooth navigation
* Goal reached and stop behavior

---

How to Run

1. Launch Gazebo world
2. Spawn robot model
3. Run navigation node

Command:

ros2 run <your_package_name> tars_navigator

4. Publish a goal

Example:

ros2 topic pub /goal_gps geometry_msgs/msg/Point "{x: 12.9717, y: 77.5947, z: 0.0}"

---

Expected Output

* Robot rotates toward goal
* Moves smoothly without jitter
* Slows down near target
* Stops within tolerance
* Red marker visible at goal

---

Youtube Video Link : https://youtu.be/JsH9aVqCl5w
