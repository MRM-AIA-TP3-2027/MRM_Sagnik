











TASK 2: Cashier System Implementation using ROS2










Cashier System ROS 2:
Overview -
This project implements a cashier system using ROS 2.
It uses multiple nodes to generate bills, update inventory, and track system status.

Features -
* Bill generation 
* Inventory tracking 
* Total income calculation 
* Real time communication between nodes

System Structure -
cashier_system/
??? msg/
? ??? Bill.msg
??? src/
? ??? bill_generator.cpp
? ??? inventory_node.cpp
? ??? status_node.cpp
??? CMakeLists.txt
??? package.xml


Message Definition -
string item_name
int32 quantity
float32 price_per_unit

Build Instructions –
cd ~/ROS_Projects/cashier_ws
colcon build
source install/setup.bash

Run Instructions -
Open 3 terminals.
Terminal 1:
ros2 run cashier_system bill_generator
Terminal 2:
ros2 run cashier_system inventory_node
Terminal 3:
ros2 run cashier_system status_node

Example Output -
Bill Generator:
Item: Apple
Quantity: 5
Price: 10
Bill sent

Inventory Node:
Item: Apple
Remaining: 5
Total Income: 50
Status Node:
System running...

Concepts Used -
* ROS 2 Nodes 
* Publisher Subscriber model 
* Custom messages 
* C++ 

Future Improvements -
* Add GUI 
* Prevent negative stock 
* Add database

rqt Graph -









Video Link - https://youtu.be/Aw7Eexlj_Gw
