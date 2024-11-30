# Butler_robot
The system has two main components:

    Task Manager (task_manager.py): This handles the list of orders, the states of the orders, and coordinates task flow. It publishes navigation goals to the robot, handles order statuses, and processes timeouts and cancellations.
    Navigation Node (move_base): The navigation system in ROS, which is responsible for moving the robot between various locations based on goals received from the task manager.

    rosrun butler_robot task_manager.py _locations_file:=/home/siddharth/butler_ws/src/butler_robot/config/locations.yaml
    rosrun butler_robot navigation_node.py
