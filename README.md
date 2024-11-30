# Butler_robot
The system has two main components:

    Task Manager (task_manager.py): This handles the list of orders, the states of the orders, and coordinates task flow. It publishes navigation goals to the robot, handles order statuses, and processes timeouts and cancellations.
    Navigation Node (move_base): The navigation system in ROS, which is responsible for moving the robot between various locations based on goals received from the task manager.

    
    The locations for the delivery tasks are defined in a YAML file, which contains the x and y coordinates for each location. This file can be loaded at runtime by the task manager.

    The task manager maintains a queue of orders. Each order has a status (pending, in-progress, completed, canceled) and associated location. The task manager processes each order sequentially by sending the robot to the designated location      and updating the order's status.

    Timeout for each task to handle situations where the robot doesn't reach its destination within the expected time. This can be done by monitoring the elapsed time since sending a goal, and if the goal is not reached within the time     limit, the task manager marks the order as failed and retries.


    Test the system in Gazebo for all scenarios listed in the problem statement.
