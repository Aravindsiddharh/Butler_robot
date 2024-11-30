#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import yaml
from std_msgs.msg import String
import time

class TaskManager:
    def __init__(self):
        rospy.init_node('task_manager', anonymous=True)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.orders = []  # List to hold orders as a queue of tasks
        rospy.loginfo("Task Manager Initialized")

    def load_locations(self):
        # Load locations from a YAML file or predefined list
        locations_file = rospy.get_param('~locations_file', '/path/to/locations.yaml')
        try:
            with open(locations_file, 'r') as f:
                locations = yaml.safe_load(f)
                return locations
        except Exception as e:
            rospy.logerr("Failed to load locations file: %s", e)
            return []

    def add_order(self, order_id, location):
        # Adds a new order to the queue
        self.orders.append({'order_id': order_id, 'location': location, 'status': 'pending'})

    def send_goal(self, location):
        goal = PoseStamped()
        goal.header.frame_id = 'map'  # Using 'map' frame for global coordinates
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = location['x']
        goal.pose.position.y = location['y']
        goal.pose.orientation.w = 1.0  # Assuming the robot is facing forward (you can adjust orientation)
        self.goal_pub.publish(goal)
        rospy.loginfo("Goal sent: %s", goal)

    def process_orders(self):
        # Simulates processing of multiple orders
        for order in self.orders:
            if order['status'] == 'pending':
                self.send_goal(order['location'])
                order['status'] = 'in-progress'
                rospy.loginfo(f"Processing order {order['order_id']}: {order['status']}")
                rospy.sleep(5)  # Simulate time for delivery (adjust as needed)
                order['status'] = 'completed'
                rospy.loginfo(f"Completed order {order['order_id']}: {order['status']}")

    def cancel_order(self, order_id):
        # Cancel a specific order
        for order in self.orders:
            if order['order_id'] == order_id:
                order['status'] = 'canceled'
                rospy.loginfo(f"Canceled order {order_id}")

if __name__ == '__main__':
    try:
        task_manager = TaskManager()
        task_manager.add_order(1, {'x': 2.0, 'y': 3.0})  # Add order 1
        task_manager.add_order(2, {'x': 4.0, 'y': 5.0})  # Add order 2
        task_manager.process_orders()
    except rospy.ROSInterruptException:
        pass

