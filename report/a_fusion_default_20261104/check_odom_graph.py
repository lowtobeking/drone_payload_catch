"""Read only: inspect A's external-odometry ROS publisher/subscriber graph."""
import json
import time
import rclpy
from rclpy.node import Node

rclpy.init()
node=Node("a_odom_graph_readonly")
topic="/fmu/in/vehicle_visual_odometry"
try:
    end=time.monotonic()+5
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.05)
    result={"publishers":[{"name":x.node_name,"namespace":x.node_namespace}
                          for x in node.get_publishers_info_by_topic(topic)],
            "subscribers":[{"name":x.node_name,"namespace":x.node_namespace}
                           for x in node.get_subscriptions_info_by_topic(topic)]}
    print(json.dumps(result))
finally:
    node.destroy_node()
    rclpy.shutdown()
