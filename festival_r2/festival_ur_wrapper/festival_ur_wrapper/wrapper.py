# -*- coding: utf-8 -*-

import rtde_control, rtde_receive
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from festival_ur_interfaces.srv import MoveJoint


class URWrapper(Node):
    def __init__(self):
        super().__init__("festival_ur_wrapper")
        self.init_parameters()

        self.rtde_c = rtde_control.RTDEControlInterface(self.ip_address)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.ip_address)
        self.get_logger().info("URWrapper initialized")

        # Initialize robot
        self.rtde_c.moveJ(self.init_joint)
        self.get_logger().info("Robot initialized")
        print(self.init_joint)
        # ROS2 publisher
        self.current_joint_pub = self.create_publisher(
            JointState, "current_joint_states", 10
        )

        # ROS2 subscriber
        self.target_joint = self.create_subscription(
            JointState, "target_joint", self.target_joint_callback, 10
        )

        # ROS2 service
        self.target_joint_srv = self.create_service(
            MoveJoint, "target_joint_srv", self.target_joint_srv_callback
        )

        # ROS2 timer
        self.timer_ = self.create_timer(0.1, self.timer_publisher_callback)

    def init_parameters(self):
        self.get_logger().info("Initializing parameters")

        self.declare_parameter("ip_address", "192.168.0.3")
        self.declare_parameter("init_joint", [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0])
        self.declare_parameter("dt", 0.008)

        self.ip_address = (
            self.get_parameter("ip_address").get_parameter_value().string_value
        )
        self.init_joint = (
            self.get_parameter("init_joint").get_parameter_value().double_array_value
        )

        self.dt = self.get_parameter("dt").get_parameter_value().double_value
        self.get_logger().info("Parameters initialized")

    def timer_publisher_callback(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        joint_state.position = self.rtde_r.getActualQ()
        joint_state.velocity = self.rtde_r.getActualQd()
        self.current_joint_pub.publish(joint_state)

    def target_joint_callback(self, msg):
        if msg.header.frame_id == "state":
            t_start = self.rtde_c.initPeriod()
            self.rtde_c.servoJ(msg.position, time=self.dt, lookahead_time=0.1, gain=300)
            self.rtde_c.waitPeriod(t_start)
        elif msg.header.frame_id == "speed":
            t_start = self.rtde_c.initPeriod()
            self.rtde_c.speedJ(msg.velocity, acceleration=0.5, time=self.dt)
            self.rtde_c.waitPeriod(t_start)
        # else:
        #     self.rtde_c.servoStop()
        #     self.rtde_c.speedStop()

    def target_joint_srv_callback(self, request, response):
        self.rtde_c.moveJ(request.position)
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)

    node = URWrapper()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
