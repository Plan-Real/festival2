import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class JointControlNode(Node):
    def __init__(self):
        super().__init__("joint_control_node")
        self.publisher_ = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        joint_state.position = [0.0, -1.57, 3.14, -1.57, 0.0, 0.0]
        joint_state.velocity = []
        joint_state.effort = []
        self.publisher_.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    joint_control_node = JointControlNode()
    rclpy.spin(joint_control_node)
    joint_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
