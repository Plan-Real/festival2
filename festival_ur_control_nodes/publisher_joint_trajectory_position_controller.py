# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from festival_ur_core.msg import Purpose

class PublisherJointTrajectory(Node):
    def __init__(self):
        super().__init__("publisher_joint_trajectory_position_controller")


        self.joint_purpose_sub = self.create_subscription(
            Purpose, "joint_purpose", self.joint_purpose_callback, 1
        )
        # self.joint_state_sub = self.create_subscription(
        #     JointState, "joint_states", self.joint_state_callback, 10
        # )
        
        self.publisher_ = self.create_publisher(JointTrajectory, "scaled_joint_trajectory_controller", 1)



    def joint_state_callback(self, msg):

        if not self.joint_state_msg_received:

            # check start state
            limit_exceeded = [False] * len(msg.name)
            for idx, enum in enumerate(msg.name):
                if (msg.position[idx] < self.starting_point[enum][0]) or (
                    msg.position[idx] > self.starting_point[enum][1]
                ):
                    self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
                    limit_exceeded[idx] = True

            if any(limit_exceeded):
                self.starting_point_ok = False
            else:
                self.starting_point_ok = True

            self.joint_state_msg_received = True
        else:
            return



    def joint_purpose_callback(self, msg):
        float_goal = []
        joint_names =[]
        goal_purpose=msg.joints
        for value in goal_purpose:
            float_goal.append(float(value))
        self.get_logger().info("joint_1 : {}, joint_2 : {}, joint_3 : {}, joint_4 : {}, joint_5 : {}, joint_6 : {}"
                               .format(goal_purpose[0],goal_purpose[1], goal_purpose[2], goal_purpose[3], goal_purpose[4], goal_purpose[5] ) )
        
        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        traj = JointTrajectory()
        traj.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = float_goal
        point.time_from_start = Duration(msg.time)

        traj.points.append(point)
        self.publisher_.publish(traj)

  
def main(args=None):
    rclpy.init(args=args)

    publisher_joint_trajectory = PublisherJointTrajectory()

    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
