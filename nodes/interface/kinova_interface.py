#! /usr/bin/env python3

import time
from typing import Union, List, Tuple

import rospy

# import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

from std_msgs.msg import Header, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class KinovaInterface:
    def __init__(
        self,
        arm_type: str = "j2n6s300",
        dim_joint: int = 6,
        dim_finger: int = 3,
        dim_finger_tip: int = 3,
        wait: bool = True,
    ):
        super().__init__()
        self.n_dim = dim_joint + dim_finger + dim_finger_tip
        self.arm_type = arm_type
        self.cached_msg = None

        rospy.init_node("gazebo_kinova_interface", anonymous=False)

        # add joint force publishers
        self.effort_joints_pub = [
            rospy.Publisher(
                f"/{self.arm_type}/joint_{i + 1}_effort_controller/command",
                Float64,
                queue_size=10,
            )
            for i in range(dim_joint)
        ]
        # add finger force publishers
        self.effort_joints_pub.extend(
            [
                rospy.Publisher(
                    f"/{self.arm_type}/finger_{i + 1}_effort_controller/command",
                    Float64,
                    queue_size=10,
                )
                for i in range(dim_finger)
            ]
        )
        # add finger tip force publishers
        self.effort_joints_pub.extend(
            [
                rospy.Publisher(
                    f"/{self.arm_type}/finger_tip_{i + 1}_effort_controller/command",
                    Float64,
                    queue_size=10,
                )
                for i in range(dim_finger_tip)
            ]
        )

        self.subscriber = rospy.Subscriber(
            f"/{self.arm_type}/joint_states", JointState, self.new_state_cb
        )

        if wait:
            time.sleep(1)

    def new_state_cb(self, msg):
        self.cached_msg = msg

    def send_joint_effort(self, tau: List[Union[float, int, None]]) -> None:
        """
        Given a list of numbers (or None), send them as the force to be applied on
        each joint. If None is given, that joint will be skipped.
        """
        assert len(tau) == self.n_dim, tau
        for force, pub in zip(tau, self.effort_joints_pub):
            if force is not None:
                pub.publish(Float64(force))

    def get_current_state(self) -> Tuple[List[float], List[float]]:
        """
        Return the name, current q and qd of the arm.
        IMPORTANT: Note that ros/kinova does not automatically wraps the values,
            so the joint rotational angles will not be within [-pi, pi).
        """
        return self.cached_msg.name, self.cached_msg.position, self.cached_msg.velocity
