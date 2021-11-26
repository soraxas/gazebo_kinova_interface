#! /usr/bin/env python3

import time
from typing import Union, List, Tuple

import rospy
# import moveit_commander
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Duration


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

        self.effort_joints_pub = self.__create_publishers(
            self.arm_type,
            f"delta_effort_controller/delta_effort_command",
            dim_joint,
            dim_finger,
            dim_finger_tip,
            data_type=Float64,
        )
        self.effort_duration_joints_pub = self.__create_publishers(
            self.arm_type,
            f"delta_effort_controller/delta_effort_duration",
            dim_joint,
            dim_finger,
            dim_finger_tip,
            data_type=Duration,
        )

        self.subscriber = rospy.Subscriber(
            f"/{self.arm_type}/joint_states", JointState, self.new_state_cb
        )

        if wait:
            time.sleep(1)

    @classmethod
    def __create_publishers(
            cls, arm_type, cmd_suffix, dim_joint, dim_finger, dim_finger_tip, data_type
    ):
        container = []
        # add joint force publishers
        container.extend(
            [
                rospy.Publisher(
                    f"/{arm_type}/joint_{i + 1}_{cmd_suffix}",
                    data_type,
                    queue_size=10,
                )
                for i in range(dim_joint)
            ]
        )
        # add finger force publishers
        container.extend(
            [
                rospy.Publisher(
                    f"/{arm_type}/finger_{i + 1}_{cmd_suffix}",
                    data_type,
                    queue_size=10,
                )
                for i in range(dim_finger)
            ]
        )
        # add finger tip force publishers
        container.extend(
            [
                rospy.Publisher(
                    f"/{arm_type}/finger_tip_{i + 1}_{cmd_suffix}",
                    data_type,
                    queue_size=10,
                )
                for i in range(dim_finger_tip)
            ]
        )
        return container

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

    def send_effort_duration(self, duration: List[Union[float, int, None]]) -> None:
        """
        Given a list of numbers (or None), send them as the force to be applied on
        each joint. If None is given, that joint will be skipped.
        """
        assert len(duration) == self.n_dim, duration
        for duration, pub in zip(duration, self.effort_duration_joints_pub):
            if duration is not None:
                d = Duration()
                d.data.nsecs = duration
                pub.publish(d)

    def get_current_state(self) -> Tuple[List[float], List[float]]:
        """
        Return the name, current q and qd of the arm.
        IMPORTANT: Note that ros/kinova does not automatically wraps the values,
            so the joint rotational angles will not be within [-pi, pi).
        """
        return self.cached_msg.name, self.cached_msg.position, self.cached_msg.velocity
