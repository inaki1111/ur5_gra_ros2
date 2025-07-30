#!/usr/bin/env python3
import rclpy
import pprint
from ur5e_moveit_client import UR5eMoveItClient

_pp = pprint.PrettyPrinter(indent=4)

def init_arm():
    print('Starting ROS 2')
    rclpy.init()
    return UR5eMoveItClient()

def shutdown_arm():
    rclpy.shutdown()

def print_joint_states(node):
    jp = node.get_joint_positions()
    print('Joint positions:')
    _pp.pprint(list(jp))

def print_tool_pose(node, link_name='tool0'):
    """position of the end effector"""
    pos, ori = node.get_link_position(link_name)
    if pos and ori:
        print(f'Position of {link_name}: {pos}')
        print(f'Orientation of {link_name}: {ori}')

def move_joints(node, joint_values):
    node.move_to_joints(joint_values)

def move_pose(node, pose):
    node.move_to_pose(pose)
