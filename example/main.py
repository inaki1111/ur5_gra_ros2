# main.py
#!/usr/bin/env python3
from arm_functions import (
    init_arm, shutdown_arm,
    print_joint_states, print_tool_pose,
    move_joints, move_pose
)
from gripper_functions import RobotiqGripper
import time

def main():
    # initialze the robot and gripper
    node = init_arm()
    gripper = RobotiqGripper()
    gripper.connect()
    gripper.activate()

    # intiial state
    print_joint_states(node)
    print_tool_pose(node, 'tool0')



    # poses to send to the robot
    poses = {
        'pose0': [-0.2, 0.7, 0.5, 1.0, 0.0, 0.0, 0.0],
        'pose1': [-0.2, 0.7, 0.2, 1.0, 0.0, 0.0, 0.0],
        'pose2': [-0.2, 0.7, 0.5, 1.0, 0.0, 0.0, 0.0],
        'pose3': [ 0.2, 0.7, 0.5, 1.0, 0.0, 0.0, 0.0],
        'pose4': [ 0.2, 0.7, 0.2, 1.0, 0.0, 0.0, 0.0],
        'pose5': [ 0.0, 0.3, 0.5, 1.0, 0.0, 0.0, 0.0],
    }

    for name, p in poses.items():
        move_pose(node, p)
        print(f'{name} done')
        time.sleep(1)
        if name == 'pose1':
            gripper.move_and_wait_for_pos(255, 255, 255)  # close
        if name == 'pose4':
            gripper.move_and_wait_for_pos(0, 255, 255)    # open

    # print state
    print_joint_states(node)
    print_tool_pose(node, 'tool0')

    # disconnect the gripper
    gripper.disconnect()
    shutdown_arm()

if __name__ == '__main__':
    main()
