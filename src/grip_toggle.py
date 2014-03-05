#!/usr/bin/env python
# RSL dev_scrap
# Tell Baxter to make way -- moves arms out of the way
# so he stops blocking the aisle.

import argparse
from pprint import pprint

import rospy

import baxter_interface
import dataflow

def main(goal_pose='side_car'):
    print("Making way...")
    rospy.init_node('rob_make_way')
    robot = baxter_interface.RobotEnable()
    gripper_left = baxter_interface.Gripper("left")
    gripper_right = baxter_interface.Gripper("right")
    io_left_lower = baxter_interface.DigitalIO('left_lower_button')
    io_left_upper = baxter_interface.DigitalIO('left_upper_button')
    io_right_lower = baxter_interface.DigitalIO('right_lower_button')
    io_right_upper = baxter_interface.DigitalIO('right_upper_button')
    rate = rospy.Rate(20)

    print("enabling")
    robot.enable()
    print("moving...")
    while not win() and not rospy.is_shutdown():
        # Watch for gripper button presses
        if io_left_lower.state:
            self._gripper_left.open()
        elif io_left_upper.state:
            self._gripper_left.close()
        if io_right_lower.state:
            self._gripper_right.open()
        elif io_right_upper.state:
            self._gripper_right.close()

#         print("go")
        left.set_joint_positions(dict(zip(left.joint_names(),pose['left'])))
        right.set_joint_positions(dict(zip(right.joint_names(),pose['right'])))
#         self._arms[limb].set_joint_positions(dict(zip(self._arms[limb].joint_names(),self._joint_moves[tuck][limb])))
        rate.sleep()
    print('You may proceed.')
    print_diff()

    while not self.done():

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', choices=['side_car','neutral','untuck'],
                        dest='pose', default='side_car', nargs='?')
    args=parser.parse_args()

    main(args.pose)
