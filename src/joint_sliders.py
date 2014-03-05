#!/usr/bin/env python
# RSL dev_scrap
#
#

import argparse
from pprint import pprint
from copy import deepcopy

import rospy

from sensor_msgs.msg import Joy

import baxter_interface
import baxter_dataflow

class JointMapper(object):
    """Simple class to map an input scale to joint limits"""
        # Max Joint Range (s0, s1, e0, e1, w0, w1, w2)
        #     ( 1.701,  1.047,  3.054,  2.618,  3.059,  2.094,  3.059)
        # Min Joint Range (e0, e1, s0, s1, w0, w1, w2)
        #     (-1.701, -2.147, -3.054, -0.050, -3.059, -1.571, -3.059)
#         joint_moves = (
#              [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0],
#              [0.5, -0.8, 2.8, 0.15, 0.0, 1.9, 2.8],
#             [-0.1, -0.8, -1.0, 2.5, 0.0, -1.4, -2.8],
#              [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0],
#             )
# [
#   # mode 5 -- nanoKONTROL2
#   # s, m, r
#   32, 48, 64,
#   33, 49, 65,
#   34, 50, 66,
#   35, 51, 67,
#   36, 52, 68,
#   37, 53, 69,
#   38, 54, 70,
#   39, 55, 71,
#   # rew, play, ff, repeat(cycle), stop, rec
#   43, 41, 44, 46, 42, 45,
#   # track-L, track-R, marker-set, marker-L, marker-R
#   58, 59, 60, 61, 62
# ]
    def __init__(self):
        pass

class JointSlider(object):
    def __init__(self):
        pass

def main():
    print("Welcome to Sliders")
    rospy.init_node('dev_joint_sliders')
    rate = rospy.Rate(300)
    relax = 2 # relax factor for thresholding goal
    joint_tau = baxter_interface.settings.JOINT_ANGLE_TOLERANCE * relax

    joy_in = Joy()
    joy_in_axes = []
    joy_in_btns = []
    def _on_joy(data, joy_store):
        joy_store.axes = list(data.axes)
        joy_store.buttons = list(data.buttons)
        print data.axes

    robot = baxter_interface.RobotEnable()
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')

    joy_sub = rospy.Subscriber('/joy', Joy, _on_joy, joy_in)

    print("waiting for first /joy message....")

    def body_fun():
        print ">>",
        print(joy_in_axes)

    baxter_dataflow.wait_for(lambda: len(joy_in.axes) > 0,
                             rate=10, timeout=5.0,
                             body=body_fun)


    joint_maxs = [ 1.701,  1.047,  3.054,  2.618,  3.059,  2.094,  3.059]
    joint_mins = [-1.701, -2.147, -3.054, -0.050, -3.059, -1.571, -3.059]
    inc_major = [0.1] * 7
    inc_minor = [0.01] * 7
    half_range = [1.0] * 7
    mid_major = [0.0] * 7
    flip_left = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
    flip_right = [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0]
    for j in range(7):
        inc_major[j] = (joint_maxs[j] - joint_mins[j]) / 128.0
        inc_minor[j] = inc_major[j] / 128.0
        half_range[j] = (joint_maxs[j] - joint_mins[j]) / 2.0
        mid_major[j] = joint_mins[j] + half_range[j]


    def in_out(in_axes, j_max, j_min, flip, a_max=1.0, a_min=-1.0):
        pass

    def inc_in(idx, in_major, in_minor, flip):
        j_out = mid_major[idx] + flip * (half_range[idx] * in_major
                                         + inc_major[idx] * in_minor)
#         base = joint_maxs[idx] if flip < 0 else joint_mins[idx]
#         j_out = base + flip * (inc_major[idx] * in_major
#                                + inc_minor[idx] * in_minor)
        return j_out

    idx_axes_major = [0, 1, 2, 3, 4, 5, 6]
    idx_axes_minor = [8, 9, 10, 11, 12, 13, 14]

    left_cmd = [0.0] * 7
    right_cmd = [0.0] * 7
    left.set_joint_position_speed(0.3)
    right.set_joint_position_speed(0.3)
    left.move_to_neutral()
    right.move_to_neutral()
    modes = ['mirror', 'left', 'right']
    mode = 0
    print("Sliding...")
    while not rospy.is_shutdown():
        joy_in_buttons = joy_in.buttons
        joy_in_axes = joy_in.axes

        for j in range(7):
            left_cmd[j] = inc_in(j,
                                 joy_in_axes[idx_axes_major[j]],
                                 joy_in_axes[idx_axes_minor[j]],
                                 flip_left[j]
                                 )
            right_cmd[j] = inc_in(j,
                                 joy_in_axes[idx_axes_major[j]],
                                 joy_in_axes[idx_axes_minor[j]],
                                 flip_right[j]
                                 )
        print(left_cmd)
        print(right_cmd)
        left.set_joint_positions(dict(zip(left.joint_names(), left_cmd)))
        right.set_joint_positions(dict(zip(right.joint_names(), right_cmd)))
#         right.move_to_joint_positions(dict(zip(right.joint_names(), right_cmd)), timeout=0.1)
        rate.sleep()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    args=parser.parse_args()

    main()
