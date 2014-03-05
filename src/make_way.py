#!/usr/bin/env python
# RSL dev_scrap
# Tell Baxter to make way -- moves arms out of the way
# so he stops blocking the aisle.

import argparse
from pprint import pprint

import rospy

import baxter_interface

def main(goal_pose='side_car'):
    print("Making way...")
    rospy.init_node('rob_make_way')
    robot = baxter_interface.RobotEnable()
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    rate = rospy.Rate(20)
    relax = 2 # relax factor for thresholding goal
    joint_tau = baxter_interface.settings.JOINT_ANGLE_TOLERANCE * relax

    pose_target = {
        'side_car': {
                        'left': [0.5,  -1.35, 0.44, 1.84, -0.12, 1.19, 0.0],
                        'right': [-0.5,  -1.35, -0.44, 1.84, 0.12, 1.19, 0.0],
                    },
         'neutral': {
                         'left': [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0],
                         'right': [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]
                    },
          'untuck': {
                        'left': [-0.08,  -1.0, -1.19, 1.94, 0.67, 1.03, -0.50],
                        'right': [0.08,  -1.0, 1.19, 1.94, -0.67, 1.03, 0.50]
                    },
           'zeros': {
                        'left': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        'right': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    }
                  }
    pose = pose_target[goal_pose]
#     pose = pose_target['side_car']

    def win():
        made_it = True
        for side,limb in [('left',left), ('right',right)]:
            for idx,joint in enumerate(limb.joint_names()):
                diff = abs(pose[side][idx] - limb.joint_angle(joint))
                if diff >= joint_tau:
                    made_it = False
        return made_it
    
    def print_diff():
        for side,limb in [('left',left), ('right',right)]:
            print('---Final %s----' % (side,))
            for idx,joint in enumerate(limb.joint_names()):
                print("%s: % .2f  ~=  % .5f" % (joint,pose[side][idx],limb.joint_angle(joint)))
# 
#         for arm in [left, right]
#             for joint in arm.joint_names():
#                 print("%s: %f" % (joint,arm.joint_angle(joint),))
#         pprint(pose['left'])
#         pjoints = [left.joint_angle(joint) for joint in left.joint_names()]
#         pprint(pjoints)
#     #     pprint(left.joint_angles())
#         print('---Final Right---')
#         pprint(pose['right'])
#         pjoints = [right.joint_angle(joint) for joint in right.joint_names()]
#         pprint(pjoints)
#     #     pprint(right.joint_angles())



    print("enabling")
    robot.enable()
    print("moving...")
    while not win() and not rospy.is_shutdown():
#         print("go")
        left.set_joint_positions(dict(zip(left.joint_names(),pose['left'])))
        right.set_joint_positions(dict(zip(right.joint_names(),pose['right'])))
#         self._arms[limb].set_joint_positions(dict(zip(self._arms[limb].joint_names(),self._joint_moves[tuck][limb])))
        rate.sleep()
    print('You may proceed.')
    print_diff()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', choices=['side_car','neutral','untuck', 'zeros'],
                        dest='pose', default='side_car', nargs='?')
    args=parser.parse_args()

    main(args.pose)
