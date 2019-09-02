#!/usr/bin/env python

'''
TODO Documentation
'''

import sys
import json
import rospy
import moveit_commander

from geometry_msgs.msg import Pose, Vector3, Quaternion


DEFAULT_MOVE_GROUP = 'manipulator'


class PathReaderMoveit:

    def __init__(self, filepath, repeat, move_group):
        self._repeat = repeat

        file = open(filepath,'r')
        self._data = json.load(file)
        file.close()

        self._move_group = moveit_commander.MoveGroupCommander(move_group)

    def spin(self):

        while not rospy.is_shutdown():

            # run path
            count = 1
            for p in self._data['path']:

                print 'Pose #{}'.format(count)
                count += 1

                pose = self._format_pose(p)

                self._move_group.set_pose_target(pose)
                self._move_group.go(wait=True)
                self._move_group.stop()
                self._move_group.clear_pose_targets()

            # if done
            if not self._repeat:
                break

    def _format_pose(self, dct):
        return Pose(
            position=Vector3(
                x=dct['position'][0],
                y=dct['position'][1],
                z=dct['position'][2]),
            orientation=Quaternion(
                x=dct['orientation'][0],
                y=dct['orientation'][1],
                z=dct['orientation'][2],
                w=dct['orientation'][3]))


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('path_reader_moveit')

    filepath = rospy.get_param('~filepath')
    repeat = rospy.get_param('~repeat',False)
    move_group = rospy.get_param('~move_group',DEFAULT_MOVE_GROUP)

    node = PathReaderMoveit(filepath, repeat, move_group)
    node.spin()
