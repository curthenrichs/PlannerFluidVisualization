#!/usr/bin/env python

'''
MIT License

Copyright (c) 2019 Curt Henrichs

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

'''
Trajectory Visualizer Node

Node listens to end-effector transform provided and transforms the TF into a
perturbation for the fluid visualization. View taken is a top-down perspective
mapping X- and Y-axis values to offsets from center of screen in normalized unit
space. Z-axis value is used to control radius of perturbation circle within
predefined range. Orientation is converted from quaternion into RGB values for
the fluid perturbation point.

On start of movement (when last and current poses don't match) the node will
publish a trajectory_start to the visualization front-end; likewise when poses
match a trajectory_stop is published. While moving the set_config service is
called to update color and radius. Also while moving trajectory_updates are
being published.

Publishers:
    - /visualization/trajectory_start := VisualizationTrajectory
        "Published when robot arm starts moving, updates perturbation offset"
    - /visualization/trajectory_update := VisualizationTrajectory
        "Published while robot arm is moving, updates perturbation offset"
    - /visualization/trajectory_stop := Empty
        "Published when robot arm is still"

Subscribers:
    - /tf := tfMessage
        "Standard transform tree"

Services Requested:
    - /visualization/set_config := SetConfig
        "Sets color and radius of perturbation in visualization"
'''

import tf
import math
import json
import rospy

from std_msgs.msg import Empty, String
from planner_fluid_visualization.msg import VisualizationTrajectory
from planner_fluid_visualization.srv import GetConfig, SetConfig


class TrajectoryVisualizer:

    def __init__(self, tf_base='/base_link', tf_ee='/ee_link', update_timestep=0.01):
        self._tf_ee = tf_ee
        self._tf_base = tf_base
        self._started = False
        self.prevXOffset = -1
        self.prevYOffset = -1

        rospy.wait_for_service('/visualization/set_config')

        self._listener = tf.TransformListener()

        self._start_pub = rospy.Publisher('/visualization/trajectory_start',VisualizationTrajectory,queue_size=1)
        self._update_pub = rospy.Publisher('/visualization/trajectory_update',VisualizationTrajectory,queue_size=5)
        self._stop_pub = rospy.Publisher('/visualization/trajectory_stop',Empty,queue_size=1)

        self._set_config = rospy.ServiceProxy('/visualization/set_config',SetConfig)

        self._timer = rospy.Timer(rospy.Duration(update_timestep), self._timer_cb, False)

    def spin(self):

        rospy.sleep(10)

        vt, st = self._generate_update(True)

        if vt != None and st != None:
            self._set_config(st)
            self._start_pub.publish(vt)
            self._started = True
            print 'Started'

        while not rospy.is_shutdown():
            rospy.sleep(0.25)

        self._stop_pub.publish(Empty())

    def _generate_update(self, bypass=False):

        try:
            (pos,rot) = self._listener.lookupTransform(self._tf_base, self._tf_ee, rospy.Time(0))
            (rx, ry, rz) = tf.transformations.euler_from_quaternion(rot)

            r = (self._normalize_angle(rx) + math.pi) * 0.15 / ( 2 * math.pi)
            g = (self._normalize_angle(ry) + math.pi) * 0.15 / ( 2 * math.pi)
            b = (self._normalize_angle(rz) + math.pi) * 0.15 / ( 2 * math.pi)

            config = {
                'COLOR': {'r': r, 'g': g, 'b': b},
                'SPLAT_RADIUS': (pos[2] + 1) / 2
            }

            xOffset = pos[0]
            yOffset = -pos[1]

            tempX = self.prevXOffset
            tempY = self.prevYOffset
            self.prevXOffset = xOffset
            self.prevYOffset = yOffset

            if not bypass:
                if xOffset == tempX or yOffset == tempY:
                    return None, None

            return VisualizationTrajectory(xOffset,yOffset), json.dumps(config)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None, None

    def _normalize_angle(self, angle):
        angle = angle * 180.0 / math.pi
        while angle <= -180:
            angle += 360;
        while angle > 180:
            angle -= 360;
        return angle * math.pi / 180.0;

    def _timer_cb(self, event):
        if self._started:
            vt, st = self._generate_update()
            if vt != None and st != None:
                print vt, st
                self._set_config(st)
                self._update_pub.publish(vt)


if __name__ == "__main__":
    rospy.init_node('trajectory_visualizer')

    node = TrajectoryVisualizer()
    node.spin()
