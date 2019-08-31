#!/usr/bin/env python

'''
TODO Documentation

let config = {
    SIM_RESOLUTION: 128,
    DYE_RESOLUTION: 1024,
    CAPTURE_RESOLUTION: 512,
    DENSITY_DISSIPATION: 1,
    VELOCITY_DISSIPATION: 0.2,
    PRESSURE: 0.8,
    PRESSURE_ITERATIONS: 20,
    CURL: 30,
    SPLAT_RADIUS: 0.25,
    SPLAT_FORCE: 6000,
    SHADING: true,
    COLORFUL: true,
    COLOR: {r: 255, g: 255, b:255},
    COLOR_UPDATE_SPEED: 10,
    PAUSED: false,
    BACK_COLOR: { r: 0, g: 0, b: 0 },
    TRANSPARENT: false,
    BLOOM: true,
    BLOOM_ITERATIONS: 8,
    BLOOM_RESOLUTION: 256,
    BLOOM_INTENSITY: 0.8,
    BLOOM_THRESHOLD: 0.6,
    BLOOM_SOFT_KNEE: 0.7,
    SUNRAYS: true,
    SUNRAYS_RESOLUTION: 196,
    SUNRAYS_WEIGHT: 1.0,
}
'''

import tf
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
        rospy.wait_for_service('/visualization/get_config')

        self._listener = tf.TransformListener()

        self._start_pub = rospy.Publisher('/visualization/trajectory_start',VisualizationTrajectory,queue_size=1)
        self._update_pub = rospy.Publisher('/visualization/trajectory_update',VisualizationTrajectory,queue_size=5)
        self._stop_pub = rospy.Publisher('/visualization/trajectory_stop',Empty,queue_size=1)

        self._set_config = rospy.ServiceProxy('/visualization/set_config',SetConfig)
        self._get_config = rospy.ServiceProxy('/visualization/get_config',GetConfig)

        self._timer = rospy.Timer(rospy.Duration(update_timestep), self._timer_cb)

    def spin(self):

        rospy.sleep(10)

        vt, st = self._generate_update()

        if vt != None and st != None:
            self._set_config(st)
            self._start_pub.publish(vt)
            self._started = True
            print 'Started'

        while not rospy.is_shutdown():
            rospy.sleep(0.25)

        self._stop_pub.publish(Empty())

    def _generate_update(self):

        try:
            (pos,rot) = self._listener.lookupTransform(self._tf_ee, self._tf_base, rospy.Time(0))

            #TODO mapping here

            config = {'COLOR': {'r': 0.125, 'g': 0.055, 'b':0.090}}

            xOffset = 1
            yOffset = 1

            tempX = self.prevXOffset
            tempY = self.prevYOffset
            self.prevXOffset = xOffset
            self.prevYOffset = yOffset

            if xOffset == self.prevXOffset or yOffset == self.prevYOffset:
                return None, None

            return VisualizationTrajectory(xOffset,yOffset), json.dumps(config)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None, None

    def _timer_cb(self, event):
        if self._started:
            vt, st = self._generate_update()
            if vt != None and st != None:
                self._set_config(st)
                self._update_pub.publish(vt)


if __name__ == "__main__":
    rospy.init_node('trajectory_visualizer')

    node = TrajectoryVisualizer()
    node.spin()
