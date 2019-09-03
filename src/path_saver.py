#!/usr/bin/env python

'''
TODO Documentation
'''

import tf
import json
import rospy


DEFAULT_BASE_FRAME = '/base_link'
DEFAULT_EE_FRAME = '/ee_link'


class PathSaver:

    def __init__(self, filepath, base_frame, ee_frame):
        self._base_frame = base_frame
        self._ee_frame = ee_frame
        self._filepath = filepath

        self._listener = tf.TransformListener()

    def spin(self):
        data = {
            'base_frame': self._base_frame,
            'ee_frame': self._ee_frame,
            'path': []
        }

        file = open(self._filepath,'w')

        try:
            while not rospy.is_shutdown():
                inStr = raw_input('Press enter to capture pose (or press q to quit)')
                if inStr.lower() == 'q':
                    break
                (pos, rot) = self._listener.lookupTransform(self._base_frame, self._ee_frame, rospy.Time(0))
                data['path'].append({'position': pos, 'orientation': rot})
        except:
            pass

        json.dump(data, file, indent=4)
        file.close()


if __name__ == "__main__":
    rospy.init_node('path_saver')

    filepath = rospy.get_param('~filepath')
    base_frame = rospy.get_param('~base_frame',DEFAULT_BASE_FRAME)
    ee_frame = rospy.get_param('~ee_frame',DEFAULT_EE_FRAME)

    node = PathSaver(filepath, base_frame, ee_frame)
    node.spin()
