#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import line_detector  as classify

__author__ =  'Fred'
__version__=  '0.1'
__license__ = 'BSD'

if __name__ == '__main__':
    rospy.init_node('lineDetector_node', anonymous=True)
    obs = classify.lineDetector()
    rospy.spin()
