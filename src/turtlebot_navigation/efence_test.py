#!/usr/bin/env python

import sys
import rospy
import tf

import numpy as np
from robot_msgs.srv import *
from geometry_msgs.msg import Point

if __name__ == "__main__":
    rospy.wait_for_service('/eFence/opt');
    print "connected the service"
    try:
        fun = rospy.ServiceProxy('eFence/opt', E_Fence);
        print "fun back ok"
        req = E_FenceRequest();
        req.efence.header.seq = 3;
        req.efence.header.frame_id = "ADD";

        req.efence.polygon.points.append(Point(30.5, 21.8, 0.0))
        req.efence.polygon.points.append(Point(30.5, 7.8, 0.0))

        print req
        res = fun(req);
        print res
    except rospy.ROSInterruptException:
        pass


