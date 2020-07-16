#!/usr/bin/env python

import rospy
from tim781_safety_io import TimSafetyIo


def main():

    rospy.init_node("tim_safety_node")

    rc_node = TimSafetyIo()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
