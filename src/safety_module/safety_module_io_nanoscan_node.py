#!/usr/bin/env python

import rospy
from safety_module_io_nanoscan import SafetyModuleIO


def main():

    rospy.init_node("safety_module_io")

    rc_node = SafetyModuleIO()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
