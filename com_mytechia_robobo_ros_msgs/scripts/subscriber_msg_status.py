#!/usr/bin/env python

import rospy

from com_mytechia_robobo_ros_msgs.msg import Status

rospy.init_node('topic_status_subscriber')

def listener(status):

    parameters={}
    for key_value in status.value:
        parameters[key_value.key.strip()]= key_value.value.strip();

    print 'Status name:{0} key_value:{1}'.format(status.name, parameters)


sub= rospy.Subscriber('status', Status, listener)

rospy.spin()


