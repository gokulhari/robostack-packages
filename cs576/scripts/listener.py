#!/usr/bin/env python
import rospy
from cs576.msg import Chain2D


def callback(msg):
    msg_str = " ".join(map(str, msg.config))
    rospy.loginfo(rospy.get_caller_id() + " config = [%s]", msg_str)
    rospy.loginfo(rospy.get_caller_id() + " W = %s", str(msg.W))
    rospy.loginfo(rospy.get_caller_id() + " D = %s", str(msg.D))
    rospy.loginfo(rospy.get_caller_id() + " L = %s", str(msg.L))


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("chatter", Chain2D, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    listener()
