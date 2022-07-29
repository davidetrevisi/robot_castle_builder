#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


class_to_attach = ""
if len(sys.argv) != 1:
    class_to_attach = sys.argv[1]
else:
    class_to_attach = 'cube'


if __name__ == '__main__':
    rospy.init_node('demo_attach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    rospy.loginfo("Attaching object and gripper")
    req = AttachRequest()
    req.model_name_1 = class_to_attach
    req.link_name_1 = "link"
    req.model_name_2 = "robot"
    req.link_name_2 = "wrist_3_link"

    attach_srv.call(req)
