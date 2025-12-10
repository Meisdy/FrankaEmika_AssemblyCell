#!/usr/bin/env python3
import rospy
from franka_msgs.msg import FrankaState
from std_msgs.msg import String

last_mode = None

def cb(msg):
    global last_mode

    mode = msg.robot_mode  # z. B. 4 = user_stop
    pub = rospy.Publisher("/franka_button_event", String, queue_size=1)

    # Debug-Ausgabe
    rospy.loginfo(f"Franka Mode: {mode}")

    # If we saw a STOP before and now return to active state
    if last_mode == 5 and mode != 5:
        rospy.logwarn("BUTTON RELEASE DETECTED → sending RESTART event")
        pub.publish("RESTART")

    last_mode = mode

rospy.init_node("franka_button_watch")
rospy.loginfo("Button Watch Node gestartet – warte auf Franka State...")
rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, cb)
rospy.spin()

