#!/usr/bin/env python3
import rospy
from franka_msgs.msg import FrankaState
import numpy as np
import sys
from tf.transformations import quaternion_from_matrix

SAVE_FILE = "punkte.txt"
latest_T = None

def franka_state_cb(msg: FrankaState):
    global latest_T
    latest_T = np.array(msg.O_T_EE).reshape(4, 4)

def save_pose(T):
    # ✅ KORREKTE Extraktion der Translation
    pos = T[3, 0:3]  # <-- statt T[0:3,3]

    quat = quaternion_from_matrix(T)

    with open(SAVE_FILE, "a") as f:
        f.write(",".join(map(str, [
            pos[0], pos[1], pos[2],
            quat[0], quat[1], quat[2], quat[3]
        ])) + "\n")

def main():
    global latest_T
    rospy.init_node("franka_teach_points")
    rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, franka_state_cb)

    print("Roboter per Hand bewegen – ENTER speichert Position | q beendet")

    count = 0
    while not rospy.is_shutdown():
        inp = input("> ")
        if inp.lower() == "q":
            print(f"Gespeichert: {count} Punkte")
            sys.exit(0)

        if latest_T is None:
            print("Warte auf Robot-States…")
            continue

        save_pose(latest_T)
        count += 1
        print(f"✅ Punkt {count} gespeichert")

if __name__ == "__main__":
    main()
