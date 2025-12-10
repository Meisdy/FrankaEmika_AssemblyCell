#!/usr/bin/env python3
import rospy
import sys
import json
import moveit_commander

SAVE_FILE = "punkte.jsonl"  # JSON pro Zeile mit Namen

def save_pose(name, pose):
    point = {
        "name": name,
        "pos": [pose.position.x, pose.position.y, pose.position.z],
        "quat": [pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w]
    }

    with open(SAVE_FILE, "a") as f:
        f.write(json.dumps(point) + "\n")


def main():
    rospy.init_node("franka_teach_points_v2")
    moveit_commander.roscpp_initialize([])

    arm = moveit_commander.MoveGroupCommander("panda_arm")

    print("Roboter per Hand bewegen – ENTER speichert Position | q beendet")

    while not rospy.is_shutdown():
        name = input("Name: ").strip()
        if name.lower() == "q":
            print("Beendet.")
            sys.exit(0)

        if name == "":
            print("⚠ Kein Name → übersprungen.")
            continue

        # ✅ Pose aus MoveIt holen → KORREKTER Frame
        pose = arm.get_current_pose().pose

        save_pose(name, pose)
        print(f"✅ Gespeichert: {name}")


if __name__ == "__main__":
    main()


# #!/usr/bin/env python3
# import rospy
# from franka_msgs.msg import FrankaState
# import numpy as np
# import sys
# import json
# from tf.transformations import quaternion_from_matrix

# ############################
# # This funktion is saving points in a json file in dict format
# # Output format: {"name": "home", "pos": [0.32, 0.22, 0.45], "quat": [0,0,0,1]}
# ############################



# SAVE_FILE = "punkte.jsonl"  # JSON pro Zeile
# latest_T = None

# def franka_state_cb(msg: FrankaState):
#     global latest_T
#     latest_T = np.array(msg.O_T_EE).reshape(4, 4)

# def save_pose(T, name):
#     # Translation sitzt in der 4. Zeile (Index 3), Spalten 0..2
#     pos = T[3, 0:3]
#     quat = quaternion_from_matrix(T)

#     point = {
#         "name": name,
#         "pos": pos.tolist(),
#         "quat": quat.tolist()
#     }

#     with open(SAVE_FILE, "a") as f:
#         f.write(json.dumps(point) + "\n")


# def main():
#     global latest_T
#     rospy.init_node("franka_teach_points")
#     rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, franka_state_cb)

#     print("ENTER speichert | Name eingeben | q beendet")

#     while not rospy.is_shutdown():
#         name = input("Name: ").strip()
#         if name.lower() == "q":
#             print("Beendet.")
#             sys.exit(0)

#         if latest_T is None:
#             print("Warte auf Robot-State…")
#             continue

#         if name == "":
#             print("Kein Name → übersprungen.")
#             continue

#         save_pose(latest_T, name)
#         print(f"✅ Gespeichert: {name}")

# if __name__ == "__main__":
#     main()
