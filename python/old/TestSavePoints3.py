#!/usr/bin/env python3
import sys, json, rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import numpy as np

SAVE_FILE = "punkte.jsonl"


# Quaternion normalize
def normalize_quat(q):
    q = np.array(q, dtype=float)
    n = np.linalg.norm(q)
    if n > 0:
        q /= n
    return q.tolist()


# Save to JSONL
def save_point(name: str, xyz, quat):
    rec = {"name": name,
           "pos": xyz,
           "quat": normalize_quat(quat)}
    with open(SAVE_FILE, "a") as f:
        f.write(json.dumps(rec) + "\n")
    print(f"✅ Saved: {name}")


# Main teach logic
def teach_buffer(color: str, arm: MoveGroupCommander):
    pose: Pose = arm.get_current_pose().pose
    cyl = [pose.position.x, pose.position.y, pose.position.z]
    quat = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ]
    quat = normalize_quat(quat)

    # dynamic k selection
    g = 1 if color.lower() == "blue" else -1

    k = -0.02

    # compute offsets now (using k)
    OFFSET_PLATE = [
        -0.165,
        g * k * np.sin(np.deg2rad(15))-g*0.01 * np.sin(np.deg2rad(15)),
        k * np.cos(np.deg2rad(15))
    ]

    OFFSET_HEX = [-0.055, g * 0.009* np.sin(np.deg2rad(15)), 0.0]

    plate = [cyl[i] + OFFSET_PLATE[i] for i in range(3)]
    hexp  = [cyl[i] + OFFSET_HEX[i] for i in range(3)]

    save_point(f"Buffer_Cyl_{color}", cyl, quat)
    save_point(f"Buffer_Plate_{color}", plate, quat)
    save_point(f"Buffer_Hex_{color}", hexp, quat)


# ROS utils
def wait_for_joint_state():
    try:
        rospy.wait_for_message("/joint_states", JointState, timeout=5.0)
        return True
    except:
        print("❌ No /joint_states received.")
        return False


# MAIN LOOP
def main():
    rospy.init_node("teach_points", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    if not wait_for_joint_state():
        sys.exit(1)

    arm = MoveGroupCommander("panda_arm")

    print("\nCommands:")
    print("  teach <Color>   = save Cyl/Plate/Hex for given color")
    print("  q               = quit\n")

    while not rospy.is_shutdown():
        cmd = input("> ").strip()
        if not cmd:
            continue

        if cmd == "q":
            break

        if cmd.startswith("teach "):
            color = cmd.split(" ", 1)[1].strip()
            teach_buffer(color, arm)
        else:
            print("Unknown command")

    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
