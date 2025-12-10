#!/usr/bin/env python3
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np

SAVE_FILE = "punkte.txt"

def normalize_quaternion(pose):
    q = np.array([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ])
    norm = np.linalg.norm(q)
    if norm > 0:
        q = q / norm
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

def load_points():
    punkte = []
    with open(SAVE_FILE, "r") as f:
        for line in f:
            vals = list(map(float, line.split(",")))
            if len(vals) == 7:
                pose = geometry_msgs.msg.Pose()
                pose.position.x = vals[0]
                pose.position.y = vals[1]
                pose.position.z = vals[2]
                pose.orientation.x = vals[3]
                pose.orientation.y = vals[4]
                pose.orientation.z = vals[5]
                pose.orientation.w = vals[6]
                normalize_quaternion(pose)
                punkte.append(pose)
    return punkte

def main():
    rospy.init_node("punkt_abfahren")
    moveit_commander.roscpp_initialize([])

    arm = moveit_commander.MoveGroupCommander("panda_arm")
    gripper = moveit_commander.MoveGroupCommander("panda_hand")

    # ✅ Orientation wird ernst genommen
    arm.set_goal_orientation_tolerance(0.005)
    arm.set_goal_position_tolerance(0.005)

    print("\n Lade gespeicherte Punkte aus:", SAVE_FILE)
    punkte = load_points()
    print(f"✅ {len(punkte)} Punkte geladen\n")



    if len(punkte) == 0:
        print("❌ Keine Punkte gefunden!")
        return

    print("ENTER = nächster Punkt | q = Ende")

    for i, pose in enumerate(punkte):
        inp = input(f"Punkt {i+1}/{len(punkte)} → ENTER für Bewegung: ")
        if inp.lower() == "q":
            print("Beendet.")
            break

        print(f"➡️  Fahre zu Punkt {i+1}…")
        arm.set_start_state_to_current_state()
        normalize_quaternion(pose)
        arm.set_pose_target(pose)
        arm.go(wait=True)
        arm.stop()
        arm.clear_pose_targets()
        print("✅ erreicht\n")

        width = 0.02 + i*0.03
        print("width", width)
        set_width(gripper,width)  # 3 cm öffnen
        gripper.go(wait=True)
        print("Gripper executed")



    print("Alle Punkte abgefahren.")


def set_width(gripper, width):
    # Panda: max. 0.08m → jede Fingerhälfte 0.04m
    # Finger joint bewegt ~die halbe Öffnung
    target = width / 2.0
    joint_values = gripper.get_current_joint_values()
    gripper.set_joint_value_target("panda_finger_joint1",target)
    gripper.set_joint_value_target("panda_finger_joint2", target)
    gripper.go(wait=True)



if __name__ == "__main__":
    main()
