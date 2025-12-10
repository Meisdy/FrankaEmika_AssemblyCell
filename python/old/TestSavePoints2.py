#!/usr/bin/env python3
import sys, json
import rospy
import moveit_commander
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

SAVE_FILE = "punkte5.jsonl"

# --- How To Use Thi Script ---#
# rosrun topic_tools relay /franka_state_controller/joint_states /joint_states



# ---------- utils ----------
def normalize_quaternion(pose: Pose):
    import numpy as np
    q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], dtype=float)
    n = np.linalg.norm(q)
    if n > 0:
        q /= n
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q.tolist()

def wait_for_valid_time():
    # ROS time must be non-zero (handles leftover /use_sim_time state)
    for _ in range(50):
        if rospy.Time.now().to_sec() > 0:
            return True
        rospy.sleep(0.1)
    print("❌ ROS time not valid.")
    return False

def wait_for_joint_state():
    try:
        rospy.wait_for_message("/joint_states", JointState, timeout=5.0)
        return True
    except rospy.ROSException:
        print("❌ No /joint_states received in time.")
        return False

def load_points():
    points = {}
    try:
        with open(SAVE_FILE, "r") as f:
            for line in f:
                line = line.strip().rstrip(";")
                if not line:
                    continue
                try:
                    data = json.loads(line)
                    if not all(k in data for k in ("name","pos","quat")):
                        continue
                    p = Pose()
                    p.position.x, p.position.y, p.position.z = data["pos"]
                    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = data["quat"]
                    normalize_quaternion(p)
                    points[data["name"]] = p
                except json.JSONDecodeError:
                    continue
    except FileNotFoundError:
        pass
    return points

def save_point(name: str, pose: Pose):
    normalize_quaternion(pose)
    rec = {"name": name,
           "pos": [pose.position.x, pose.position.y, pose.position.z],
           "quat": [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]}
    with open(SAVE_FILE, "a") as f:
        f.write(json.dumps(rec) + "\n")

def delete_point(name: str):
    # rewrite file without the entry
    pts = load_points()
    if name not in pts:
        return False
    del pts[name]
    with open(SAVE_FILE, "w") as f:
        for n, p in pts.items():
            rec = {"name": n,
                   "pos": [p.position.x, p.position.y, p.position.z],
                   "quat": [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]}
            f.write(json.dumps(rec) + "\n")
    return True

def go_to_point(name: str, arm: MoveGroupCommander, points: dict, vel_scale=0.2, acc_scale=0.2):
    pose = points.get(name)
    if pose is None:
        print(f"❌ Point '{name}' not found.")
        return False
    normalize_quaternion(pose)
    arm.set_start_state_to_current_state()
    arm.set_max_velocity_scaling_factor(vel_scale)
    arm.set_max_acceleration_scaling_factor(acc_scale)

    ps = PoseStamped()
    ps.header.frame_id = arm.get_planning_frame()
    ps.pose = pose

    arm.set_pose_target(ps)
    ok = arm.go(wait=True)
    arm.stop()
    arm.clear_pose_targets()
    print("✅ Reached." if ok else "❌ Planning/Execution failed.")
    return ok

def set_width(gripper: MoveGroupCommander, width: float):
    # Panda: total open 0.08 → each finger ~ width/2
    target = max(0.0, min(0.04, width/2.0))
    gripper.set_joint_value_target({"panda_finger_joint1": target, "panda_finger_joint2": target})
    gripper.go(wait=True)

# ---------- main ----------
def main():
    rospy.init_node("teach_and_go_points", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    if not wait_for_valid_time() or not wait_for_joint_state():
        sys.exit(1)

    robot = RobotCommander()
    scene = PlanningSceneInterface()
    arm = MoveGroupCommander("panda_arm")
    gripper = MoveGroupCommander("panda_hand")

    # tighter but sane tolerances
    arm.set_goal_position_tolerance(0.003)
    arm.set_goal_orientation_tolerance(0.01)

    print(f"Planning frame: {arm.get_planning_frame()}")
    print("Loading points...")
    points = load_points()
    print("Loaded:", list(points.keys()) if points else "none")

    print("\nCommands:")
    print("  s <name>     : save current pose as <name>")
    print("  goto <name>     : move to saved pose")
    print("  list            : list points")
    print("  del <name>      : delete point")
    print("  grip <width_m>  : set gripper opening (meters, e.g. 0.03)")
    print("  q            : exit\n")

    while not rospy.is_shutdown():
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not cmd:
            continue

        if cmd == "q":
            break
        elif cmd == "list":
            print(sorted(points.keys()))
        elif cmd.startswith("s "):
            name = cmd.split(" ",1)[1].strip()
            if not name:
                print("❌ name required")
                continue
            pose = arm.get_current_pose().pose  # MoveIt frame = correct
            save_point(name, pose)
            points[name] = pose
            print(f"✅ Saved '{name}'")
        elif cmd.startswith("goto "):
            name = cmd.split(" ",1)[1].strip()
            go_to_point(name, arm, points)
        elif cmd.startswith("del "):
            name = cmd.split(" ",1)[1].strip()
            if delete_point(name):
                points.pop(name, None)
                print(f"✅ Deleted '{name}'")
            else:
                print(f"❌ '{name}' not found")
        elif cmd.startswith("grip "):
            try:
                w = float(cmd.split(" ",1)[1])
                set_width(gripper, w)
                print("✅ Gripper set")
            except Exception:
                print("❌ usage: grip 0.03")
        else:
            print("Unknown command")

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
