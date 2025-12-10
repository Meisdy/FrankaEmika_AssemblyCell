#!/usr/bin/env python3
import sys, json, shutil
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

SAVE_FILE = "punkte3.jsonl"
REF_NAME = "Assembly_Container"
CALIB_NAME = "Calibration"

# ------------ Utils ------------
def wait_for_valid_time():
    for _ in range(50):
        if rospy.Time.now().to_sec() > 0:
            return True
        rospy.sleep(0.1)
    print("❌ ROS time invalid.")
    return False

def wait_for_joint_state():
    try:
        rospy.wait_for_message("/joint_states", JointState, timeout=5.0)
        return True
    except:
        print("❌ No /joint_states.")
        return False

def load_points():
    pts = {}
    try:
        with open(SAVE_FILE, "r") as f:
            for line in f:
                if not line.strip():
                    continue
                obj = json.loads(line)
                pts[obj["name"]] = obj["pos"]
    except:
        pass
    return pts

def save_point(name: str, xyz):
    rec = {"name": name, "pos": xyz}
    with open(SAVE_FILE, "a") as f:
        f.write(json.dumps(rec) + "\n")

def delete_point(name: str):
    pts = load_points()
    if name not in pts:
        return False
    del pts[name]
    with open(SAVE_FILE, "w") as f:
        for n,p in pts.items():
            f.write(json.dumps({"name": n, "pos": p}) + "\n")
    return True

# ------------ Movement ------------
def go_to_point(name: str, arm: MoveGroupCommander, points):
    xyz = points.get(name)
    if xyz is None:
        print("❌ Not found:", name)
        return False

    ps = PoseStamped()
    ps.header.frame_id = arm.get_planning_frame()
    ps.pose.position.x = xyz[0]
    ps.pose.position.y = xyz[1]
    ps.pose.position.z = xyz[2]

    arm.set_pose_target(ps)
    ok = arm.go(wait=True)
    arm.stop()
    arm.clear_pose_targets()
    print("✅" if ok else "❌")
    return ok

# ------------ Alignment ------------
def align_points(filename: str):
    points_raw = []
    ref_old = None
    ref_new = None

    with open(filename, "r") as f:
        for line in f:
            if not line.strip():
                continue
            obj = json.loads(line)
            points_raw.append(obj)
            if obj["name"] == REF_NAME:
                ref_old = obj
            if obj["name"] == CALIB_NAME:
                ref_new = obj

    if ref_old is None:
        print("❌ No Assembly_Container in file.")
        return
    if ref_new is None:
        print("❌ No Calibration point in file.")
        return

    new_ref = ref_new["pos"]
    old_ref = ref_old["pos"]

    offset = [0,0,0.002]#[-old_ref[i] + new_ref[i] for i in range(3)]
    print("📐 Offset =", offset)

    shutil.copy(filename, filename + ".bak")

    for o in points_raw:
        if o["name"] == CALIB_NAME:
            o["pos"] = new_ref
        else:
            o["pos"] = [o["pos"][i] + offset[i] for i in range(3)]

    with open(filename, "w") as f:
        for o in points_raw:
            f.write(json.dumps(o) + "\n")

    print("✅ Updated. Backup created.")


# ------------ Main ------------
def main():
    rospy.init_node("teach_points", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    if not wait_for_valid_time() or not wait_for_joint_state():
        sys.exit(1)

    arm = MoveGroupCommander("panda_arm")
    points = load_points()

    print("Loaded points:", list(points.keys()))
    print("\nCommands:",
          "\n  s <name>      = save current xyz",
          "\n  goto <name>   = move to xyz",
          "\n  align         = apply offset to all (robot does NOT move)",
          "\n  del <name>    = delete point",
          "\n  list          = show points",
          "\n  q             = quit\n")

    while not rospy.is_shutdown():
        cmd = input("> ").strip()
        if not cmd: continue

        if cmd == "q":
            break
        elif cmd == "list":
            print(sorted(points.keys()))

        elif cmd.startswith("s "):
            name = cmd.split(" ",1)[1]
            pose = arm.get_current_pose().pose
            xyz = [pose.position.x, pose.position.y, pose.position.z]
            save_point(name, xyz)
            points[name] = xyz
            print("✅ Saved.")

        elif cmd.startswith("goto "):
            name = cmd.split(" ",1)[1]
            go_to_point(name, arm, points)

        elif cmd == "align":
            align_points(SAVE_FILE)
            points = load_points()

        elif cmd.startswith("del "):
            name = cmd.split(" ",1)[1]
            if delete_point(name):
                points.pop(name, None)
                print("✅ Deleted.")

        else:
            print("Unknown.")

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
