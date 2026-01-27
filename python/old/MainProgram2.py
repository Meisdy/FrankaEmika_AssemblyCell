import rospy
import moveit_commander
import numpy as np
import json
import time
from geometry_msgs.msg import Pose
from moveit_commander.exception import MoveItCommanderException
import actionlib
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryGoal

import PLC_Connection_Franka

SAVE_FILE = "positions.jsonl"       # file in which the postions are saved
BUFFER_FILE = "buffer_state.json"   # file in which the buffer states are saved

PLC_AMS = "5.89.239.104.1.1"
PLC_IP = "169.254.215.116"

mapping = {
    1: ["Blue", "Red", "Yellow", "", "", "", "", "", "Uncertain"],                       # Color
    2: ["Cyl", "Hex", "Plate", "Plate_right", "", "", "", "", "Uncertain"],              # Shape
    3: ["1", "2", "3", "4", "5", "6"],                          # Position
    4: ["assemble", "dissassemble", "rotate", "discard"]    # Mode
}
next_task = 0

def main():
    global next_task
    print("Start")
    plc = PLC_Connection_Franka.PLC_Connection_Franka(PLC_IP, PLC_AMS, 851)
    #plc = PLC_Connection_Franka.PLC_Connection_Franka("", "", 0, simulate=True)    #simulation 
    print("Back in main program")
    
    # --- Set up FRANKA ---
    rospy.init_node("franka_go_points")
    moveit_commander.roscpp_initialize([])

    arm = moveit_commander.MoveGroupCommander("panda_arm")
    gripper = moveit_commander.MoveGroupCommander("panda_hand")

    arm.set_goal_orientation_tolerance(0.005)
    arm.set_goal_position_tolerance(0.005)

    use_pilz(arm)
    set_mode_ptp(arm)
    

    # --- Load positions ---

    print("\n Load saved positions from:", SAVE_FILE)
    positions = load_points()
    buffer_state = initialize_buffer_state(positions)
    save_buffer_state(buffer_state)
    print("Loaded positions:", list(positions.keys()))

    # test_code = ["1242", "1162","1352","1351", "1241", "1161", "2321", "2211", "2131", "1242", "1162","1352", "2212", "2132","2322", "3351", "3241", "3161","3242", "3162","3352"]
    print("Waiting for start signal")
    while not rospy.is_shutdown():

        plc.set(mwFrStatus=2)    # Status = Ready
        mwFrStart, mwFrRefill, mwFrFinished, mwFrStatus, mwFrVisiCode, mwAssemblyColor, mwFrState = plc.get("mwFrStart", "mwFrRefill", "mwFrFinished", "mwFrStatus", "mwFrVisiCode", "mwAssemblyColor","mwFrState")

        #mxFrTaskCode = test_code    # just for test purpose

        if not mwFrStart:
            continue
        else:
            try:

                # --- Generate task-list from vision code ---
                print("AssemblyColor", mwAssemblyColor)
                mxFrTaskCode = create_tasklist_franka(mwAssemblyColor,mwFrVisiCode)
                print(mxFrTaskCode)
                plc.set(mwFrFinished=False)
                plc.set(mwFrStatus=3)    # Status = Running 

                # --- Execute robot tasks from list ---

                print("# -- Go to home -- ")
                MoveJ_J("Home", positions, arm)

                Pick_From_Conveyor(positions,arm,gripper)

                for i in range(next_task,len(mxFrTaskCode)):
                    print(decode(mxFrTaskCode[i]))
                    next_task= i+1
                    execute_code(mxFrTaskCode[i], positions, arm, gripper)
                next_task= 0

                Place_On_Conveyor(positions, arm, gripper)

                plc.set(mwFrFinished=True)

            # --- Exceptions in case of error / buffer empty

            except MoveItCommanderException:
                plc.set(mwFrStatus=404)
                print("❌ Robot movement was stopped!")
                arm.stop()
                arm.clear_pose_targets()
                recover_franka()
                MoveJ("Home", positions, arm)
                continue

            except BufferEmptyError as e:
                print("Buffer empty")
                plc.set(QxFrRefill=True)
                plc.set(mwFrStatus=405)
                while not plc.get("mwFrRefilled")[0]:
                    print("Buffer empty")
                    time.sleep(0.5)
                plc.set(QxFrRefill=False)
       

def recover_franka():
    print("⏳ Recovering robot...")
    client = actionlib.SimpleActionClient(
        '/franka_control/error_recovery',
        ErrorRecoveryAction
    )
    client.wait_for_server()
    client.send_goal(ErrorRecoveryGoal())
    client.wait_for_result()
    print("✅ Franka is ready to rock and roll!")


def execute_code(code, positions, arm, gripper):
    decoded = decode(code)
    pos_name = f"Buffer_{decoded[1]}_{decoded[0]}"
    if decoded[3] == "assemble":
        Assemble(pos_name, positions, arm, gripper, decoded[2])
    elif decoded[3] == "dissassemble":
        Disassemble(pos_name, positions, arm, gripper, decoded[2])
    elif decoded[3] == "rotate":
        print("Rotate function")
        Rotate(pos_name, positions, arm, gripper, decoded[2])
    elif decoded[3] == "discard":
        print("Discard function")
        Discard(pos_name, positions, arm, gripper, decoded[2])


def decode(code):
    out = []
    for i in range(4):
        digit = int(code[i]) - 1
        out.append(mapping[i+1][digit])
    return out


# --------------------------
#     Robot Routines
# --------------------------

# --- Pick up container from conveyor and place on assembly-position ---

def Pick_From_Conveyor(positions, arm, gripper):
    print("# -- Go to Home -- ")
    MoveJ_J("Home", positions, arm)

    print("# -- Open Gripper -- ")
    gripper_open(gripper)

    print("# -- Approach Conveyor -- ")
    MoveJ_J("Approach_Conveyor", positions, arm)
    
    print("# -- Approach Conveyor -- ")
    MoveJ("Conveyor_PickUp", positions, arm, offset=[0.0, 0.0, 0.05])

    print("# -- Go to Conveyor -- ")
    MoveL("Conveyor_PickUp", positions, arm)

    print("# -- Close Gripper -- ")
    gripper_close(gripper)

    print("# -- Approach Conveyor -- ")
    MoveL("Conveyor_PickUp", positions, arm, offset=[0.0, 0.0, 0.05])

    print("# -- Approach Conveyor -- ")
    MoveJ_J("Approach_Conveyor", positions, arm)

    print("# -- Go to Home -- ")
    MoveJ_J("Home", positions, arm)

    print("# -- Approach Assembly Container -- ")
    MoveJ("Assembly_Container", positions, arm, offset=[0.0, 0.0, 0.05])

    print("# -- Go to Assembly Container -- ")
    MoveL("Assembly_Container", positions, arm,  offset=[0.0, 0.0, 0.01])

    print("# -- Open Gripper -- ")
    gripper_open(gripper)

    print("# -- Approach Assembly Container -- ")
    MoveL("Assembly_Container", positions, arm, offset=[0.0, 0.0, 0.05])


# --- Pick up container from assembly-position and place it on conveyor ---

def Place_On_Conveyor(positions, arm, gripper):
    print("# -- Go to Home -- ")
    MoveJ_J("Home", positions, arm)

    print("# -- Open Gripper -- ")
    gripper_open(gripper)

    print("# -- Approach Assembly Container -- ")
    MoveJ("Assembly_Container", positions, arm, offset=[0.0, 0.0, 0.05])

    print("# -- Go to Assembly Container -- ")
    MoveL("Assembly_Container", positions, arm,  offset=[0.0, 0.0, 0.01])

    print("# -- Close Gripper -- ")
    gripper_close(gripper)

    print("# -- Approach Assembly Container -- ")
    MoveL("Assembly_Container", positions, arm, offset=[0.0, 0.0, 0.05])

    print("# -- Go to Home -- ")
    MoveJ_J("Home", positions, arm)

    print("# -- Approach Conveyor -- ")
    MoveJ_J("Approach_Conveyor", positions, arm)
    
    print("# -- Approach Conveyor -- ")
    MoveJ("Conveyor_PickUp", positions, arm, offset=[0.0, 0.0, 0.05])

    print("# -- Go to Conveyor -- ")
    MoveL("Conveyor_PickUp", positions, arm, offset=[0.0, 0.0, 0.01])

    print("# -- Open Gripper -- ")
    gripper_open(gripper)

    print("# -- Approach Conveyor -- ")
    MoveL("Conveyor_PickUp", positions, arm, offset=[0.0, 0.0, 0.05])

    print("# -- Approach Conveyor -- ")
    MoveJ_J("Approach_Conveyor", positions, arm)

    print("# -- Go to Home -- ")
    MoveJ_J("Home", positions, arm)


# --- Take part from buffer and assemble on container

def Assemble(name, positions, arm, gripper, position):
    buffer_state = load_buffer_state()
    buffer_state[name] -= 1
    save_buffer_state(buffer_state)

    if "Plate" in name:
        offset = transform_offset_deg([0,0,0.1], 0, -15, 0)
        grip_width = 0.02
        buffer = "Approach_Buffer_Plate"
    else:
        offset = transform_offset_deg([0,0,0.1], 15, 0, 0)
        grip_width = None
        buffer = "Approach_Buffer_Cyl_Hex"

    print("# -- Open Gripper -- ")
    gripper_open(gripper, grip_width)

    print("# -- Approach Buffer -- ")
    MoveJ_J(buffer, positions, arm)

    print(f"# -- Approach Buffer {name} -- ")
    MoveJ(name, positions, arm, offset=offset)

    print(f"# -- Go To Buffer {name} -- ")
    MoveL(name, positions, arm)

    print("# -- Close Gripper -- ")
    gripper_close(gripper)

    print(f"# -- Approach Buffer {name} -- ")
    MoveL(name, positions, arm, offset=offset)

    print("# -- Approach Assembly  -- ")
    MoveJ_J("Approach_Assembly", positions, arm)

    print(f"# -- Approach pos {position} -- ")
    MoveJ("Assembly_" + position, positions, arm, offset=[0.0, 0.0, 0.08])

    print(f"# -- Go to pos {position} -- ")
    MoveL("Assembly_" + position, positions, arm, offset=[0.0, 0.0, 0.01])

    print("# -- Open Gripper -- ")
    gripper_open(gripper, grip_width)

    print(f"# -- Approach pos {position} -- ")
    MoveL("Assembly_" + position, positions, arm, offset=[0.0, 0.0, 0.08])


    check_buffer_status()


# --- Disassemble container parts and add them to buffer (if possible) ---

def Disassemble(name, positions, arm, gripper, position):
    if "right" in name:
                Rotate(name[:-6], positions, arm, gripper, position)
                name = name.replace("right_", "")
                print(name)

    buffer_state = load_buffer_state()
    buffer_state[name] += 1
    save_buffer_state(buffer_state)

    shape = name.split("_")[-2]
    max_cap = 4 if shape in ("Hex", "Cyl") else 3

    if "Plate" in name:
            Approach_offset = transform_offset_deg([0.13,0,0.1], 0, -15, 0)
            offset = transform_offset_deg([0.13,0,0.003], 0, -15, 0)
            grip_width = 0.02
            buffer = "Approach_Buffer_Plate"

    else:
        Approach_offset = transform_offset_deg([0,0.175,0.1], 15, 0, 0)
        offset = transform_offset_deg([0,0.175,0.003], 15, 0, 0)
        grip_width = None
        buffer = "Approach_Buffer_Cyl_Hex"

    print("# -- Open Gripper -- ")
    gripper_open(gripper,grip_width)
    
    print("# -- Approach Assembly  -- ")
    MoveJ_J("Approach_Assembly", positions, arm)

    print(f"# -- Approach pos {position} -- ")
    MoveJ("Assembly_" + position, positions, arm, offset=[0.0, 0.0, 0.08])

    print(f"# -- pos  {position} -- ")
    MoveL("Assembly_" + position, positions, arm)

    print("# -- Close Gripper -- ")
    gripper_close(gripper)

    print(f"# -- Approach Assembly_{position} -- ")
    MoveL("Assembly_" + position, positions, arm, offset=[0.0, 0.0, 0.08])

    if buffer_state[name] > max_cap:
        print(f"⚠ {name} voll → Discard")
        buffer_state[name] -= 1
        save_buffer_state(buffer_state)

        MoveJ_J("Discard", positions, arm)
        gripper_open(gripper)
        return False

    print("# -- Approach general Buffer")
    MoveJ_J(buffer, positions, arm)

    print(f"# -- Approach Buffer {name} -- ")
    MoveJ(name, positions, arm, offset=Approach_offset)

    print(f"# -- Approach Buffer {name} -- ")
    MoveL(name, positions, arm, offset=offset)

    print("# -- Open Gripper -- ")
    gripper_open(gripper,grip_width)

    print(f"# -- Approach Buffer {name} -- ")
    MoveL(name, positions, arm, offset=Approach_offset)

    return True


# --- Discarding part into box ---

def Discard(name, positions, arm, gripper, position):
    grip_width = 0.02

    print("# -- Open Gripper -- ")
    gripper_open(gripper,grip_width)
    
  
    print("# -- Approach Assembly  -- ")
    MoveJ_J("Approach_Assembly", positions, arm)

    print("# -- Approach pos  -- ")
    MoveJ("Assembly_" + position, positions, arm, offset=[0.0, 0.0, 0.08])

    print("# -- pos  -- ")
    MoveL("Assembly_" + position, positions, arm)

    print("# -- Close Gripper -- ")
    gripper_close(gripper)

    print(f"# -- Approach Assembly_{position} -- ")
    MoveL("Assembly_" + position, positions, arm, offset=[0.0, 0.0, 0.08])

    MoveJ_J("Discard", positions, arm)
    gripper_open(gripper)
    return True


# --- Rotate part 180 degrees and place it again at same position ---

def Rotate(name, positions, arm, gripper, position):
    grip_width = 0.02
    print("# -- Open Gripper -- ")
    gripper_open(gripper, grip_width)

    print("# -- Approach Assembly  -- ")
    MoveJ_J("Approach_Assembly", positions, arm)

    print("# -- Approach pos  -- ")
    MoveJ("Assembly_" + position, positions, arm, offset=[0.0, 0.0, 0.08])

    print("# -- Go to pos  -- ")
    MoveL("Assembly_" + position, positions, arm, offset=[0.0, 0.0, 0.0])

    print("# -- Close Gripper -- ")
    gripper_close(gripper)

    print("# -- Approach pos -- ")
    MoveL("Assembly_" + position, positions, arm, offset=[0.0, 0.0, 0.08])

    print("# -- Approach Assembly  -- ")
    MoveJ_J("Approach_Assembly_Rot", positions, arm)

    print("# -- Go to pos  -- ")
    MoveJ("Assembly_Rot_" + position, positions, arm, offset=[0.0, 0.0, 0.08])

    print("# -- Go to pos  -- ")
    MoveL("Assembly_Rot_" + position, positions, arm, offset=[0.0, 0.0, 0.01])

    print("# -- Open Gripper -- ")
    gripper_open(gripper)

    print("# -- Approach pos -- ")
    MoveL("Assembly_Rot_" + position, positions, arm, offset=[0.0, 0.0, 0.08])



# --------------------------
#     MOTION FUNCTIONS
# --------------------------

# Linear movement 

def MoveL(name, positions, arm, offset=None):

    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(0.1)
    set_mode_lin(arm)
    go_to_point_pose_only(name, positions, arm, offset)


# Non-linear movement 

def MoveJ(name, positions, arm, offset=None):

    arm.set_max_velocity_scaling_factor(1.0)
    arm.set_max_acceleration_scaling_factor(1.0)

    set_mode_ptp(arm)
    go_to_point_pose_only(name, positions, arm, offset)


# Joint control movement (no offset support)

def MoveJ_J(name, positions, arm):

    arm.set_max_velocity_scaling_factor(1.0)
    arm.set_max_acceleration_scaling_factor(1.0)

    entry = positions.get(name)
    if entry is None:
        print(f"⚠ Position '{name}' not found.")
        return False

    joints = entry["joints"]
    if joints is None:
        raise MoveItCommanderException(f"❌ '{name}' has no joint data.")

    set_mode_ptp(arm)
    arm.set_joint_value_target(joints)

    success = arm.go(wait=True)
    arm.stop()
    arm.clear_pose_targets()

    if not success:
        raise MoveItCommanderException("❌ Joint PTP failed.")

    return success


# For PTP/LIN movement

def go_to_point_pose_only(name, positions, arm, offset=None):
    entry = positions.get(name)
    if entry is None:
        print(f"⚠ Position '{name}' not found.")
        return False

    pose = entry["pose"]

    pose_goal = Pose()
    pose_goal.position.x = pose.position.x
    pose_goal.position.y = pose.position.y
    pose_goal.position.z = pose.position.z
    pose_goal.orientation = pose.orientation

    if offset is not None:
        pose_goal.position.x += offset[0]
        pose_goal.position.y += offset[1]
        pose_goal.position.z += offset[2]

    normalize_quaternion(pose_goal)
    arm.set_pose_target(pose_goal)

    success = arm.go(wait=True)
    arm.stop()
    arm.clear_pose_targets()

    if not success:
        raise MoveItCommanderException("❌ Pose-PTP/LIN failed.")

    return success


# --------------------------
#      GRIPPER
# --------------------------

def gripper_open(gripper, width=None):
    if width is None:
        width = 0.04
    set_width(gripper, width)
    gripper.go(wait=True)


def gripper_close(gripper):
    set_width(gripper, 0.0005)
    gripper.go(wait=True)


def set_width(gripper, width):
    target = width / 2.0
    gripper.set_joint_value_target("panda_finger_joint1", target)
    gripper.set_joint_value_target("panda_finger_joint2", target)
    gripper.go(wait=True)


# --------------------------
#       PLANNERS
# --------------------------

def set_mode_ptp(arm):
    arm.set_planner_id("PTP")
    arm.set_planning_pipeline_id("pilz_industrial_motion_planner")
    arm.set_max_velocity_scaling_factor(1.0)

def set_mode_lin(arm):
    arm.set_planner_id("LIN")
    arm.set_planning_pipeline_id("pilz_industrial_motion_planner")
    arm.set_max_velocity_scaling_factor(1.0)

def use_pilz(arm):
    arm.set_planner_id("PTP")
    arm.set_planning_pipeline_id("pilz_industrial_motion_planner")


# --------------------------
#   UTILITIES
# --------------------------

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
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q

def load_points():
    positions = {}
    try:
        with open(SAVE_FILE, "r") as f:
            for line in f:
                if not line.strip():
                    continue

                data = json.loads(line.strip())

                pose = Pose()
                pose.position.x = data["pos"][0]
                pose.position.y = data["pos"][1]
                pose.position.z = data["pos"][2]
                pose.orientation.x = data["quat"][0]
                pose.orientation.y = data["quat"][1]
                pose.orientation.z = data["quat"][2]
                pose.orientation.w = data["quat"][3]

                joints = data.get("joints", None)

                positions[data["name"]] = {
                    "pose": pose,
                    "joints": joints
                }

        
    except FileNotFoundError:
        print(f"⚠ File not found: {SAVE_FILE}")

    return positions


# --- Buffer state ---

def initialize_buffer_state(positions):
    buffer_state = {}
    for name in positions.keys():
        if name.startswith("Buffer_"):
            _, shape, _ = name.split("_")
            if shape in ("Hex", "Cyl"):
                buffer_state[name] = 4
            elif shape == "Plate":
                buffer_state[name] = 3
            else:
                buffer_state[name] = 0
    return buffer_state

def save_buffer_state(buffer_state):
    with open(BUFFER_FILE, "w") as f:
        json.dump(buffer_state, f)

def load_buffer_state():
    try:
        with open(BUFFER_FILE, "r") as f:
            return json.load(f)
    except:
        return {}

def check_buffer_status():
    with open(BUFFER_FILE, "r") as f:
        data = json.load(f)
    for name, value in data.items():
        if value == 0:
            raise BufferEmptyError(f"Buffer {name} is empty!")


# Workobject transformation

def transform_offset_deg(local_offset, roll_deg, pitch_deg, yaw_deg):
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(yaw_deg)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])
    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0, 0, 1]
    ])

    R = Rz @ Ry @ Rx
    return R.dot(np.array(local_offset))


# Create tasklist for robot form vision code

def create_tasklist_franka(assembly_color, detected_objects):
    # inputs: assembly_color (str), detected_objects (array))
    # output: tasklist (array)
    print(assembly_color)

    print("Detected objects visiCode: ", detected_objects)

    # map color strings to numeric codes
    COLOR_CODE = {'none': 0, 'blue': 1, 'red': 2, 'yellow': 3}
    color = COLOR_CODE.get(assembly_color, 0)

    # initialize empty tasklist
    tasklist = []

    # --------- determine position order ---------
    position_order = [0] * 7
    for idx, obj in enumerate(detected_objects):
        code2 = obj[0] if isinstance(obj, list) else obj
        position = code2 % 10
        position_order[position] = idx

    print(f"Position order: {position_order[1:]}")

    # --------- check the upper objects ---------
    # position 2
    code2 = detected_objects[position_order[2]]
    shape2 = (code2 // 10) % 10
    obj_color2 = code2 // 100

    # position 1
    code1 = detected_objects[position_order[1]]
    shape1 = (code1 // 10) % 10
    obj_color1 = code1 // 100

    # position 3
    code3 = detected_objects[position_order[3]]
    shape3 = (code3 // 10) % 10
    obj_color3 = code3 // 100

    print(color)
    print(shape2)
    print(obj_color2)
    # --- check if it is a plate_left ---
    if shape2 == 3 and obj_color2 == color:
        print("Plate left detected with correct color at position 2")

        if  shape1 == 2 and obj_color1 == color:
            print("Hexagon detected with correct color at position 1")

        elif shape1 == 5 and obj_color1 == 5:
            print("Empty position at position 1")
            tasklist.append(str(color * 100 + 20 + 1) + "1")

        elif shape1 == 9 and obj_color1 == 9:
            print("Uncertain object at position 1")
            tasklist.append(str(detected_objects[position_order[1]]) + "4")
            tasklist.append(str(color * 100 + 20 + 1) + "1")
            
        else:
            print("Hexagon at position 1 incorrect")
            tasklist.append(str(detected_objects[position_order[1]]) + "2")
            tasklist.append(str(color * 100 + 20 + 1) + "1")

        if shape3 == 1 and obj_color3 == color:
            print("Circle detected with correct color at position 3")
        
        elif shape3 == 5 and obj_color3 == 5:
            print("Empty position at position 3")
            tasklist.append(str(color * 100 + 10 + 3) + "1")

        elif shape3 == 9 and obj_color3 == 9:
            print("Uncertain object at position 3")
            tasklist.append(str(detected_objects[position_order[3]]) + "4")
            tasklist.append(str(color * 100 + 10 + 3) + "1")

        else:
            print("Circle at position 3 incorrect")
            tasklist.append(str(detected_objects[position_order[3]]) + "2")
            tasklist.append(str(color * 100 + 10 + 3) + "1")

    # --- check if it is a plate_right ---
    elif shape2 == 4 and obj_color2 == color:
        print("Plate right detected with correct color at position 2")
        if shape1 == 5 and obj_color1 == 5:
            print("Empty position at position 1")
        elif shape1 == 9 and obj_color1 == 9:
            print("Uncertain object at position 1")
            tasklist.append(str(detected_objects[position_order[1]]) + "4")
        else: 
            print("Hexagon at position 1 incorrect")
            tasklist.append(str(detected_objects[position_order[1]]) + "2")

        if shape3 == 5 and obj_color3 == 5:
            print("Empty position at position 3")
        elif shape3 == 9 and obj_color3 == 9:
            print("Uncertain object at position 3")
            tasklist.append(str(detected_objects[position_order[3]]) + "4")
        else: 
            print("Circle at position 3 incorrect")
            tasklist.append(str(detected_objects[position_order[3]]) + "2")

        tasklist.append(str(detected_objects[position_order[2]]) + "3")
        tasklist.append(str(color * 100 + 20 + 1) + "1")
        tasklist.append(str(color * 100 + 10 + 3) + "1")

    # --- incorrect object at position 2 ---
    else:
        print("Plate at position 2 incorrect")
        if shape1 == 5 and obj_color1 == 5:
            print("Empty position at position 1")
        elif shape1 == 9 and obj_color1 == 9:
            print("Uncertain object at position 1")
            tasklist.append(str(detected_objects[position_order[1]]) + "4")
        else: 
            print("Hexagon at position 1 incorrect")
            tasklist.append(str(detected_objects[position_order[1]]) + "2")

        if shape3 == 5 and obj_color3 == 5:
            print("Empty position at position 3")
        elif shape3 == 9 and obj_color3 == 9:
            print("Uncertain object at position 3")
            tasklist.append(str(detected_objects[position_order[3]]) + "4")
        else: 
            print("Circle at position 3 incorrect")
            tasklist.append(str(detected_objects[position_order[3]]) + "2")

        if shape2 == 5 and obj_color2 == 5:
            print("Empty position at position 2")
        elif shape2 == 9 and obj_color2 == 9:
            print("Uncertain object at position 2")
            tasklist.append(str(detected_objects[position_order[2]]) + "4")
        else: 
            print("Plate at position 2 incorrect")
            tasklist.append(str(detected_objects[position_order[2]]) + "2")
        
        tasklist.append(str(color * 100 + 30 + 2) + "1")
        tasklist.append(str(color * 100 + 20 + 1) + "1")
        tasklist.append(str(color * 100 + 10 + 3) + "1")

    # --------- check the lower objects ---------
    # position 5
    code5 = detected_objects[position_order[5]]
    shape5 = (code5 // 10) % 10
    obj_color5 = code5 // 100

    # position 4
    code4 = detected_objects[position_order[4]]
    shape4 = (code4 // 10) % 10
    obj_color4 = code4 // 100

    # position 6
    code6 = detected_objects[position_order[6]]
    shape6 = (code6 // 10) % 10
    obj_color6 = code6 // 100

    # --- check if it is a plate_left ---
    if shape5 == 3 and obj_color5 == color:
        print("Plate left detected with correct color at position 5")

        if shape4 == 2 and obj_color4 == color:
            print("Hexagon detected with correct color at position 4")

        elif shape4 == 5 and obj_color4 == 5:
            print("Empty position at position 4")
            tasklist.append(str(color * 100 + 20 + 4) + "1")

        elif shape4 == 9 and obj_color4 == 9:
            print("Uncertain object at position 4")
            tasklist.append(str(detected_objects[position_order[4]]) + "4")
            tasklist.append(str(color * 100 + 20 + 4) + "1")

        else:
            print("Hexagon at position 4 incorrect")
            tasklist.append(str(detected_objects[position_order[4]]) + "2")
            tasklist.append(str(color * 100 + 20 + 4) + "1")

        if shape6 == 1 and obj_color6 == color:
            print("Circle detected with correct color at position 6")

        elif shape6 == 5 and obj_color6 == 5:
            print("Empty position at position 6")
            tasklist.append(str(color * 100 + 10 + 6) + "1")

        elif shape6 == 9 and obj_color6 == 9:
            print("Uncertain object at position 6")
            tasklist.append(str(detected_objects[position_order[6]]) + "4")
            tasklist.append(str(color * 100 + 10 + 6) + "1")

        else:
            print("Circle at position 6 incorrect")
            tasklist.append(str(detected_objects[position_order[6]]) + "2")
            tasklist.append(str(color * 100 + 10 + 6) + "1")

    # --- check if it is a plate_right ---
    elif shape5 == 4 and obj_color5 == color:
        print("Plate right detected with correct color at position 5")
        if shape4 == 5 and obj_color4 == 5:
            print("Empty position at position 4")
        elif shape4 == 9 and obj_color4 == 9:
            print("Uncertain object at position 4")
            tasklist.append(str(detected_objects[position_order[4]]) + "4")
        else:
            print("Hexagon at position 4 incorrect")
            tasklist.append(str(detected_objects[position_order[4]]) + "2")

        if shape6 == 5 and obj_color6 == 5:
            print("Empty position at position 6")
        elif shape6 == 9 and obj_color6 == 9:
            print("Uncertain object at position 6")
            tasklist.append(str(detected_objects[position_order[6]]) + "4")
        else:
            print("Circle at position 6 incorrect")
            tasklist.append(str(detected_objects[position_order[6]]) + "2")

        tasklist.append(str(detected_objects[position_order[5]]) + "3")
        tasklist.append(str(color * 100 + 20 + 4) + "1")
        tasklist.append(str(color * 100 + 10 + 6) + "1")

    # --- incorrect object at position 5 ---
    else:
        print("Plate at position 5 incorrect")
        if shape4 == 5 and obj_color4 == 5:
            print("Empty position at position 4")
        elif shape4 == 9 and obj_color4 == 9:
            print("Uncertain object at position 4")
            tasklist.append(str(detected_objects[position_order[4]]) + "4")
        else:
            print("Hexagon at position 4 incorrect")
            tasklist.append(str(detected_objects[position_order[4]]) + "2")

        if shape6 == 5 and obj_color6 == 5:
            print("Empty position at position 6")
        elif shape6 == 9 and obj_color6 == 9:
            print("Uncertain object at position 6")
            tasklist.append(str(detected_objects[position_order[6]]) + "4")
        else:
            print("Circle at position 6 incorrect")
            tasklist.append(str(detected_objects[position_order[6]]) + "2")

        if shape5 == 5 and obj_color5 == 5:
            print("Empty position at position 5")
        elif shape5 == 9 and obj_color5 == 9:
            print("Uncertain object at position 5")
            tasklist.append(str(detected_objects[position_order[5]]) + "4")
        else:
            print("Plate at position 5 incorrect")
            tasklist.append(str(detected_objects[position_order[5]]) + "2")

        tasklist.append(str(color * 100 + 30 + 5) + "1")
        tasklist.append(str(color * 100 + 20 + 4) + "1")
        tasklist.append(str(color * 100 + 10 + 6) + "1")

    return tasklist

class BufferEmptyError(Exception):
    """Triggerd when buffer is empty"""


if __name__ == "__main__":
    main()
