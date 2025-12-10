#!/usr/bin/env python3
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np
import json
import time
import json
from geometry_msgs.msg import Pose
import numpy as np
from moveit_commander.exception import MoveItCommanderException
import actionlib
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryGoal

SAVE_FILE = "punkte4.jsonl"
BUFFER_FILE = "buffer_state.json"


#starting at 1
mapping = {
    1: ["Red", "Blue", "Yellow", "", ""],        # Color
    2: ["Cyl", "Hex", "Plate", "", ""], # Shape
    3: ["1", "2", "3", "4", "5", "6"],                # Position
    4: ["assemble", "dissassemble", "", "", ""]  # Mode
}

def main():
    

    rospy.init_node("franka_go_points")
    moveit_commander.roscpp_initialize([])

    arm = moveit_commander.MoveGroupCommander("panda_arm")
    gripper = moveit_commander.MoveGroupCommander("panda_hand")

    # ✅ Orientation wird ernst genommen
    arm.set_goal_orientation_tolerance(0.005)
    arm.set_goal_position_tolerance(0.005)

    use_pilz(arm)      # Arm: PTP/LIN
    use_ompl(gripper)  # Gripper: O', 'Hex', '4', 'assemble']MPL forever
    set_mode_ptp(arm)

    print("\n Lade gespeicherte Punkte aus:", SAVE_FILE)
    punkte = load_points()
    buffer_state = initialize_buffer_state(punkte)
    save_buffer_state(buffer_state)

    print("Geladene Punkte:", list(punkte.keys()))

    print("# -- Go to Home -- #")
    MoveJ("Home", punkte, arm)

    test_codes = [ "2321", "2231", "2111"] #,"2232", "2112","2322","1351", "1261", "1141", "2322", "2232", "2112", "1352", "1262", "1142", ]

    while not rospy.is_shutdown():
        # TODO Get PLC inputs
        # TODO Is PLC ready, not ready, when buffers are empty
        try:
            # Get container from conveyor
            #PickUp_Conveyor(punkte,arm,gripper)

            # Decode the code and execute till non is left
            for i in range(len(test_codes)):
                print(decode(test_codes[i]))
                execute_code(test_codes[i],punkte,arm,gripper)

            # Then place it on the AGV
            PlaceOn_AGV(punkte,arm,gripper)

            # TODO Send Buffer State json
            # TODO Send RobState TaskFinished
            
           

        except MoveItCommanderException:
            print("❌ Bewegung wurde vom Roboter gestoppt!")

            arm.stop()
            arm.clear_pose_targets()

            recover_franka()

            MoveJ("Home", punkte, arm)
            continue



  
   

def recover_franka():
    print("⏳ Recovery wird ausgeführt...")

    client = actionlib.SimpleActionClient(
        '/franka_control/error_recovery',
        ErrorRecoveryAction
    )

    client.wait_for_server()
    client.send_goal(ErrorRecoveryGoal())
    client.wait_for_result()

    print("✅ Franka ist wieder freigeschaltet!")

def execute_code(code,punkte, arm, gripper):
    decoded = decode(code)
    pos_name = f"Buffer_{decoded[1]}_{decoded[0]}"
    if decoded[3] == "assemble":
        Assemble(pos_name, punkte, arm, gripper, decoded[2])
    elif decoded[3] == "dissassemble":
        Disassemble(pos_name, punkte, arm, gripper, decoded[2])


def decode(code):
    out = []
    for i in range(4):
        digit = int(code[i]) - 1
        out.append(mapping[i+1][digit])  # Keys = int
    return out



def PlaceOn_AGV(punkte, arm, gripper):
    
    print("# -- Approach Assembly Container -- ")
    MoveJ("Assembly_Container", punkte, arm, offset=[0.0, 0.0, 0.05])

    print("# -- Open Gripper -- ")
    gripper_open(gripper)

    print("# -- Go to Assembly Container -- ")
    MoveL("Assembly_Container", punkte, arm)

    print("# -- Close Gripper -- ")
    gripper_close(gripper)

    print("# -- Approach Assembly Container -- ")
    MoveL("Assembly_Container", punkte, arm, offset=[0.0, 0.0, 0.05])

    print("# -- Approach AGV -- ")
    MoveJ("AGV", punkte, arm, offset=[0.0, 0.0, 0.05])

    print("# -- Go To AGV -- ")
    MoveL("AGV", punkte, arm)

    print("# -- Open Gripper -- ")
    gripper_open(gripper)

    print("# -- Approach AGV -- ")
    MoveJ("AGV", punkte, arm, offset=[0.0, 0.0, 0.05])

    print("# -- Go to Home -- ")
    MoveJ("Home", punkte, arm)
   

    

def PickUp_Conveyor(punkte, arm, gripper):

    print("# -- Go to Home -- ")
    MoveJ("Home", punkte, arm)

    print("# -- Open Gripper -- ")
    gripper_open(gripper)

    print("# -- Approach Conveyor -- ")
    MoveJ("Conveyor_PickUp", punkte, arm, offset=[0.0, 0.0, 0.05])

    print("# -- Go to Conveyor -- ")
    MoveL("Conveyor_PickUp", punkte, arm)

    print("# -- Close Gripper -- ")
    gripper_close(gripper)

    print("# -- Approach Conveyor -- ")
    MoveL("Conveyor_PickUp", punkte, arm, offset=[0.0, 0.0, 0.05])

    print("# -- Approach Assembly Container -- ")
    MoveJ("Assembly_Container", punkte, arm, offset=[0.0, 0.0, 0.05])

    print("# -- Go to Assembly Container -- ")
    MoveL("Assembly_Container", punkte, arm)

    print("# -- Open Gripper -- ")
    gripper_open(gripper)

    print("# -- Approach Assembly Container -- ")
    MoveL("Assembly_Container", punkte, arm, offset=[0.0, 0.0, 0.05])

def Disassemble(name, punkte, arm, gripper, position):
    # Buffer aus Datei laden
    buffer_state = load_buffer_state()
    # Einfügen
    buffer_state[name] += 1
    # Kapazität bestimmen
    shape = name.split("_")[-2]
    # Speichern
    save_buffer_state(buffer_state)
    
    max_cap = 4 if shape in ("Hex", "Cyl") else 3
    
    # Speichern
    save_buffer_state(buffer_state)
    print("# -- Open Gripper -- ")
    gripper_open(gripper)

    # At assembly Postion
    print("# -- Approach pos 1 -- ")
    MoveJ(f"Assembly_{position}", punkte, arm, offset=[0.0, 0.0, 0.06])

    print("# -- pos 1 -- ")
    MoveL(f"Assembly_{position}", punkte, arm)

    print("# -- Close Gripper -- ")
    gripper_close(gripper)

    print(f"# -- Approach Assembly_{position} -- ")
    MoveL(f"Assembly_{position}", punkte, arm, offset=[0.0, 0.0, 0.06])

    if buffer_state[name] > max_cap:
        print(f"⚠ {name} voll → Discard")
        buffer_state[name] -= 1

        MoveJ("Discard", punkte, arm)
        gripper_open(gripper)
        return False



    # Approach Buffer
    print(f"# -- Approach Refill Buffer {name}-- ")
    MoveJ(f"Refill_{name}", punkte, arm, offset=[0.0, 0.0, 0.06])

    # At Buffer
    print(f"# -- Refill Buffer {name}-- ")
    MoveJ(f"Refill_{name}", punkte, arm)

    print("# -- Open Gripper -- ")
    gripper_open(gripper)
    return True

def Assemble(name, punkte, arm, gripper, position):
    # Buffer aus Datei laden
    buffer_state = load_buffer_state()
    # Update
    buffer_state[name] -= 1
    # Speichern
    save_buffer_state(buffer_state)

    
    k = 0.05
    if  "Plate" in name:
        offset = transform_offset_deg([0,0,0.1],0,-15,0)
        grip_width = 0.02

        #g = -1
        #offset = [g*k*np.sin(np.deg2rad(15)),0,k*np.cos(np.deg2rad(15))]
    else:
        offset = transform_offset_deg([0,0,0.1],15,0,0)
        grip_width = None
        #g = -1
        #offset = [0,g*k*np.sin(np.deg2rad(15)),k*np.cos(np.deg2rad(15))]

    print("# -- Go to home -- ")
    MoveJ(f"Home", punkte, arm)

    print("# -- Open Gripper -- ")
    gripper_open(gripper,grip_width)

    # At Buffer
    print("# -- Approach Buffer name -- ")
    MoveL(f"{name}", punkte, arm,offset=offset)
    
    print("# -- Go To Buffer name -- ")
    MoveL(f"{name}", punkte, arm)
    
    print("# -- Close Gripper -- ")
    gripper_close(gripper)
    
    print("# -- Approach Buffer name -- ")
    MoveL(f"{name}", punkte, arm,offset=offset)

    print("# -- Go to home -- ")
    MoveJ(f"Home", punkte, arm)
    
    # At assembly Postion
    print("# -- Approach pos  -- ")
    MoveJ(f"Assembly_{position}", punkte, arm, offset=[0.0, 0.0, 0.08])

    print("# -- Go to pos  -- ")
    MoveL(f"Assembly_{position}", punkte, arm, offset=[0.0, 0.0, 0.01] )

    print("# -- Open Gripper -- ")
    gripper_open(gripper,grip_width)

    print("# -- Approach pos -- ")
    MoveL(f"Assembly_{position}", punkte, arm, offset=[0.0, 0.0, 0.08])


def MoveL(name, punkte, arm, offset=None):
    set_mode_lin(arm)
    go_to_point(name, punkte, arm, offset)

def MoveJ(name, punkte, arm, offset=None):
    set_mode_ptp(arm)
    go_to_point(name, punkte, arm, offset)

def go_to_point(name, punkte, arm, offset=None):
    pose = punkte.get(name)
    if pose is None:
        print(f"⚠ Punkt '{name}' nicht gefunden.")
        return False

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = pose.position.x
    pose_goal.position.y = pose.position.y
    pose_goal.position.z = pose.position.z
    pose_goal.orientation = pose.orientation

    #  Optional Offset addieren
    if offset is not None:
        pose_goal.position.x += offset[0]
        pose_goal.position.y += offset[1]
        pose_goal.position.z += offset[2]

    normalize_quaternion(pose_goal)
    arm.set_start_state_to_current_state()
    arm.set_pose_target(pose_goal)
    success = arm.go(wait=True)

    arm.stop()
    arm.clear_pose_targets()
    
    if not success:
        raise MoveItCommanderException("Bewegung fehlgeschlagen / durch Stop abgebrochen")


    return success


def get_point(name):# rosrun topic_tools relay /franka_state_controller/joint_states /joint_states

    punkte = load_points()
    for n, pose in punkte:
        if n == name:
            return pose
    print(f"⚠ Punkt '{name}' nicht vorhanden.")
    return None


def gripper_open(gripper, width=None):
    if width is None:
        width = 0.04  
    set_width(gripper, width)
    gripper.go(wait=True)

    
def gripper_close(gripper):
    set_width(gripper, 0.001)
    gripper.go(wait=True)


def set_width(gripper, width):
    # Panda: max. 0.08m → jede Fingerhälfte 0.04m
    # Finger joint bewegt ~die halbe Öffnung
    target = width / 2.0
    joint_values = gripper.get_current_joint_values()
    gripper.set_joint_value_target("panda_finger_joint1",target)
    gripper.set_joint_value_target("panda_finger_joint2", target)
    gripper.go(wait=True)

def set_mode_ptp(arm):
    arm.set_planner_id("PTP")
    arm.set_max_velocity_scaling_factor(0.3)

def set_mode_lin(arm):
    arm.set_planner_id("LIN")

def use_pilz(arm):
    arm.set_planner_id("PTP")  # oder LIN, CIRC
    arm.set_planning_pipeline_id("pilz_industrial_motion_planner")

def use_ompl(arm):
    arm.set_planner_id("")
    arm.set_planning_pipeline_id("ompl")


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
    punkte = {}

    try:
        with open(SAVE_FILE, "r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue

                try:
                    data = json.loads(line)
                except json.JSONDecodeError as e:
                    print(f"⚠ Ungültige Zeile ignoriert: {line}")
                    continue

                if not all(k in data for k in ("name", "pos", "quat")):
                    print(f"⚠ Unvollständige Daten ignoriert: {data}")
                    continue

                pose = Pose()
                pose.position.x = data["pos"][0]
                pose.position.y = data["pos"][1]
                pose.position.z = data["pos"][2]
                pose.orientation.x = data["quat"][0]
                pose.orientation.y = data["quat"][1]
                pose.orientation.z = data["quat"][2]
                pose.orientation.w = data["quat"][3]

                punkte[data["name"]] = pose

        # Initilize the buffers
        buffer_state = {}

        for name in punkte.keys():
            if not name.startswith("Buffer_"):
                continue

            parts = name.split("_")

            # Format: Buffer_Shape_Color
            if len(parts) != 3:
                print(f"⚠ Ungültiger Buffer-Name: {name}")
                continue

            _, shape, color = parts

            if shape in ("Hex", "Cyl"):
                buffer_state[name] = 4
            elif shape == "Plate":
                buffer_state[name] = 3
            else:
                print(f"⚠ Unbekannte Shape {shape}: setze 1")
                buffer_state[name] = 0

        print("\n✅ Buffer initialisiert:")
        for k, v in buffer_state.items():
            print(f"{k}: {v}")

    except FileNotFoundError:
        print(f"⚠ Datei nicht gefunden: {SAVE_FILE}")

    return punkte

def initialize_buffer_state(punkte):
    buffer_state = {}

    for name in punkte.keys():
        if not name.startswith("Buffer_"):
            continue

        # Format: Buffer_Color_Shape
        parts = name.split("_")
        if len(parts) != 3:
            print(f"⚠ Ungültiger Buffer-Name: {name}")
            continue

        _, shape, color = parts

        if shape in ("Hex", "Cyl"):
            buffer_state[name] = 4
        elif shape == "Plate":
            buffer_state[name] = 3
        else:
            print(f"⚠ Unbekannte Shape {shape}: setze 1")
            buffer_state[name] = 1

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
    


def transform_offset_deg(local_offset, roll_deg, pitch_deg, yaw_deg):
    # Convert DEG → RAD
    roll  = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw   = np.deg2rad(yaw_deg)

    ox, oy, oz = local_offset

    # Rotation X
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])

    # Rotation Y
    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # Rotation Z
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Gesamtrotation
    R = Rz @ Ry @ Rx

    local_vec = np.array(local_offset)
    global_vec = R.dot(local_vec)

    return global_vec  # -> [dx, dy, dz] im globalen Koordinatensystem


if __name__ == "__main__":
    main()
