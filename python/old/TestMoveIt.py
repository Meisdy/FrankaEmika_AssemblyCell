#!/usr/bin/env python3
import rospy
import moveit_commander
import geometry_msgs.msg
from visualization_msgs.msg import Marker

def publish_marker(position):
    marker_pub = rospy.Publisher("/target_marker", Marker, queue_size=10, latch=True)
    marker = Marker()
    marker.header.frame_id = "panda_link0"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.position.x = position.x
    marker.pose.position.y = position.y
    marker.pose.position.z = position.z
    marker_pub.publish(marker)
    rospy.sleep(0.2)

def main():
    rospy.init_node("visualize_and_move")
    moveit_commander.roscpp_initialize([])
    arm = moveit_commander.MoveGroupCommander("panda_arm")

    # Zielposition definieren
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.5326153829889275
    target_pose.position.y = 0.008447671899554752
    target_pose.position.z = 0.5258883500946044
    target_pose.orientation.w = 0.02060697999490455  # gerade Haltung
    print(" Zielpunkt wird zur Visualisierung gesendet...")
    publish_marker(target_pose.position)
    arm.set_pose_target(target_pose)

    input(" Drücke ENTER um Bewegung auszuführen...")

    print(" Bewegung startet...")
    arm.go(wait=True)
    arm.stop()
    arm.clear_pose_targets()
    print("Bewegung abgeschlossen!")

if __name__ == "__main__":
    main()
