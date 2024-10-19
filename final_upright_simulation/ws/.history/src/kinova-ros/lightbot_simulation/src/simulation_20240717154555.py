#!/usr/bin/env python

import rospy
import moveit_commander
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler, quaternion_from_matrix
import tf2_ros
import math
import matplotlib.pyplot as plt
import sys
import tf2_geometry_msgs

# Global variables for publishers and broadcasters
joint_pub = None
tf_broadcaster = None

def load_spherical_coordinates(filename):
    points = []
    with open(filename, 'r') as f:
        for line in f:
            r, theta, phi = map(float, line.strip().split())
            points.append((r, theta, phi))
    return points

def spherical_to_cartesian(r, theta, phi):
    x = r * math.sin(phi) * math.cos(theta)
    y = r * math.sin(phi) * math.sin(theta)
    z = r * math.cos(phi)
    return x, y, z


def create_pose(x, y, z, target_frame):
    pose = PoseStamped()
    pose.header.frame_id = target_frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    
    # Calculate orientation to point Z-axis towards the origin of surface_link
    direction = np.array([-x, -y, -z])
    direction /= np.linalg.norm(direction)
    
    up = np.array([0, 0, 1])
    right = np.cross(up, direction)
    right /= np.linalg.norm(right)
    up = np.cross(direction, right)
    
    # Create a 4x4 transformation matrix
    rotation_matrix = np.column_stack((right, up, direction))
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = [x, y, z]
    
    q = quaternion_from_matrix(transform_matrix)
    
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    
    return pose

def update_camera_position(camera_x, camera_y, camera_z, method='joint_state'):
    global joint_pub, tf_broadcaster
    
    if method == 'joint_state':
        # Method 1: Update using joint_states
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['camera_joint']  # Replace with your actual joint name
        joint_state.position = [camera_y]  # Assuming the joint controls Y position
        joint_pub.publish(joint_state)
        
        # If you need to update X and Z as well, you might need additional joints in your URDF
        
    elif method == 'tf':
        # Method 2: Update using tf2 broadcaster
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"  # Replace with the parent frame of your camera
        t.child_frame_id = "camera_link"
        t.transform.translation.x = camera_x
        t.transform.translation.y = camera_y
        t.transform.translation.z = camera_z
        t.transform.rotation.w = 1.0
        tf_broadcaster.sendTransform(t)

def count_valid_poses(camera_x, camera_y, camera_z, goal_poses, move_group, tf_buffer):
    valid_count = 0
    
    # Update camera position
    update_camera_position(camera_x, camera_y, camera_z, method='joint_state')  # or 'tf'
    
    rospy.sleep(0.1)  # Give some time for the update to propagate
    
    for r, theta, phi in goal_poses:
        x, y, z = spherical_to_cartesian(r, theta, phi)
        pose = create_pose(x, y, z, "surface_link")
        
        try:
            # Transform the pose to the planning frame
            pose_transformed = tf_buffer.transform(pose, move_group.get_planning_frame(), rospy.Duration(1.0))
            
            # Check if the pose is reachable
            if move_group.compute_ik(pose_transformed.pose):
                # Check for collisions
                move_group.set_pose_target(pose_transformed.pose)
                plan = move_group.plan()
                if plan[0]:  # If a valid plan is found
                    valid_count += 1
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
        
    return valid_count

def optimize_camera_position(goal_poses, x_pos, y_range, z_range, y_step, z_step):
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    best_y, best_z, best_count = 0, 0, 0
    
    y_positions = np.arange(y_range[0], y_range[1], y_step)
    z_positions = np.arange(z_range[0], z_range[1], z_step)
    
    results = np.zeros((len(z_positions), len(y_positions)))
    
    total_poses = len(goal_poses)
    
    for i, camera_z in enumerate(z_positions):
        for j, camera_y in enumerate(y_positions):
            count = count_valid_poses(x_pos, camera_y, camera_z, goal_poses, move_group, tf_buffer)
            results[i, j] = (count / total_poses) * 100  # Calculate percentage
            if count > best_count:
                best_y, best_z, best_count = camera_y, camera_z, count
    
    return x_pos, best_y, best_z, best_count, results, y_positions, z_positions

def plot_results(results, y_positions, z_positions, optimal_y, optimal_z):
    plt.figure(figsize=(10, 8))
    plt.imshow(results, cmap='viridis', origin='lower', aspect='auto', 
               extent=[y_positions[0], y_positions[-1], z_positions[0], z_positions[-1]])
    plt.colorbar(label='Percentage of Reachable Poses')
    plt.title('Percentage of Reachable Goal Poses')
    plt.xlabel('Camera Y Position')
    plt.ylabel('Camera Z Position')
    
    # Mark the optimal position
    plt.plot(optimal_y, optimal_z, 'r*', markersize=15, label='Optimal Position')
    plt.legend()
    
    plt.tight_layout()
    plt.savefig('camera_position_heatmap.png')
    plt.show()

if __name__ == '__main__':
    try:
        rospy.init_node('camera_position_optimizer', anonymous=True)
        
        # Set up publishers and broadcasters
        global joint_pub, tf_broadcaster
        joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Load goal poses from file
        goal_poses = load_spherical_coordinates('goal_poses.lp')
        
        # Set the range and step size for the camera position
        x_pos = 0.0  # Fixed X position
        y_range = (-0.5, 0)  # Y range
        z_range = (0.15, 0.75)   # Z range
        y_step = 0.05
        z_step = 0.05
        
        optimal_x, optimal_y, optimal_z, max_valid_poses, results, y_positions, z_positions = optimize_camera_position(
            goal_poses, x_pos, y_range, z_range, y_step, z_step)
        
        print("Optimal camera position: X=", optimal_x, ", Y=", optimal_y, ", Z=", optimal_z)
        print("Maximum number of valid poses: ", max_valid_poses)
        
        # print(f"Optimal camera position: X={optimal_x}, Y={optimal_y}, Z={optimal_z}")
        # print(f"Maximum number of valid poses: {max_valid_poses}")
        
        # Plot the results
        plot_results(results, y_positions, z_positions, optimal_y, optimal_z)
        
    except rospy.ROSInterruptException:
        pass