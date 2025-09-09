# bag_tum placeholder
import rosbag
import numpy as np

def save_to_tum(filename, data):
    with open(filename, 'w') as f:
        for timestamp, pos, orient in data:
            f.write(f"{timestamp} {pos[0]} {pos[1]} {pos[2]} {orient[0]} {orient[1]} {orient[2]} {orient[3]}\n")

def main():
    # Replace with your actual ROS bag file
    bag = rosbag.Bag('./2025-04-25-11-20-17.bag')

    odom_data = []
    pose_data = []

    for topic, msg, t in bag.read_messages(topics=['/Odometry', '/vrpn_client_node/unitree_b1/pose']):
        timestamp = t.to_sec()
        if topic == '/Odometry':
            pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
            orient = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            odom_data.append((timestamp, pos, orient))

        elif topic == '/vrpn_client_node/unitree_b1/pose':
            pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            orient = [msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w]
            pose_data.append((timestamp, pos, orient))

    bag.close()

    # Save to TUM format files
    save_to_tum("odom_data.tum", odom_data)
    save_to_tum("pose_data.tum", pose_data)

if __name__ == '__main__':
    main()
