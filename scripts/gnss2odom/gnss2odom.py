# gnss2odom placeholder
#!/usr/bin/env python3

import rospy
import numpy as np
import pymap3d as pm
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class GnssToPoseNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('gnss_to_pose_converter', anonymous=True)

        # Get parameters
        self.reference_set = False
        self.ref_lat = rospy.get_param('~ref_lat', None)
        self.ref_lon = rospy.get_param('~ref_lon', None)
        self.ref_alt = rospy.get_param('~ref_alt', None)

        # Set fixed reference point if provided
        if self.ref_lat is not None and self.ref_lon is not None and self.ref_alt is not None:
            self.reference_set = True
            rospy.loginfo(f"Using fixed reference point: lat={self.ref_lat}, lon={self.ref_lon}, alt={self.ref_alt}")

        # Subscribe to GNSS data in PoseStamped format
        self.gnss_sub = rospy.Subscriber('/gnss_pose', PoseStamped, self.gnss_callback)

        # Publisher for odometry
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        rospy.loginfo("GNSS to Pose converter node initialized")

    def gnss_callback(self, msg):
        """Callback for PoseStamped messages"""
        # In this PoseStamped format, we assume:
        # position.x = latitude
        # position.y = longitude
        # position.z = altitude
        # orientation = current orientation in quaternion

        lat = msg.pose.position.x
        lon = msg.pose.position.y
        alt = msg.pose.position.z

        # Extract orientation from PoseStamped
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # Set reference point if not already set
        if not self.reference_set:
            self.ref_lat = lat
            self.ref_lon = lon
            self.ref_alt = alt
            self.reference_set = True
            rospy.loginfo(f"Setting reference point to first GNSS fix: lat={lat}, lon={lon}, alt={alt}")

        # Convert to ENU coordinates
        e, n, u = pm.geodetic2enu(lat, lon, alt, self.ref_lat, self.ref_lon, self.ref_alt)

        # Create and publish odometry message
        self.publish_odometry(e, n, u, qx, qy, qz, qw, msg.header.stamp)

    def publish_odometry(self, x, y, z, qx, qy, qz, qw, timestamp):
        """Publish Odometry message"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp if timestamp != rospy.Time(0) else rospy.Time.now()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"

        # Set position and orientation
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Set covariance (default values)
        odom_msg.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                                   0, 0.01, 0, 0, 0, 0,
                                   0, 0, 0.01, 0, 0, 0,
                                   0, 0, 0, 0.01, 0, 0,
                                   0, 0, 0, 0, 0.01, 0,
                                   0, 0, 0, 0, 0, 0.01]

        # Set twist (velocity) to zero since we don't have velocity information
        odom_msg.twist.covariance = [0.01, 0, 0, 0, 0, 0,
                                    0, 0.01, 0, 0, 0, 0,
                                    0, 0, 0.01, 0, 0, 0,
                                    0, 0, 0, 0.01, 0, 0,
                                    0, 0, 0, 0, 0.01, 0,
                                    0, 0, 0, 0, 0, 0.01]

        # Publish Odometry
        self.odom_pub.publish(odom_msg)

if __name__ == '__main__':
    try:
        node = GnssToPoseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
