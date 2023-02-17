# /usr/bin/env python2
'''
Python implementation of a path analyzer. The path analyzer finds 
the closest point on the path to the ego vehicle's pose. This is highly
configurable to support various path following control algorithms.
'''
from __future__ import print_function

import argparse
from autoware_msgs.msg import Lane
from collections import deque
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
import json
import numpy as np
import rospy
import rviz_tools_py as rviz_tools
from tf.transformations import euler_from_quaternion, quaternion_from_euler

parser = argparse.ArgumentParser(
    prog='LateralControlROSNode',
    formatter_class=argparse.RawDescriptionHelpFormatter,
    description="Lateral Controller for Autoware AI",
    epilog='A python ROS node implementing a lateral controller for use with '\
            'Autoware AI.')
parser.add_argument('--config', type=str,
                    required=True,
                    help='The path to the configuration file.')

class NpEncoder(json.JSONEncoder):
    """Class used to enable JSON serialization of Numpy objects."""
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.floating):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, np.bool_):
            return bool(obj)
        return super(NpEncoder, self).default(obj)


class NoReferenceDataError(Exception):
    '''Raised when path analyzer does not have reference data.'''
    pass


class ROSPathAnalyzer:
    def __init__(self, max_n_poses=1000):
        rospy.init_node('path_analyzer')
        rospy.loginfo('Initialized path_analyzer.')
        rospy.on_shutdown(self.MyHook)

        self.config_path = args.config
        self.config = {}
        with open(self.config_path, 'r') as f:
            self.config = json.load(f)

        self.rate = rospy.Rate(self.config["rate_Hz"])

        self.lookahead = self.config["controller_config"]["lookahead_m"]

        self.path_poses = deque(maxlen=max_n_poses)
        self.prev_output_pose = None  # For local planner tolerance.

        self.ego_pose = None

        self.ego_velocity = np.zeros((3,1)) # [m/s]
        self.ego_angular_velocity = np.zeros((3,1)) # [rad/s]

        self.waypoint_marker = rviz_tools.RvizMarkers(
                '/map', 'closest_waypoint_marker')
        self.position_marker = rviz_tools.RvizMarkers(
                '/map', 'ego_position'
        )

        rospy.Subscriber("/final_waypoints", Lane, 
						 callback=self.LaneArrayCallback, queue_size=1)
        rospy.Subscriber("/current_pose", PoseStamped, 
						 callback=self.CurrentPoseCallback, queue_size=1)

        rospy.loginfo('Begin publishing reference poses.')
        while not rospy.is_shutdown():
            self.publisher()

    def MyHook(self):
        rospy.loginfo('Shutting down path_analyzer node.')
        return None

    def LaneArrayCallback(self, msg):
        '''Get the position and orientation of the reference path.'''
        for waypoint in msg.waypoints:
            # Maintain newer poses in the left of the deque.
            self.path_poses.appendleft(waypoint.pose)

    def CurrentPoseCallback(self, msg):
        '''Get the current position of the ego vehicle.'''
        self.ego_pose = msg.pose

    def publisher(self):
        pub_reference_pose = rospy.Publisher("/ctrl_reference_pose",
											 PoseStamped, queue_size=1)
        ctrl_reference_msg = PoseStamped()

        try:
            ctrl_reference_position, \
            ctrl_reference_orientation = self.get_closest_reference()
        except NoReferenceDataError:
            return

        ctrl_reference_msg.header.stamp = rospy.get_rostime()
        ctrl_reference_msg.pose.position = ctrl_reference_position
        ctrl_reference_msg.pose.orientation = ctrl_reference_orientation
        pub_reference_pose.publish(ctrl_reference_msg)

        # Publish a sphere to mark where the reference message is.
        scale = Vector3(1.2,1.2,1.2) # diameter
        blue = (0,0,1) # tuple of RGB values (blue)
        red = (1,0,0)  # tuple of RGB values (red)
        self.waypoint_marker.publishSphere(ctrl_reference_msg.pose, 
                                           blue, scale, 0.06)
        ego_pose = Pose()
        ego_pose = self.ego_pose
        self.position_marker.publishArrow(ego_pose, red, scale, 0.06)

    def get_closest_reference(self):
        '''Returns data from the closest point to the ego position.'''
        # Store path and ego poses temporarily so they are not overwritten
        #  mid-computation.
        path_poses = list(self.path_poses)
        ego_pose = self.ego_pose
        # If a path exists send closest point, else raise error.
        if len(path_poses) > 0 and self.ego_pose is not None:
            # Assume ego position is on rear axle, and planar motion. Project 
            # to front axle.
            _, _, ego_yaw = euler_from_quaternion(
                self.get_orientation_from_pose(ego_pose).flatten())
            projection = np.array([
                [self.lookahead*np.cos(ego_yaw)],
                [self.lookahead*np.sin(ego_yaw)],
                [0]])
            front_axle_position = \
                self.get_position_from_pose(ego_pose) + projection

            # Find closest point in path.
            ego_to_path = [self.get_position_from_pose(path_pose.pose)\
                - front_axle_position for path_pose in path_poses]
            distances = np.linalg.norm(ego_to_path, axis=1)
            i_min_distance = np.argmin(distances)
            self.prev_output_pose = path_poses[i_min_distance]

            return path_poses[i_min_distance].pose.position, \
                   path_poses[i_min_distance].pose.orientation
        else:
            if len(path_poses) == 0:
                rospy.loginfo_throttle(1, "Waiting for reference data.")
            if self.ego_pose is None:
                rospy.loginfo_throttle(1, "Waiting for ego pose data.")
            raise NoReferenceDataError

    def get_position_from_pose(self, pose):
        return np.array([
            [pose.position.x],
            [pose.position.y],
            [pose.position.z]])
    
    def get_orientation_from_pose(self, pose):
        return np.array([
            [pose.orientation.x],
            [pose.orientation.y],
            [pose.orientation.z],
            [pose.orientation.w]])

    def limit_angle(self, angle):
        '''Limit |angle| to (-pi,pi).'''
        return (angle+np.pi) % (2*np.pi) - np.pi
    
if __name__ == '__main__':
    args = parser.parse_args()
    try:
        ROSPathAnalyzer()
    except rospy.ROSInterruptException:
        pass
