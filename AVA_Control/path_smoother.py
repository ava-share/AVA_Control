"""A simple cubic spline local planner."""

import argparse
from autoware_msgs.msg import LaneArray, Lane, Waypoint
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
import matplotlib.pyplot as plt 
import numpy as np
import rospy
from scipy.interpolate import CubicSpline 
import rviz_tools_py as rviz_tools
from tf.transformations import euler_from_quaternion, quaternion_from_euler           

parser = argparse.ArgumentParser(
    prog='LocalPlannerROSNode',
    formatter_class=argparse.RawDescriptionHelpFormatter,
    description="A simple cubic spline local planner for Autoware AI",
    epilog='A python ROS node implementing a cubic spline local planner for '\
           'Autoware AI. It takes in the global plan, fits a cubic spline to '\
           'the plan, and satisfies start and end heading conditions.')
# parser.add_argument('--config', type=str,
#                     required=True,
#                     help='The path to the configuration file.')


class ROSLocalPlanner:
    """An implementation of a local planner for ROS."""
    def __init__(self):
        rospy.init_node('local_planner')
        rospy.loginfo('Initialized local_planner.')
        rospy.on_shutdown(self.MyHook)
        self.rate = rospy.Rate(10)
        self.converge_distance = 20 # [m]
        self.plan_distance = 25  # [m]
        self.wheelbase = 2.9 # [m]

        self.ego_pose = None
        self.global_plan = None

        # Setup subscribers.
        rospy.Subscriber("/lane_waypoints_array", LaneArray, 
                                callback=self.GlobalPathCallback, queue_size=1)
        rospy.Subscriber("/current_pose", PoseStamped, 
						 callback=self.CurrentPoseCallback, queue_size=1)

        self.path_markers = rviz_tools.RvizMarkers('/map', 'local_plan')
        self.path_direction_markers = rviz_tools.RvizMarkers('/map', 'local_direction')

        rospy.loginfo('Begin publishing reference poses.')
        while not rospy.is_shutdown():
            self.publisher()

    def MyHook(self):
        rospy.loginfo('Shutting down local_planner node...')
        rospy.loginfo('local_planner shutdown complete.')

    def GlobalPathCallback(self, msg):
        '''Get the position and orientation of the global path.'''
        self.global_plan = msg

    def CurrentPoseCallback(self, msg):
        '''Get the current position of the ego vehicle.'''
        self.ego_pose = msg.pose

    def publisher(self):
        '''Get the cubic spline that best fits to the data and interpolate.'''
        # Locally store data so overwriting doesn't happen during computation.
        global_plan = self.global_plan
        ego_pose = self.ego_pose
        
        if global_plan is None:
            rospy.loginfo_throttle(1, "Waiting for a global plan.")

        if ego_pose is None:
            rospy.loginfo_throttle(1, "Waiting for an ego pose.")

        if global_plan is None or ego_pose is None:
            return

        # Assume the first lane is the desired lane.
        desired_lane = global_plan.lanes[0]

        # Unpack ego pose.
        ego_pose_np = self.pose_to_np(ego_pose)

        # Store waypoints that are far from the ego position.
        feasible_waypoints = self.get_feasible_waypoints(desired_lane, 
                                                         ego_pose_np)

        # Fit a spline from ego pose to the feasible waypoints.
        local_plan, yaw_ref = self.fit_spline(ego_pose_np, feasible_waypoints)
        path_points, path_quats, path_poses = self.convert_to_ros(local_plan, 
                                                                  yaw_ref)

        # Create RVIZ path for visualization.
        self.publish_rviz_path(path_points, path_poses)

        # Publish to /final_waypoints
        pub_path = rospy.Publisher("/final_waypoints", Lane, queue_size=1)
        local_path = Lane()
        # Convert poses to waypoints.
        waypoints = []
        for pose in path_poses:
            w = Waypoint()
            w.pose.pose = pose
            waypoints.append(w)
        local_path.header.stamp = rospy.get_rostime()
        local_path.waypoints = waypoints

        pub_path.publish(local_path)

    def fit_spline(self, ego_pose, waypoints):
        '''Creates a cubic spline connecting the ego_pose to waypoints.
        
        Rotates the ego_pose, front axle, and waypoints to a coordinate
        axis parallel to the vector from the ego position and the closest
        waypoint to enforce CubicSpline's need for increasing x.
        '''
        # Compute rotation matrices.
        theta = np.arctan2(waypoints[0,1] - ego_pose[1],
                           waypoints[0,0] - ego_pose[0])
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        R_inv = np.linalg.inv(R)

        # Compute front axle position.
        front_axle = np.array([
            [ego_pose[0] + self.wheelbase*np.cos(ego_pose[2])],
            [ego_pose[1] + self.wheelbase*np.sin(ego_pose[2])],
            [0]
        ])

        # Insert front axle and ego position to enforce heading constraint.
        waypoints = np.vstack((ego_pose.reshape((1,3)), 
                               front_axle.reshape((1,3)), 
                               waypoints))
        
        # Rotate waypoints.
        rotated_waypoints = [R_inv.dot(p[0:2]) for p in waypoints]
        rotated_waypoints = np.array(rotated_waypoints)

        # Fit spline.
        spline = CubicSpline(rotated_waypoints[:,0],
                             rotated_waypoints[:,1],
                             bc_type='natural')
        plan_x = np.linspace(rotated_waypoints[0,0],
                             rotated_waypoints[0,0] + self.plan_distance)
        plan_y = spline(plan_x)
        plan_dy = spline(plan_x, 1)
        plan = np.vstack((plan_x, plan_y))
        plan_direction = np.vstack((np.ones(plan_dy.shape), plan_dy))

        # Rotate back to global coordinate system.
        plan_unrotated = [R.dot(p) for p in plan.T]
        plan_direction_unrotated = [R.dot(p) for p in plan_direction.T]
        plan_yaw = [np.arctan2(p[1],p[0]) for p in plan_direction_unrotated]

        return (plan_unrotated, plan_yaw)

    def convert_to_ros(self, positions, yaws):
        '''Converts positions to Points and yaws to Quaternions.'''
        path = [Point(p[0],p[1],0) for p in positions]
        orientation = [quaternion_from_euler(0, 0, yaw) for yaw in yaws]
        direction = [Quaternion(q[0], q[1], q[2], q[3]) for q in orientation]
        poses = [Pose(p,q) for p,q in zip(path, direction)]
        return path, direction, poses

    def publish_rviz_path(self, path, poses):
        '''Publishes markers for RVIZ visualization of the local path.'''
        width = 0.1
        self.path_markers.publishPath(path, 'orange', width, 0.1)
        arrow_length = 2.0
        self.path_direction_markers.publishArrow(poses[1], 'pink', arrow_length, 0.1)

    def get_feasible_waypoints(self, lane, ego_pose):
        '''Returns the waypoints in front of the ego vehicle without the
        nearest waypoints.'''
        waypoints = []
        for waypoint in lane.waypoints:
            waypoint_pose_np = self.pose_to_np(waypoint.pose.pose)
            waypoints.append(waypoint_pose_np)
        waypoints = np.array(waypoints)
        # Find closest waypoint.
        ego_to_waypoints = waypoints - ego_pose
        distances = np.linalg.norm(ego_to_waypoints[:,:2], axis=1)
        closest_i = np.argmin(distances)
        # Assume global path is in increasing order, filter out points behind
        # closest waypoint.
        # Remove waypoints past the plan distance.
        distant_i = np.argmin(abs(distances[closest_i:] - self.plan_distance))
        distant_i += closest_i
        # Remove waypoints within the convergence distance.
        convergence_i = np.argmin(abs(distances[closest_i:] - \
                                      self.converge_distance))
        convergence_i += closest_i
        return waypoints[convergence_i:distant_i+1]

    @staticmethod
    def angle(vector_1, vector_2):
        '''Computes the angle from vector_1 to vector_2.'''
        angle_2 = np.arctan2(vector_2[1,0], vector_2[0,0])
        angle_1 = np.arctan2(vector_1[1,0], vector_1[0,0])
        return angle_2 - angle_1

    @staticmethod
    def pose_to_np(pose):
        '''Converts pose to 3x1 numpy array.'''
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Wrap yaw angle around (-pi, pi).
        if yaw > np.pi:
            yaw -= 2*np.pi
        if yaw < -np.pi:
            yaw += 2*np.pi
        return np.array([x, y, yaw])
    
    @classmethod
    def waypoints_to_np(cls, waypoints):
        '''Converts a list of waypoints to an Nx3x1 numpy array.'''
        n = len(waypoints)
        waypoints_np = np.zeros((n,3,1))
        for i, waypoint in enumerate(waypoints):
            waypoints_np[i,:,:] = cls.pose_to_np(waypoint.pose)
        return waypoints_np

if __name__ == '__main__':
    args = parser.parse_args()
    try:
        ROSLocalPlanner()
    except rospy.ROSInterruptException:
        pass