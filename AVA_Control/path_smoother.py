"""A simple cubic spline local planner."""

import argparse
from autoware_msgs.msg import LaneArray
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
        self.converge_distance = 10 # [m]

        self.ego_pose = None
        self.global_plan = None

        # Setup subscribers.
        rospy.Subscriber("/lane_waypoints_array", LaneArray, 
                                callback=self.GlobalPathCallback, queue_size=1)
        rospy.Subscriber("/current_pose", PoseStamped, 
						 callback=self.CurrentPoseCallback, queue_size=1)

        self.markers = rviz_tools.RvizMarkers('/map', 'local_plan')

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
        # TODO: Fix bug where points are allowed behind the vehicle.
        feasible_waypoints = []
        for waypoint in desired_lane.waypoints:
            waypoint_pose_np = self.pose_to_np(waypoint.pose.pose)
            difference = waypoint_pose_np - ego_pose_np
            distance_to_ego = np.linalg.norm(difference[0:2], axis=0)
            if distance_to_ego > self.converge_distance:
                feasible_waypoints.append(waypoint_pose_np)

        # Rotate waypoints and ego position to a coordinate axis aligned
        # with the vector from the ego position and the closest feasible
        # waypoint to enforce CubicSpline's need for increasing x.
        # TODO: Fix bug where the local path is separated from the ego vehicle
        # and the offset appears to be a function of the initial position.
        theta = np.arctan2(feasible_waypoints[0][1,0] - ego_pose_np[1,0],
                           feasible_waypoints[0][0,0] - ego_pose_np[0,0])
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        R_inv = np.linalg.inv(R)

        front_axle = np.array([
            [ego_pose_np[0,0] + 2.9*np.cos(ego_pose_np[2,0])],
            [ego_pose_np[1,0] + 2.9*np.sin(ego_pose_np[2,0])],
            [0]
        ])
        feasible_waypoints.insert(0, front_axle)
        feasible_waypoints.insert(0, ego_pose_np)

        rotated_feasible_points = [R_inv.dot(point[0:2]) for \
                                   point in feasible_waypoints]
        rotated_feasible_points = np.array(rotated_feasible_points)
        points = np.array(feasible_waypoints)

        # Fit spline from ego position to closest waypoint.
        local_plan = CubicSpline(rotated_feasible_points[:,0,0],
                                 rotated_feasible_points[:,1,0],
                                 bc_type=((2, 0),
                                          (2, 0)))
        local_plan_x = np.linspace(points[0,0],points[0,0] + 50)
        local_plan_y = local_plan(local_plan_x)
        local_plan_positions = np.vstack((local_plan_x, local_plan_y))

        # Rotate back to global coordinate system.
        local_plan_global = [R.dot(point) for point in local_plan_positions.T]

        # Create RVIZ path for visualization.
        path = []
        for position in local_plan_global:
            path.append(Point(position[0],position[1],0))
        width = 0.02
        self.markers.publishPath(path, 'orange', width, 0.1)

    @staticmethod
    def pose_to_np(pose):
        """Converts pose to 3x1 numpy array."""
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Wrap yaw angle around (-pi, pi).
        if yaw > np.pi:
            yaw -= 2*np.pi
        if yaw < -np.pi:
            yaw += 2*np.pi
        return np.array([[x],[y],[yaw]])
    
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

    # ctr_points = np.array(ctr_points)
    # cs = CubicSpline(ctr_points[:,0],ctr_points[:,1],
    #                  bc_type=((1, -65/180*np.pi), (1, -40/180*np.pi)))
    # eval_points_x = np.linspace(min(ctr_points[:,0]),max(ctr_points[:,0]))
    # eval_points_y = cs(eval_points_x)
    # plt.plot(eval_points_x, eval_points_y)
    # plt.scatter(ctr_points[:,0],ctr_points[:,1], c='black')
    # plt.show()