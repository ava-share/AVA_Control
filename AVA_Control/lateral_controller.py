#!/usr/bin/env python
"""Python implementation of Stanley controller using kinematic model for
Autoware AI. Uses Autoware AI messages version 1.14.0.
"""
import argparse
from autoware_msgs.msg import ControlCommandStamped
from collections import deque
from geometry_msgs.msg import TwistStamped, PoseStamped
import json
import numpy as np
import os
import rospy
from tf.transformations import euler_from_quaternion

parser = argparse.ArgumentParser(
    prog='LateralControlROSNode',
    formatter_class=argparse.RawDescriptionHelpFormatter,
    description="Lateral Controller for Autoware AI",
    epilog='A python ROS node implementing a lateral controller for use with '\
            'Autoware AI.')
parser.add_argument('--config', type=str,
                    required=True,
                    help='The path to the configuration file.')

def uniquify(path):
    """Create a unique filename if file already exists."""
    filename, extension = os.path.splitext(path)
    counter = 0
    while os.path.exists(path):
        path = filename + "_" + str(counter) + extension
        counter += 1

    return path


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


class AbstractLateralController:
    """An absctract lateral controller class intended to define the minimum
    methods and attributes to be implemented as a rosnode."""
    
    def compute_error(self, ego_x, ego_y, ego_yaw, ref_x, ref_y, ref_yaw, 
                      lookahead):
        """Computes the crosstrack error and yaw error.

        Args:
            ego_x (float): current x location of the ego vehicle [m].
            ego_y (float): current y location of the ego vehicle [m].
            ego_yaw (float): yaw heading of the ego vehicle [rad].
            ref_x (float): x locations of the path [m].
            ref_y (float): y locations of the path [m].
            ref_yaw (float): yaw heading of the reference path [rad].
            lookahead (float): point in front of rear axle to use to compute
                the reference error.
        Returns:
            float: the crosstrack error [m].
        """

        # Calculate position of the front axle
        front_axle_x = ego_x + lookahead * np.cos(ego_yaw)
        front_axle_y = ego_y + lookahead * np.sin(ego_yaw)

        ref_to_axle = np.array([front_axle_x - ref_x, front_axle_y - ref_y])
        crosstrack_vector = np.array([np.sin(ref_yaw), -np.cos(ref_yaw)])
        crosstrack_err = ref_to_axle.dot(crosstrack_vector)
        if self.config["enable_logging"]:
            self.recorder.crosstrack_error.append(crosstrack_err)
        
        return crosstrack_err

    def get_steer_angle(self):
        """Computes the steer angle according to the lateral control law."""
        raise NotImplementedError
    

class StanleyController(AbstractLateralController):
    def __init__(self, recorder, config, control_gain, softening_gain, 
                 max_steer, wheelbase, enable_LPF, LPF_bandwidth):
        
        """
        TUNING PARAMETERS   Tune control_gain first. Probably won't need to 
                            change softening_gain.
            :param control_gain:                (float) time constant [1/s]
            :param softening_gain:              (float) softening gain [m/s]
            :param max_steer:                   (float) vehicle's steering 
                                                limits [rad]
        VEHICLE PARAMETERS
            :param wheelbase:                   (float) vehicle's wheelbase [m]
            :param path_x:                      (numpy.ndarray) list of x-coordinates along the path
            :param path_y:                      (numpy.ndarray) list of y-coordinates along the path
            :param path_yaw:                    (numpy.ndarray) list of discrete yaw values along the path
            :param dt:                          (float) discrete time period [s]
        At every time step
            :param x:                           (float) vehicle's x-coordinate [m]
            :param y:                           (float) vehicle's y-coordinate [m]
            :param yaw:                         (float) vehicle's heading [rad]
            :param target_velocity:             (float) vehicle's velocity [m/s]
            :param steering_angle:              (float) vehicle's steering angle [rad]
        :return limited_steering_angle:     (float) steering angle after imposing steering limits [rad]
        :return target_index:               (int) closest path index
        :return crosstrack_error:           (float) distance from closest path index [m]
        """
        self.recorder = recorder
        self.config = config
        
        self.k = control_gain
        self.k_soft = softening_gain
        self.max_steer = max_steer
        self.L = wheelbase

        self.prev_steer_angle = 0  # [rad]
        
        # TUNING PARAMETERS (change if needed)
        self.enable_LPF = enable_LPF
        self.LPF_bandwidth = LPF_bandwidth  # [Hz]
             
    
    def get_steer_angle(self, x, y, yaw, current_velocity, px, py, pyaw):
        """
        :param x:
        :param y:
        :param yaw:
        :param current_velocity:
        :return: steering output, target index, crosstrack error
        """
        crosstrack_error = self.compute_error(x, y, yaw, px, py, pyaw, self.L)
        if self.config["enable_debugging"]:
            print('Crosstrack_Error: '+str(crosstrack_error))

        # Stanley Control law.
        yaw_term = pyaw - yaw
        tangent_term = np.arctan((self.k*crosstrack_error/ \
                                  (current_velocity + self.k_soft)))
        steer_angle = yaw_term + tangent_term
        if self.config["enable_debugging"]:
            print('Psi: '+str(yaw_term)+'\tTangent Term: '+str(tangent_term))
            print('Stanley Output: '+str(steer_angle))

        # Constrains steering angle to the vehicle limits.
        steer_angle = np.clip(steer_angle, -self.max_steer, self.max_steer)

        # Low pass filter (linearly interpolate between previous and current
        # angle).
        if self.enable_LPF:
            Ts = 1./100. # sampling rate of steering angle signal
            Tc = 1./self.LPF_bandwidth
            steer_angle = (1.0 - Ts/Tc) * self.prev_steer_angle + \
                          Ts/Tc * steer_angle
            
        self.prev_steer_angle = steer_angle

        return steer_angle


class DiscreteFilter:
    """An FIR filter implemented as a difference equation. This is to remove
    dependency on scipy.signal.lfilter.
    
    The discrete time transfer function is:
                                -1              -M
                b[0] + b[1]z  + ... + b[M] z
        Y(z) = -------------------------------- X(z)
                            -1              -N
                a[0] + a[1]z  + ... + a[N] z
    """
    def __init__(self, b, a):
        self.b = b  # the numerator coefficients
        self.a = a  # the denominator coefficients
        self.past_inputs = deque([0] * len(b), maxlen=len(b))
        self.past_outputs = deque([0] * (len(a) - 1), maxlen=len(a)-1)

    def get_output(self, input):
        """Computes the output of the filter.
        
        Computes the next output, y[n] given y[n-1:n-N] and x[n:n-M] where 
        n is the sample number and N is the filter order:
            a[0]*y[n] = b[0]*x[n] + b[1]*x[n-1] + ... + b[M]*x[n-M]
                        - a[1]*y[n-1] - ... - a[N]*y[n-N]
        """
        self.past_inputs.appendleft(input)
        new_output = np.dot(self.b, self.past_inputs) - \
                     np.dot(self.a[1:], self.past_outputs)
        new_output /= self.a[0]
        self.past_outputs.appendleft(new_output)
        return new_output


class SISOLookaheadController(AbstractLateralController):
    """A frequency-based controller designed using the youla parameterization
    technique. Designed in the continuous-time domain and converted to a
    discrete FIR filter."""

    def __init__(self, numerator, denominator, lookahead, config):
        self.config = config
        self.control_filter = DiscreteFilter(numerator, 
                                             denominator)
        self.lookahead = lookahead

    def get_steer_angle(self, x, y, yaw, ref_x, ref_y, ref_yaw):
        """Compute steer angle using a discrete FIR filter."""
        crosstrack_error = self.compute_error(x, y, yaw, ref_x, ref_y, \
                                              ref_yaw, self.lookahead)
        rospy.loginfo("Crosstrack error [m]: %s"%crosstrack_error)
        delta = self.control_filter.get_output(crosstrack_error)
        delta *= 2
        #delta = 0.01 * crosstrack_error
        rospy.loginfo("Steer Angle [deg]: %s"%(delta*180/np.pi))
        delta = np.clip(delta, -35 * np.pi / 180, 35 * np.pi / 180)
        return delta


class Data:
    """Class implementation of data logging for lateral controller."""

    def __init__(self, experiment_config):
        self.config = experiment_config
        self.steering = []
        self.vx = []
        self.px = []
        self.py = []
        self.pyaw = []
        self.x = []
        self.y = []
        self.yaw = []
        self.crosstrack_error = []
        self.auto = [] # autonomous or manual?
        self.t = [] # time

    def to_csv(self, file_path):
        """Publish all data to csv files and configuration to json file."""
        # Store data in csv file.
        with open(file_path, 'a') as f:
            test = "t [s], steering [rad], vx [m/s], px [m], py [m], "\
                "pyaw [rad], x [m], y [m], yaw [rad], crosstrack_error [m]\n"
            f.write(test)

            i = 0
            while i < len(self.steering):
                test = ','.join((
                    str(self.t[i]),             str(self.steering[i]),  
                    str(self.vx[i]),            str(self.px[i]),
                    str(self.py[i]),            str(self.crosstrack_error[i]),
                    str(self.pyaw[i]),          str(self.x[i]),
                    str(self.y[i]),             str(self.yaw[i]),
                    str(self.auto[i]))
                )
                i += 1
                f.write(test)
                f.write('\n')

    def save_data(self):
        """Stores data in csv file and experimental config to json file."""
        save_dir = os.path.join(os.getcwd(), "lateral_ctrl_results")
        experiment_name = "sim"
        data_path = os.path.join(save_dir, experiment_name + "_data.csv")
        data_path = uniquify(data_path)
        if os.path.isdir(save_dir):
            self.to_csv(data_path)
        else:
            os.mkdir(save_dir)
            self.to_csv(data_path)

        # Store configuration in json file.
        config_path = os.path.join(save_dir, experiment_name + "_config.json")
        config_path = uniquify(config_path)
        with open(config_path, "w") as f:
            f.write(json.dumps(self.config, sort_keys=True, indent=4, 
                               cls=NpEncoder))


class ROSLateralController:
    """Class implementation of a lateral controller packaged as a ROS node."""
    
    def __init__(self):
        rospy.init_node('lateral_control')
        rospy.loginfo("Initialized lateral_control node.")
        rospy.on_shutdown(self.my_hook)

        self.config_path = args.config
        self.config = {}
        with open(self.config_path, 'r') as f:
            self.config = json.load(f)
        self.recording = Data(self.config)

        self.rate = rospy.Rate(self.config['rate_Hz'])  # [Hz]

        self.ref_pose = None
        self.ego_pose = None
        self.ego_twist = None

        rospy.Subscriber("/ctrl_reference_pose", PoseStamped, 
                         callback=self.ReferencePoseCallback, queue_size=1)
        rospy.Subscriber("/current_pose"  , PoseStamped, 
                         callback = self.CurrentPoseCallback, queue_size=1)
        rospy.Subscriber("/current_velocity", TwistStamped,
                         callback = self.CurrentTwistCallback, queue_size=1)

        if self.config["controller_type"] == "stanley":
            ctrl_config = self.config["controller_config"]
            self.controller = StanleyController(
                recorder=self.recording,
                config=self.config,
                control_gain=ctrl_config["ctrl_gain"],
                softening_gain=ctrl_config["softening_gain"],
                max_steer=np.deg2rad(ctrl_config["max_steer_rad"]),
                wheelbase=ctrl_config["wheelbase_m"],
                enable_LPF=ctrl_config["enable_lowpass_filter"],
                LPF_bandwidth=ctrl_config["lowpass_filter_bandwidth_Hz"])
        elif self.config["controller_type"] == "SISOLookaheadYoula":
            ctrl_config = self.config["controller_config"]
            self.controller = SISOLookaheadController(
                config=self.config,
                numerator=ctrl_config["numerator"],
                denominator=ctrl_config["denominator"],
                lookahead=ctrl_config["lookahead_m"])
        else:
            raise NotImplementedError
        
        if self.config["enable_velocity_control"]:
            self.constant_vx = self.config["constant_velocity_mps"]
        else:
            self.constant_vx = None  # Converts to 0 unless overwritten.
        
        if self.config["enable_acceleration_control"]:
            self.constant_acc = self.config["constant_acceleration_mps2"]
        else:
            self.constant_acc = None  # Converts to 0 unless overwritten.
        
        rospy.loginfo("Begin publishing control commands.")
        while not rospy.is_shutdown():
            self.publisher()

    def my_hook(self):
        """Executed at ROS node shutdown."""
        msg_ctrl = ControlCommandStamped()
        msg_ctrl.header.stamp = rospy.get_rostime()
        msg_ctrl.cmd.steering_angle = 0
        msg_ctrl.cmd.linear_velocity = 0
        msg_ctrl.cmd.linear_acceleration = 0

        pub_pose = rospy.Publisher("/ctrl_raw", ControlCommandStamped, 
                                   queue_size=1)
        pub_pose.publish(msg_ctrl)

        # TODO: Setup automatic name changing for each run.
        if self.config["enable_logging"]:
            self.recording.save_data()

        rospy.loginfo("Shutting down lateral_control node.")
        
    def ReferencePoseCallback(self, msg):
        """Get the current reference pose."""
        self.ref_pose = msg.pose
            
    def CurrentPoseCallback(self, msg):
        """Get the current pose of the vehicle."""
        self.ego_pose = msg.pose

    def CurrentTwistCallback(self, msg):
        """Get the current linear and angular velocities of the vehicle."""
        self.ego_twist = msg.twist
 
    def publisher(self):
        """Compute and publish control command."""
        # Store ego/ref pose to prevent overwritting while computing.
        if self.ref_pose is not None:
            ref_x, ref_y, ref_yaw = self.unpack_pose(self.ref_pose)
        else:
            rospy.loginfo_throttle(1, "Waiting for reference pose.")
            return
        
        if self.ego_pose is not None:
            ego_x, ego_y, ego_yaw = self.unpack_pose(self.ego_pose)
        else:
            rospy.loginfo_throttle(1, "Waiting for ego pose.")
            return

        if self.ego_twist is not None:
            ego_vx = self.ego_twist.linear.x
        else:
            rospy.loginfo_throttle(1, "Waiting for ego twist.")
        
        # TODO: Come up with a better controller API so we don't need different
        # calls.
        if self.config["controller_type"] == "stanley":
            steer_angle = \
                self.controller.get_steer_angle(
                    ego_x, ego_y, ego_yaw, ego_vx, ref_x,
                    ref_y, ref_yaw)
        elif self.config["controller_type"] == "SISOLookaheadYoula":
            steer_angle = \
                self.controller.get_steer_angle(
                    ego_x, ego_y, ego_yaw, ref_x, ref_y, ref_yaw)
        else:
            steer_angle = 0
            rospy.logerr("No steering controller selected in config.")
        
        # Create control message to send.
        msg_ctrl = ControlCommandStamped()
        msg_ctrl.header.stamp = rospy.get_rostime()
        msg_ctrl.cmd.steering_angle = steer_angle
        msg_ctrl.cmd.linear_velocity = self.constant_vx
        msg_ctrl.cmd.linear_acceleration = self.constant_acc
        if self.config["enable_debugging"]:
            print('Steering: ' + str(steer_angle))
            print('---------------------------------------')

        # Store steering so it can be used by lateral control when filtering.
        self.recording.steering.append(steer_angle)
        if self.config["enable_logging"]:
            self.recording.t.append(rospy.get_rostime())
            self.recording.vx.append(self.ego_vx)
            self.recording.x.append(self.ego_x)
            self.recording.y.append(self.ego_y)
            self.recording.yaw.append(self.ego_yaw)
            
            self.recording.px.append(ref_x)
            self.recording.py.append(self.ref_y)
            self.recording.pyaw.append(self.ref_yaw)
            
            self.recording.auto.append(1)

        pub_pose = rospy.Publisher("/ctrl_raw", ControlCommandStamped, 
                                   queue_size=1)
        pub_pose.publish(msg_ctrl)

    @staticmethod
    def unpack_pose(pose):
        """Convert pose message to x, y, yaw."""
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return x, y, yaw


if __name__ == '__main__':
    args = parser.parse_args()
    try:
        ROSLateralController()
    except rospy.ROSInterruptException:
        pass
