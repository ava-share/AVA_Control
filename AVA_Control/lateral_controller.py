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
    epilog='A python ROS node implementing a lateral controller for use with '
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
        crosstrack_vector = np.array([np.cos(ref_yaw), np.sin(ref_yaw)])
        crosstrack_err = -(ref_to_axle[0]*crosstrack_vector[1] - \
                           ref_to_axle[1]*crosstrack_vector[0])
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

    def get_steer_angle(self, x, y, yaw, current_velocity, refx, refy, refyaw):
        """
        :param x:
        :param y:
        :param yaw:
        :param current_velocity:
        :return: steering output, target index, crosstrack error
        """
        crosstrack_error = self.compute_error(x, y, yaw, refx, refy, refyaw, self.L)
        if self.config["enable_debugging"]:
            print('Crosstrack_Error: '+str(crosstrack_error))

        # Stanley Control law.
        yaw_term = refyaw - yaw
        tangent_term = np.arctan((self.k*crosstrack_error /
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
            Ts = 1./100.  # sampling rate of steering angle signal
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
    

class DiscreteStateSpace:
    """A discrete state space model implementing the equations:
        x[k+1] = A*x[k] + B*u[k]
        y[k] = C*x[k] + D*u[k] 

    where:
        x[k+1] is the state vector, x, at time step k+1,
        A is the state space matrix
        B is the input matrix
        C is the measurement state matrix
        D is the measurement input matrix
        y[k] is the output, y, at time step k.

    Used to remove dependency on scipy.
    """

    def __init__(self, A, B, C, D, num_states):
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.num_states = num_states
        self.x_k = np.zeros((num_states,1))

    def get_output(self, input):
        """Computes the output of the discrete state equation."""
        new_x = self.A.dot(self.x_k) + self.B.dot(input)
        state_msg = "States: \n"
        for i in range(self.num_states):
            state_msg += "\t %s: %s\n" % (i, self.x_k[i])
        rospy.loginfo(state_msg[:-2])
        output = self.C.dot(self.x_k) + self.D.dot(input)
        self.x_k = new_x
        return float(output)


class SISOLookaheadController(AbstractLateralController):
    """A frequency-based controller designed using the youla parameterization
    technique. Designed in the continuous-time domain and converted to a
    discrete state space equation."""

    def __init__(self, recorder, A, B, C, D, num_states, lookahead, 
                 yaw_err_gain, config):
        self.recorder = recorder
        self.config = config
        self.discrete_ss = DiscreteStateSpace(A, B, C, D, num_states)
        self.lookahead = lookahead
        self.yaw_err_gain = yaw_err_gain

    def get_steer_angle(self, x, y, yaw, ref_x, ref_y, ref_yaw):
        """Compute steer angle using a discrete FIR filter."""
        crosstrack_error = self.compute_error(x, y, yaw, ref_x, ref_y,
                                              ref_yaw, self.lookahead)
        yaw_error = yaw - ref_yaw
        rospy.loginfo("Crosstrack error [m]: %s" % crosstrack_error)
        rospy.loginfo("Yaw Error [deg]: %s" % (yaw_error*180/np.pi))
        error = crosstrack_error + self.yaw_err_gain*yaw_error
        delta = self.discrete_ss.get_output(-error)
        rospy.loginfo("Steer Angle [deg]: %s" % (delta*180/np.pi))
        delta = np.clip(delta, -35 * np.pi / 180, 35 * np.pi / 180)
        return delta


class Data:
    """Class implementation of data logging for lateral controller."""

    def __init__(self, experiment_config):
        self.config = experiment_config
        # TODO: Add flexible attribute so specific attributes don't have to be
        # populated. Ideally, we provide a header, and then the data follows.
        self.steering = []
        self.vx = []
        self.refx = []
        self.refy = []
        self.refyaw = []
        self.egox = []
        self.egoy = []
        self.egoyaw = []
        self.crosstrack_error = []
        self.auto = []  # autonomous or manual?
        self.t = []  # time

    def to_csv(self, file_path):
        """Publish all data to csv files and configuration to json file."""
        # Store data in csv file.
        with open(file_path, 'a') as f:
            m = "t [s], steering [rad], vx [m/s], refx [m], refy [m], "\
                "refyaw [rad], egox [m], egoy [m], egoyaw [rad], "\
                "crosstrack_error [m]\n"
            f.write(m)

            i = 0
            while i < len(self.steering):
                test = ','.join((
                    str(self.t[i]),             str(self.steering[i]),
                    str(self.vx[i]),            str(self.refx[i]),
                    str(self.refy[i]),          str(self.crosstrack_error[i]),
                    str(self.refyaw[i]),        str(self.egox[i]),
                    str(self.egoy[i]),          str(self.egoyaw[i]),
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
        rospy.Subscriber("/current_pose", PoseStamped,
                         callback=self.CurrentPoseCallback, queue_size=1)
        rospy.Subscriber("/current_velocity", TwistStamped,
                         callback=self.CurrentTwistCallback, queue_size=1)

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
                recorder=self.recording,
                config=self.config,
                A=np.array(ctrl_config["A"]),
                B=np.array(ctrl_config["B"]),
                C=np.array(ctrl_config["C"]),
                D=np.array(ctrl_config["D"]),
                num_states=ctrl_config["num_controller_states"],
                lookahead=ctrl_config["lookahead_m"],
                yaw_err_gain=ctrl_config["yaw_err_gain"])
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
        rospy.loginfo("Shutting down lateral_control node.")
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
            rospy.loginfo("Saving data to csv...")
            self.recording.save_data()
            rospy.loginfo("Done saving data to csv.")
        rospy.loginfo("lateral_control node shutdown successfully.")

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
        ref_pose = self.ref_pose
        ego_pose = self.ego_pose
        ego_twist = self.ego_twist

        if ref_pose is not None:
            rospy.loginfo_once("Received reference pose.")
            ref_x, ref_y, ref_yaw = self.unpack_pose(ref_pose)
        else:
            rospy.loginfo_throttle(1, "Waiting for reference pose.")

        if ego_pose is not None:
            rospy.loginfo_once("Received ego pose.")
            ego_x, ego_y, ego_yaw = self.unpack_pose(ego_pose)
        else:
            rospy.loginfo_throttle(1, "Waiting for ego pose.")

        if ego_twist is not None:
            rospy.loginfo_once("Received ego twist.")
            ego_vx = ego_twist.linear.x
        else:
            rospy.loginfo_throttle(1, "Waiting for ego twist.")
        
        if ref_pose is None or \
                ego_pose is None or \
                ego_twist is None:
            return

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
            rospy.loginfo_once("Recording to data logger.")
            self.recording.t.append(rospy.get_rostime())
            self.recording.vx.append(ego_vx)
            self.recording.egox.append(ego_x)
            self.recording.egoy.append(ego_y)
            self.recording.egoyaw.append(ego_yaw)

            self.recording.refx.append(ref_x)
            self.recording.refy.append(ref_y)
            self.recording.refyaw.append(ref_yaw)

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
    # Change something.
    try:
        ROSLateralController()
    except rospy.ROSInterruptException:
        pass
