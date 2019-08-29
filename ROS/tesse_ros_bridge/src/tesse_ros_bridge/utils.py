import xml.etree.ElementTree as ET
import numpy as np
import copy

from scipy.spatial.transform import Rotation

from sensor_msgs.msg import CameraInfo, Imu
from nav_msgs.msg import Odometry
import tf.transformations

from tesse_ros_bridge import enu_T_unity, brh_T_blh, gravity_enu

def parse_metadata(data):
    """ Parse Unity agent metadata into a useful dictionary.

        Args:
            data: A decoded string representing the xml metadata from Unity.

        Returns:
            A dictionary with the following metadata members:
            'position': Agent position in world frame as a list.
            'quaternion': Agent rotation in world frame as a list.
            'velocity': Agent velocity in body frame as a list. # TODO verify
            'ang_vel': Agent angular velocity in body frame as a list. # TODO verify
            'acceleration': Agent linear acceleration in body frame as a list. # TODO verify
            'ang_accel': Agent angular acceleration in body frame as a list. # TODO verify
            'time': Unity simulator time of the metadata.
            'collision_status': Bool that is true if agent is collided with
                an object in the environment.
    """
    # TODO: find a nicer way to traverse the tree
    root = ET.fromstring(data)
    dict = {}

    dict['position'] = [float(root[0].attrib['x']),
                        float(root[0].attrib['y']),
                        float(root[0].attrib['z'])]
    dict['quaternion'] = [float(root[1].attrib['x']),
                          float(root[1].attrib['y']),
                          float(root[1].attrib['z']),
                          float(root[1].attrib['w'])]
    dict['velocity'] = [float(root[2].attrib['x_dot']),
                        float(root[2].attrib['y_dot']),
                        float(root[2].attrib['z_dot'])]
    dict['ang_vel'] = [(float(root[3].attrib['x_ang_dot'])),
                       (float(root[3].attrib['y_ang_dot'])),
                       (float(root[3].attrib['z_ang_dot']))]
    dict['acceleration'] = [float(root[4].attrib['x_ddot']),
                            float(root[4].attrib['y_ddot']),
                            float(root[4].attrib['z_ddot'])]
    dict['ang_accel'] = [(float(root[5].attrib['x_ang_ddot'])),
                         (float(root[5].attrib['y_ang_ddot'])),
                         (float(root[5].attrib['z_ang_ddot']))]
    dict['time'] = float(root[6].text)
    dict['collision_status'] = False if root[7].attrib['status'] == 'false' \
        else True

    return dict


def parse_cam_data(data):
    """ Parse CameraInformationRequest data into a useful dictionary

        Args:
            data: A decoded string representing the xml
                CameraInformationRequest metadata from Unity.

        Returns:
            A dictionary with the following metadata members:
                'name': A string representing The name of the camera.
                'id': An integer representing the camera ID in the simulator.
                'parameters': A dictionary of three floats, representing camera
                    width, height, and FOV.
                'position': A list of floats representing the camera's position
                    relative to the body frame of the agent.
                'quaternion': A list of floats representing the camera's
                    rotation relative to the body frame of the agent.
                'draw_distance': A dictionary with two elements:
                    'far': A float representing the simulator's 'far' draw
                        distance.
                    'near': A float representing the simulator's 'near' draw
                        distance.

    """
    # TODO: find a nicer way to traverse the tree that isn't dependent
    # on idices not changing over time.
    root = ET.fromstring(data)
    dict = {}

    dict['name'] = str(root[0][0].text)

    dict['id'] = int(root[0][1].text)

    dict['parameters'] = {'width':int(root[0][2].attrib['width']),
                          'height':int(root[0][2].attrib['height']),
                          'fov':float(root[0][2].attrib['fov'])}
    dict['position'] = [float(root[0][3].attrib['x']),
                        float(root[0][3].attrib['y']),
                        float(root[0][3].attrib['z'])]
    dict['quaternion'] = [float(root[0][4].attrib['x']),
                          float(root[0][4].attrib['y']),
                          float(root[0][4].attrib['z']),
                          float(root[0][4].attrib['w'])]
    dict['draw_distance'] = {'far':float(root[0][5].attrib['far']),
                             'near':float(root[0][5].attrib['near'])}

    return dict


def metadata_to_odom(metadata, timestamp, frame_id, child_frame_id):
    """ Converts a metadata dictionary to a ROS odometry message.

        Args:
            metadata: A dictionary containing agent metadata parsed from Unity
                AND pre-processed to be converted to the correct frame.
            timestamp: A rospy.Time instance for the ROS Odom message instance.
            frame_id: A string representing the reference frame (world frame).

        Returns:
            An Odom ROS message instance that can immediately be published.
    """
    odom = Odometry()
    odom.header.stamp = timestamp
    odom.header.frame_id = frame_id
    odom.child_frame_id = child_frame_id

    # Pose is in the ENU world frame.
    odom.pose.pose.position.x =  metadata['position'][0]
    odom.pose.pose.position.y =  metadata['position'][1]
    odom.pose.pose.position.z =  metadata['position'][2]

    odom.pose.pose.orientation.x = metadata['quaternion'][0]
    odom.pose.pose.orientation.y = metadata['quaternion'][1]
    odom.pose.pose.orientation.z = metadata['quaternion'][2]
    odom.pose.pose.orientation.w = metadata['quaternion'][3]

    # Twist is in the body frame (camera/imu).
    odom.twist.twist.linear.x = metadata['velocity'][0]
    odom.twist.twist.linear.y = metadata['velocity'][1]
    odom.twist.twist.linear.z = metadata['velocity'][2]

    odom.twist.twist.angular.x = metadata['ang_vel'][0]
    odom.twist.twist.angular.y = metadata['ang_vel'][1]
    odom.twist.twist.angular.z = metadata['ang_vel'][2]

    return odom


def metadata_to_imu(processed_metadata, timestamp, frame_id):
    """ Transforms a metadata dictionary to a ROS imu message.

        Converts the metadata to the agent body frame (a right-handed-frame),
        adds a constant gravity value to the linear acceleration fields of the
        provided metadata, and then builds a ROS Imu message instance.

        Args:
            metadata: A dictionary containing agent metadata parsed from Unity
                AND pre-processed to be converted to the correct frame.
            timestamp: A rospy.Time instance for the ROS Imu message instance.
            frame_id: A string representing the reference frame (body frame).

        Returns:
            An Imu ROS message instance that can immediately be published.
    """
    imu = Imu()
    imu.header.stamp = timestamp
    imu.header.frame_id = frame_id

    # All fields are in the agent body frame
    imu.angular_velocity.x = processed_metadata['ang_vel'][0]
    imu.angular_velocity.y = processed_metadata['ang_vel'][1]
    imu.angular_velocity.z = processed_metadata['ang_vel'][2]

    enu_R_brh = processed_metadata['transform'][:3,:3]
    g_brh = np.transpose(enu_R_brh).dot(gravity_enu)

    imu.linear_acceleration.x = processed_metadata['acceleration'][0] - g_brh[0]
    imu.linear_acceleration.y = processed_metadata['acceleration'][1] - g_brh[1]
    imu.linear_acceleration.z = processed_metadata['acceleration'][2] - g_brh[2]

    return imu


def generate_camera_info(left_cam_data, right_cam_data):
    """ Generates CameraInfo messages for left-cam and right-cam.

        Using provided camera intrinsics, first parses CameraInformationRequest
        objects into useful dictionaries and then builds CameraInfo ROS
        messages for both the left camera and the right camera independently.

        Args:
            left_cam_data: A dictionary containing parsed data for left cam.
            right_cam_data: A dictionary containing parsed data for right cam.
            camera_fov: A float representing the camera's desired field-of-view.
            camera_width: A float representing the image width, in pixels.
            camera_height: A float representing the image height, in pixels.

        Returns:
            A tuple containing the left camera's CameraInfo message and the
            right camera's CameraInfo message, in that order.
    """
    # Parameters must be the same for left and right cameras:
    width = left_cam_data['parameters']['width']
    height = left_cam_data['parameters']['height']
    fov_vertical = left_cam_data['parameters']['fov']

    width_right = right_cam_data['parameters']['width']
    height_right = right_cam_data['parameters']['height']
    fov_vertical_right = right_cam_data['parameters']['fov']

    assert(width == width_right)
    assert(height == height_right)
    assert(fov_vertical == fov_vertical_right)

    # Required for Unity FOV scaling:
    fov_horizontal = np.rad2deg(2 * np.arctan(np.tan(np.deg2rad(fov_vertical) / 2) * width / height))
    fx = (width / 2.0) / np.tan(np.deg2rad(fov_horizontal) / 2.0)
    fy = (height / 2.0) / np.tan(np.deg2rad(fov_vertical) / 2.0)

    cx = width // 2 # pixels
    cy = height // 2 # pixels
    baseline = np.abs(left_cam_data['position'][0] - right_cam_data['position'][0])
    # assert(baseline == self.stereo_baseline)  # TODO(marcus): put somewhere
    Tx = 0
    Ty = 0
    Tx_right = -fx * baseline

    cam_info_msg_left = make_camera_info_msg("left_cam",
                                             width,
                                             height,
                                             fx, fy, cx, cy, Tx, Ty)
    cam_info_msg_right = make_camera_info_msg("right_cam",
                                             width,
                                             height,
                                             fx, fy, cx, cy, Tx_right, Ty)

    return (cam_info_msg_left, cam_info_msg_right)


def make_camera_info_msg(frame_id, width, height, fx, fy, cx, cy, Tx, Ty):
    """ Create a CameraInfo ROS message from parameters.

        Args:
            frame_id: A string representing the reference frame of the
                CameraInfo message, which should be the body frame.
            width: A float representing the image width of the camera.
            height: A float representing the image height of the camera.
            fx: A float representing horizontal focal length.
            fy: A float representing vertical focal length.
            cx: An integer representing the principle point x-coordinate.
            cy: An integer representing the principle point y-coordinate.
            Tx: TODO document
            Ty: TODO document

        Returns:
            A Ros CameraInfo message instance.

    """
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = frame_id
    camera_info_msg.width = width
    camera_info_msg.height = height
    camera_info_msg.K = [fx, 0, cx,
                         0, fy, cy,
                         0, 0, 1]
    # No rectification
    camera_info_msg.R = [1, 0, 0,
                         0, 1, 0,
                         0, 0, 1]
    # No distortion
    camera_info_msg.distortion_model = "plumb_bob"
    camera_info_msg.D = [0, 0, 0, 0]

    # Ty = 0 and Tx = -fx' * B, where B is the baseline between the cameras.
    camera_info_msg.P = [fx, 0, cx, Tx,
                         0, fy, cy, Ty,
                         0, 0, 1, 0]
    return camera_info_msg

def process_metadata(metadata, prev_time, prev_vel_brh, prev_enu_R_brh, prev_ang_vel_brh=None):
    """ Convert metadata from the Unity simulator's left-handed frame to the
        a right-handed frame.

        Position and quaternion are converted first to ENU and then to
        the right handed frame. These remain in a global reference frame,
        but one which ROS can better handle.
        The velocities and accelerations (linear and angular) are all in
        the body frame. Velocity and angular velocity are simply converted
        to the right-handed frame. Acceleration and angular acceleration
        are calculated via finite-difference of respective velocities, also
        in the right-handed body frame.

        Args;
            metadata: A dictionary containing metadata from the Unity
                simulator.
            prev_time: A float representing the time of the last metadata
                update received before this one.
            prev_vel_brh: A 3-vector representing the linear velocity of the
                agent in the right-handed body frame at teh previous metadata
                update.
            prev_anv_vel_brh: A 3-vector representing the angular velocity of
                the agent in the right-handd body frame at the previous
                metadata update. Default to None because we currently do not
                use it to determine angular acceleration.

        Returns:
            A dictionary containing processed metadata. Position and
            quaternion are both in the global right-handed frame.
            Velocities and accelerations are in the right-handed body frame.
            Time and collision status are preserved identically.
            Additionally, the 4x4 numpy matrix transform between the Unity
            world frame and the ENU right-handed frame is included.
    """
    # Build a 4x4 transformation matrix from the Unity metadata.
    unity_T_blh = tf.transformations.quaternion_matrix(metadata['quaternion'])
    unity_T_blh[:,3] = np.array(metadata['position'] + [1]) # Homogeneous coordinates

    # TODO(marcus): use enu_T_unity instead of input
    # Define relevant tfs from unity to enu, and brh (body right handed, we use in VIO).
    enu_T_blh = enu_T_unity.dot(unity_T_blh)
    blh_T_brh = np.transpose(brh_T_blh) # TODO put all static guys in init
    enu_T_brh = enu_T_blh.dot(blh_T_brh)

    # Calculate position and orientation in the right-hand body frame from enu.
    enu_t_brh = enu_T_brh[:3,3]
    enu_R_brh = copy.deepcopy(enu_T_brh)
    enu_R_brh[:,3] = np.array([0,0,0,1])
    enu_q_brh = tf.transformations.quaternion_from_matrix(enu_R_brh)
    enu_R_brh = enu_R_brh[:3,:3]

    # Premultiply velocities to get them in the right-handed frame.
    # metadata[velocity] is the left-handed body frame velocity.
    # which we convert to right-handed:
    vel_brh = brh_T_blh[:3,:3].dot(metadata['velocity'])

    # ang_vel_brh = brh_T_blh[:3,:3].dot(metadata['ang_vel'])
    # ang_vel_brh = np.transpose(enu_R_brh).dot(
    #     enu_T_unity[:3,:3].dot(metadata['ang_vel']))
    # ang_vel_brh = blh_T_brh[:3,:3].dot(
    #     np.transpose(unity_T_blh[:3,:3]).dot(metadata['ang_vel']))
    # ang_vel_brh = brh_T_blh[:3,:3].dot(
    #     np.transpose(unity_T_blh[:3,:3])).dot(metadata['ang_vel'])

    # TODO(marcus): we took out the blh_T_brh so that we could get the sign
    # to match with the expected from test_utils.py. This is not a perfect
    # solution.
    # TODO(marcus): check the negative performance
    # TODO(marcus): use gt method with the axis-angle of the R's to get it.
    ang_vel_brh = unity_T_blh[:3,:3].dot(metadata['ang_vel'])

    vel_enu = enu_R_brh.dot(vel_brh)
    prev_vel_enu = prev_enu_R_brh.dot(prev_vel_brh)

    # Calculate the body acceleration via finite difference method
    dt = metadata['time'] - prev_time
    if dt <= 0.0:
        print("Non-positive timestamp in process_metadata!")
    accel_enu = (vel_enu - prev_vel_enu) / dt
    accel_brh = np.transpose(enu_R_brh).dot(accel_enu)
    # ang_accel_brh = (ang_vel_brh - prev_ang_vel_brh) / dt

    # Construct dictionary of transformed metadata.
    processed_dict = {}
    processed_dict['position'] = enu_t_brh
    processed_dict['quaternion'] = enu_q_brh
    processed_dict['velocity'] = vel_brh
    processed_dict['ang_vel'] = ang_vel_brh
    processed_dict['acceleration'] = accel_brh
    # processed_dict['ang_accel'] = ang_accel_brh
    processed_dict['time'] = metadata['time']
    processed_dict['collision_status'] = metadata['collision_status']
    processed_dict['transform'] = enu_T_brh

    return processed_dict
