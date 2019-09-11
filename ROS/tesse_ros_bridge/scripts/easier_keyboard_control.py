from tesse.env import Env
from tesse.msgs import *
import defusedxml.ElementTree as ET
from scipy.spatial.transform import Rotation
from tesse.utils import UdpListener
from pynput import keyboard

"""
1. Run TESSE: `./tesse_multiscene_v0.4_linux.x86_64`.
2. With TESSE in scope:
    a. Press `shift + t`: disables keyboard controls.
    b. Press `shift + t` again to bring them back, if you need them.
3. Run: `python easier_keyboard_control.py`.
4. Get awesome controllability without needing to disable collisions.

Use w,a,s,d for lateral velocity control. Use left and right arrows to rotate. You can increase/decrease speed with up and down arrows. Press esc to stop the script.
This script captures keyboard inputs regardless of what window is in scope. Depending on what you are doing, that might be annoying/frustrating.
"""

env = Env(simulation_ip='localhost',
          own_ip='localhost',
          )


def parse_metadata(metadata):
    data = {}
    root = ET.fromstring(metadata)

    x = float(root.find('position').attrib['x'])
    y = float(root.find('position').attrib['y'])
    z = float(root.find('position').attrib['z'])
    data['position'] = {'x': x, 'y': y, 'z': z}

    x = float(root.find('quaternion').attrib['x'])
    y = float(root.find('quaternion').attrib['y'])
    z = float(root.find('quaternion').attrib['z'])
    w = float(root.find('quaternion').attrib['w'])
    data['quaternion'] = {'x': x, 'y': y, 'z': z, 'w': w}
    data['rotation'] = Rotation((x, y, z, w)).as_euler('zxy')

    x_dot = float(root.find('velocity').attrib['x_dot'])
    y_dot = float(root.find('velocity').attrib['y_dot'])
    z_dot = float(root.find('velocity').attrib['z_dot'])
    data['velocity'] = {'x_dot': x_dot, 'y_dot': y_dot, 'z_dot': z_dot}

    x_ang_dot = float(root.find('angular_velocity').attrib['x_ang_dot'])
    y_ang_dot = float(root.find('angular_velocity').attrib['y_ang_dot'])
    z_ang_dot = float(root.find('angular_velocity').attrib['z_ang_dot'])
    data['angular_velocity'] = {'x_ang_dot': x_ang_dot,
                                'y_ang_dot': y_ang_dot, 'z_ang_dot': z_ang_dot}

    x_ddot = float(root.find('acceleration').attrib['x_ddot'])
    y_ddot = float(root.find('acceleration').attrib['y_ddot'])
    z_ddot = float(root.find('acceleration').attrib['z_ddot'])
    data['acceleration'] = {'x_ddot': x_ddot,
                            'y_ddot': y_ddot, 'z_ddot': z_ddot}

    x_ang_ddot = float(root.find('angular_acceleration').attrib['x_ang_ddot'])
    y_ang_ddot = float(root.find('angular_acceleration').attrib['y_ang_ddot'])
    z_ang_ddot = float(root.find('angular_acceleration').attrib['z_ang_ddot'])
    data['angular_acceleration'] = {
        'x_ang_ddot': x_ang_ddot, 'y_ang_ddot': y_ang_ddot, 'z_ang_ddot': z_ang_ddot}

    data['time'] = float(root.find('time').text)

    data['collision'] = root.find(
        'collision').attrib['status'].lower() == 'true'

    return data


# Initialize commands
v_z_cmd = 0
v_x_cmd = 0
yaw_dot_cmd = 0

# Controller Gains
yaw_dot_error_gain = .5
yaw_ddot_error_gain = .01

v_error_gain = 1
dv_error_gain = .01


def control(data):
    metadata = parse_metadata(data.decode('utf-8'))

    v_z_error = v_z_cmd - metadata['velocity']['z_dot']
    dv_z_error = -1 * metadata['acceleration']['z_ddot']
    force_z_cmd = v_error_gain * v_z_error + dv_error_gain * dv_z_error

    v_x_error = v_x_cmd - metadata['velocity']['x_dot']
    dv_x_error = -1 * metadata['acceleration']['x_ddot']
    force_x_cmd = v_error_gain * v_x_error + dv_error_gain * dv_x_error

    yaw_dot_error = yaw_dot_cmd - metadata['angular_velocity']['y_ang_dot']
    yaw_ddot_error = -1 * metadata['angular_acceleration']['y_ang_ddot']

    torque_cmd = yaw_dot_error * yaw_dot_error_gain + \
        yaw_ddot_error * yaw_ddot_error_gain

    env.send(AddForce(force_z_cmd, torque_cmd, force_x_cmd))


# Maps keys to values for commands
class MyException(Exception):
    pass


speed = 2


def on_press(key):
    global v_z_cmd, v_x_cmd, yaw_dot_cmd, speed
    if key == keyboard.Key.esc:
        raise MyException(key)

    elif key == keyboard.KeyCode.from_char('w'):
        v_z_cmd = speed

    elif key == keyboard.KeyCode.from_char('s'):
        v_z_cmd = -speed

    elif key == keyboard.KeyCode.from_char('d'):
        v_x_cmd = speed

    elif key == keyboard.KeyCode.from_char('a'):
        v_x_cmd = -speed

    elif key == keyboard.Key.left:
        yaw_dot_cmd = -0.5

    elif key == keyboard.Key.right:
        yaw_dot_cmd = 0.5


def on_release(key):
    global v_z_cmd, v_x_cmd, yaw_dot_cmd, speed

    if key == keyboard.KeyCode.from_char('w') or key == keyboard.KeyCode.from_char('s'):
        v_z_cmd = 0.0

    elif key == keyboard.KeyCode.from_char('d') or key == keyboard.KeyCode.from_char('a'):
        v_x_cmd = 0.0

    elif key == keyboard.Key.left or key == keyboard.Key.right:
        yaw_dot_cmd = 0

    elif key == keyboard.Key.up:
        speed += 0.2
        print("Speed set to", speed)

    elif key == keyboard.Key.down:
        speed -= 0.2
        print("Speed set to", speed)


# Start the listener and controller
listener1 = UdpListener(port=9004, rate=100)
listener1.subscribe('example', control)
listener1.start()


# Collect keyboard events to vary commands.
# Note this is always running and will capture keys even if
# this window is not in scope.
with keyboard.Listener(
        on_press=on_press, on_release=on_release) as listener2:
    try:
        listener2.join()
    except MyException as e:
        print('{0} was pressed'.format(e.args[0]))
        listener1.join()
