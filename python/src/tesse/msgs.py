###################################################################################################
# Distribution authorized to U.S. Government agencies and their contractors. Other requests for
# this document shall be referred to the MIT Lincoln Laboratory Technology Office.
#
# This material is based upon work supported by the Under Secretary of Defense for Research and
# Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions
# or recommendations expressed in this material are those of the author(s) and do not necessarily
# reflect the views of the Under Secretary of Defense for Research and Engineering.
#
# © 2019 Massachusetts Institute of Technology.
#
# The software/firmware is provided to you on an As-Is basis
#
# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013
# or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work
# are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other
# than as specifically authorized by the U.S. Government may violate any copyrights that exist in
# this work.
###################################################################################################

from enum import Enum
import struct
import numpy as np


class Transform(object):
    __tag__ = 'TLPT'

    def __init__(self, translate_x=0, translate_z=0, rotate_y=0):
        self.translate_x = translate_x
        self.translate_z = translate_z
        self.rotate_y = rotate_y

    def encode(self):
        payload = bytearray()
        payload.extend(self.__tag__.encode())
        payload.extend(struct.pack("f", self.translate_x))
        payload.extend(struct.pack("f", self.translate_z))
        payload.extend(struct.pack("f", self.rotate_y))
        return payload


class AddRelativeForceAndTorque(object):
    __tag__ = 'xBFF'

    def __init__(self, force_z=0, torque_y=0):
        self.force_z = force_z
        self.torque_y = torque_y

    def encode(self):
        payload = bytearray()
        payload.extend(self.__tag__.encode())
        payload.extend(struct.pack("f", self.force_z))
        payload.extend(struct.pack("f", self.torque_y))
        return payload


class Respawn(object):
    __tag__ = 'CScN'

    def encode(self):
        payload = bytearray()
        payload.extend(self.__tag__.encode())
        return payload


class Camera(Enum):
    ALL = -1  # ?
    RGB_LEFT = 0
    RGB_RIGHT = 1
    SEGMENTATION = 2
    DEPTH = 3
    THIRD_PERSON = 4


class Compression(Enum):
    OFF = 0
    ON = 1


class Channels(Enum):
    THREE = 0
    SINGLE = 1


class DataRequest(object):
    def __init__(self, metadata=True,
                 cameras=[(Camera.RGB_LEFT, Compression.OFF, Channels.THREE),
                          (Camera.RGB_RIGHT, Compression.OFF, Channels.THREE),
                          (Camera.SEGMENTATION, Compression.OFF, Channels.THREE),
                          (Camera.DEPTH, Compression.OFF, Channels.THREE),
                          (Camera.THIRD_PERSON, Compression.OFF, Channels.THREE)
                          ]):
        self.metadata = metadata
        self.cameras = cameras

    def encode(self):
        payload = bytearray()

        if self.metadata:
            payload.extend('tIMG'.encode())
        else:
            payload.extend('rIMG'.encode())

        for camera in self.cameras:
            payload.extend(bytearray(struct.pack('I', camera[0].value)))  # Camera
            payload.extend(bytearray(struct.pack('I', camera[1].value)))  # Compression
            payload.extend(bytearray(struct.pack('I', camera[2].value)))  # Channels

        return payload


class DataResponse(object):
    def __init__(self):
        self.data = []
        self.images = []
        self.cameras = []
        self.types = []

    def decode(self, data):
        _, data = (data[:4].decode("utf-8"), data[4:])
        payload_length_imgs, data = (struct.unpack("I", data[:4])[0], data[4:])
        _, data = (struct.unpack("I", data[:4])[0], data[4:])

        self.data = data[payload_length_imgs:].decode("utf-8")
        (self.images, self.cameras, self.type) = self.decode_images(data[:payload_length_imgs])

        return self

    def decode_images(self, data):
        (images, cameras, types) = ([], [], [])
        while len(data) > 0:
            _, data = (data[:4].decode("utf-8"), data[4:])
            img_payload_length, data = (struct.unpack("I", data[:4])[0], data[4:])
            img_width, data = (struct.unpack("I", data[:4])[0], data[4:])
            img_height, data = (struct.unpack("I", data[:4])[0], data[4:])
            cam_id, data = (struct.unpack("I", data[:4])[0], data[4:])
            img_type, data = (data[:4].decode("utf-8"), data[4:])
            data = data[8:]  # Why?

            # Pull out image
            if img_type == 'xRGB':
                img = np.flip(np.ndarray((img_height, img_width, 3), buffer=data[:img_payload_length], dtype='uint8'), 0)
            elif img_type == 'xGRY':
                img = np.flip(np.ndarray((img_height, img_width), buffer=data[:img_payload_length], dtype='uint8'), 0)
            elif img_type == 'cRGB':
                import cv2
                ndarr = np.frombuffer(data[:img_payload_length], dtype=np.uint8)
                img = cv2.imdecode(ndarr, cv2.IMREAD_UNCHANGED)
                img = img[:, :, [2, 1, 0]]

            data = data[img_payload_length:]

            images.append(img)
            cameras.append(cam_id)
            types.append(img_type)

        return images, cameras, types


class MetadataRequest(object):
    __tag__ = 'rMET'

    def encode(self):
        payload = bytearray()
        payload.extend(self.__tag__.encode())
        return payload


class CameraInformationRequest(object):
    __tag__ = 'gCaI'

    def __init__(self, camera=Camera.ALL):
        self.camera = camera

    def encode(self):
        payload = bytearray()
        payload.extend(self.__tag__.encode())
        payload.extend(bytearray(struct.pack('i', -1)))
        return payload


class SetCameraParametersRequest(object):
    __tag__ = 'sCaR'

    def __init__(self, height_in_pixels=320, width_in_pixels=480, field_of_view=60, camera=Camera.ALL):
        self.height_in_pixels = height_in_pixels
        self.width_in_pixels = width_in_pixels
        self.field_of_view = field_of_view
        self.camera = camera

    def encode(self):
        payload = bytearray()
        payload.extend(self.__tag__.encode())
        payload.extend(bytearray(struct.pack('i', self.height_in_pixels)))
        payload.extend(bytearray(struct.pack('i', self.width_in_pixels)))
        payload.extend(bytearray(struct.pack('f', self.field_of_view)))
        payload.extend(bytearray(struct.pack('i', self.camera.value)))
        return payload


class SetCameraPositionRequest(object):
    __tag__ = 'sCaP'

    def __init__(self, x=0, y=0, z=0, camera=Camera.ALL):
        self.x = x
        self.y = y
        self.z = z
        self.camera = camera

    def encode(self):
        payload = bytearray()
        payload.extend(self.__tag__.encode())
        payload.extend(bytearray(struct.pack('f', self.x)))
        payload.extend(bytearray(struct.pack('f', self.y)))
        payload.extend(bytearray(struct.pack('f', self.z)))
        payload.extend(bytearray(struct.pack('i', self.camera.value)))
        return payload


class SetCameraOrientationRequest(object):
    __tag__ = 'sCaQ'

    def __init__(self, x=0, y=0, z=0, w=1, camera=Camera.ALL):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        self.camera = camera

    def encode(self):
        payload = bytearray()
        payload.extend(self.__tag__.encode())
        payload.extend(bytearray(struct.pack('f', self.x)))
        payload.extend(bytearray(struct.pack('f', self.y)))
        payload.extend(bytearray(struct.pack('f', self.z)))
        payload.extend(bytearray(struct.pack('f', self.w)))
        payload.extend(bytearray(struct.pack('i', self.camera)))
        return payload


class SceneRequest(object):
    __tag__ = 'CScN'

    def __init__(self, index=0):
        self.index = index

    def encode(self):
        payload = bytearray()
        payload.extend(self.__tag__.encode())
        payload.extend(bytearray(struct.pack('i', 5)))
        return payload
