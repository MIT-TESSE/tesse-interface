#**************************************************************************************************
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
#**************************************************************************************************

from enum import Enum
import struct

class Transform(object):
    __tag__ = 'TLPT'

    def __init__(self, translate_x=0, translate_z=0, rotate_y=0):
        self.translate_x = translate_x
        self.translate_z = translate_z
        self.rotate_y = rotate_y

    def encode(self):
        payload = bytearray()
        payload.extend( self.__tag__.encode() )
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
        payload.extend(self.__tag__.tag.encode())
        payload.extend(struct.pack("f", self.force_z))
        payload.extend(struct.pack("f", self.torque_y))



class Camera(Enum):
    ALL = -1  # ??????????????????????????????????
    RGB_LEFT = 0
    RGB_RIGHT = 1
    SEGMENTATION = 2
    DEPTH = 3
    THIRD_PERSON = 4

class Compression(Enum):
    OFF = 0
    ON = 0

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
            payload.extend(bytearray(struct.pack('I', camera[0].value))) # Camera
            payload.extend(bytearray(struct.pack('I', camera[1].value))) # Compression
            payload.extend(bytearray(struct.pack('I', camera[2].value))) # Channels

        return payload


class DataResponse(object):
    pass



class GetCameraInformationRequest(object):
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
        payload.extend(bytearray(struct.pack('i', self.camera)))
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
        payload.extend(bytearray(struct.pack('i', self.camera)))
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


class CameraInformation(object):
    __tag__ = 'cami'
    def __init__(self, data):
        self.data = []

    def decode(data):
        self = CameraInformation()

        tag = data[0:4].decode("utf-8")
        payload_length = struct.unpack("I",data[4:8])[0]
        payload = bytearray()
        while( len(payload) < payload_length ):
            data = conn.recv( payload_length )
            payload.extend(data)
        return self
