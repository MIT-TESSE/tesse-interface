###################################################################################################
# Distribution authorized to U.S. Government agencies and their contractors. Other requests for
# this document shall be referred to the MIT Lincoln Laboratory Technology Office.
#
# This material is based upon work supported by the Under Secretary of Defense for Research and
# Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions
# or recommendations expressed in this material are those of the author(s) and do not necessarily
# reflect the views of the Under Secretary of Defense for Research and Engineering.
#
# (c) 2019 Massachusetts Institute of Technology.
#
# The software/firmware is provided to you on an As-Is basis
#
# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013
# or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work
# are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other
# than as specifically authorized by the U.S. Government may violate any copyrights that exist in
# this work.
###################################################################################################

import struct
import numpy as np

from abc import ABCMeta
from enum import Enum


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


class AbstractMessage:
    __metaclass__ = ABCMeta

    def __init__(self, *message_contents):
        self.message_contents = message_contents

    def encode(self):
        payload = bytearray()
        payload.extend(self.__tag__.encode())
        for attribute in self.message_contents:
            payload.extend(struct.pack(*attribute))
        return payload


# POSITION INTERFACE

class Transform(AbstractMessage):
    __tag__ = 'TLPT'

    def __init__(self, translate_x=0, translate_z=0, rotate_y=0):
        super(Transform, self).__init__(('f', translate_x), ('f', translate_z), ('f', rotate_y))


class AddRelativeForceAndTorque(AbstractMessage):
    __tag__ = 'xBFF'

    def __init__(self, force_z=0, torque_y=0):
        super(AddRelativeForceAndTorque, self).__init__(('f', force_z), ('f', torque_y))


class Reposition(AbstractMessage):
    __tag__ = 'sPoS'

    def __init__(self, position_x=0, position_y=0, position_z=0, orientation_x=0, orientation_y=0, orientation_z=0, orientation_w=0):
        super(Move, self).__init__(
            ('f', position_x),
            ('f', position_y),
            ('f', position_z),
            ('f', orientation_x),
            ('f', orientation_y),
            ('f', orientation_z),
            ('f', orientation_w),
        )


class Respawn(AbstractMessage):
    __tag__ = 'RSPN'


class SceneRequest(AbstractMessage):
    __tag__ = 'CScN'

    def __init__(self, index=0):
        super(SceneRequest, self).__init__(('i', index))


# METADATA INTERFACE

class MetadataRequest(AbstractMessage):
    __tag__ = 'rMET'


# IMAGE INTERFACE

class DataRequest(AbstractMessage):
    def __init__(self,
                 metadata=True,
                 cameras=[(Camera.RGB_LEFT, Compression.OFF, Channels.THREE),
                          (Camera.RGB_RIGHT, Compression.OFF, Channels.THREE),
                          (Camera.SEGMENTATION, Compression.OFF, Channels.THREE),
                          (Camera.DEPTH, Compression.OFF, Channels.THREE),
                          (Camera.THIRD_PERSON, Compression.OFF, Channels.THREE)
                          ]):
        self._validate_cameras(cameras)

        self.__tag__ = 'tIMG' if metadata else 'rIMG'
        camera_vals = []
        for camera in cameras:
            camera_vals.append(('I', camera[0].value))
            camera_vals.append(('I', camera[1].value))
            camera_vals.append(('I', camera[2].value))
        super(DataRequest, self).__init__(*camera_vals)

    def _validate_cameras(self, cameras):
        for camera in cameras:
            if camera[1] == Compression.ON and camera[2] == Channels.SINGLE:
                raise ValueError('Invalid camera configuration')


class CameraInformationRequest(AbstractMessage):
    __tag__ = 'gCaI'

    def __init__(self, camera=Camera.ALL):
        super(CameraInformationRequest, self).__init__(('i', camera.value))


class SetCameraParametersRequest(AbstractMessage):
    __tag__ = 'sCaR'

    def __init__(self, height_in_pixels=320, width_in_pixels=480, field_of_view=60, camera=Camera.ALL):
        super(SetCameraParametersRequest, self).__init__(('i', height_in_pixels), ('i', width_in_pixels), ('f', field_of_view), ('i', camera.value))


class SetCameraPositionRequest(AbstractMessage):
    __tag__ = 'sCaP'

    def __init__(self, x=0, y=0, z=0, camera=Camera.ALL):
        super(SetCameraPositionRequest, self).__init__(('f', x), ('f', y), ('f', z), ('i', camera.value))


class SetCameraOrientationRequest(AbstractMessage):
    __tag__ = 'sCaQ'

    def __init__(self, x=0, y=0, z=0, w=1, camera=Camera.ALL):
        super(SetCameraOrientationRequest, self).__init__(('f', x), ('f', y), ('f', z), ('f', w), ('i', camera.value))


class DataResponse(object):
    def __init__(self, images=None, metadata=None):
        self.data = None
        self.images = []
        self.cameras = []
        self.types = []

        self._decode_images(images)
        self._decode_metadata(metadata)

    def _decode_images(self, images=None):
        if images is not None:
            while len(images) > 0:
                img_payload_length = struct.unpack("I", images[4:8])[0]  # the first 4 is an unused header
                img_width = struct.unpack("I", images[8:12])[0]
                img_height = struct.unpack("I", images[12:16])[0]
                cam_id = struct.unpack("I", images[16:20])[0]
                # img_type = bytes(images[20:24]).decode("utf-8")  # python 3
                img_type = images[20:24].tobytes().decode("utf-8")  # python 2/3
                images = images[32:]  # everything except the header

                # img = np.frombuffer(images[:img_payload_length], dtype=np.uint8)  # python 3
                img = np.frombuffer(images[:img_payload_length].tobytes(), dtype=np.uint8)  # python 2/3
                if img_type == 'cRGB':
                    import cv2
                    img = cv2.imdecode(img, cv2.IMREAD_UNCHANGED)[:, :, ::-1]
                else:
                    img = img.reshape(img_height, img_width, -1).squeeze()
                    img = np.flip(img, 0)  # flip vertically

                images = images[img_payload_length:]

                self.images.append(img)
                self.cameras.append(cam_id)
                self.types.append(img_type)

    def _decode_metadata(self, metadata=None):
        if metadata is not None:
            # self.data = bytes(metadata).decode('utf-8')  # python 3
            self.data = metadata.tobytes().decode('utf-8')  # python 2/3
