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
from PIL import Image
import io
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


class Interface(Enum):
    POSITION = 0
    METADATA = 1
    IMAGE = 2
    STEP = 3


class ObjectType(Enum):
    CUBE = 0


class ObjectSpawnMethod(Enum):
    USER = 0 # spawn object at user specified location
    RANDOM = 1 # randomly spawn object in scene


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

    def get_interface(self):
        return self.__interface__


# POSITION INTERFACE

class PositionMessage(AbstractMessage):
    __interface__ = Interface.POSITION

    def __init__(self, *message_contents):
        super(PositionMessage, self).__init__(*message_contents)


class Transform(PositionMessage):
    __tag__ = 'TLPT'

    def __init__(self, translate_x=0, translate_z=0, rotate_y=0):
        super(Transform, self).__init__(('f', translate_x), ('f', translate_z), ('f', rotate_y))


class AddForce(PositionMessage):
    __tag__ = 'xBff'

    def __init__(self, force_z=0, torque_y=0, force_x=0):
        super(AddForce, self).__init__(('f', force_z), ('f', torque_y), ('f', force_x))


class Reposition(PositionMessage):
    __tag__ = 'sPoS'

    def __init__(self, position_x=0, position_y=0, position_z=0, orientation_x=0, orientation_y=0, orientation_z=0, orientation_w=0):
        super(Reposition, self).__init__(
            ('f', position_x),
            ('f', position_y),
            ('f', position_z),
            ('f', orientation_x),
            ('f', orientation_y),
            ('f', orientation_z),
            ('f', orientation_w),
        )


class SetHoverHeight(PositionMessage):
    __tag__ = 'xSHh'

    def __init__(self, height=2.5):
        super(SetHoverHeight, self).__init__(('f', height))


class Respawn(PositionMessage):
    __tag__ = 'RSPN'


class SetFrameRate(PositionMessage):
    __tag__ = 'fScR'

    def __init__(self, frame_rate=0):
        super(SetFrameRate, self).__init__(('I', frame_rate))


class SetRandomSeed(PositionMessage):
    __tag__ = 'SEED'

    def __init__(self, seed=0):
        super(SetRandomSeed, self).__init__(('i', seed))


class SceneRequest(PositionMessage):
    __tag__ = 'CScN'

    def __init__(self, index=0):
        super(SceneRequest, self).__init__(('i', index))


class ColliderRequest(PositionMessage):
    __tag__ = 'sCOL'

    def __init__(self, enable=1):  # 0: disables collisions, 1: enables collisions
        super(ColliderRequest, self).__init__(('B', enable))


class SpawnObjectRequest(PositionMessage):
    __tag__ = 'oSpn'

    def __init__(self, object_type=ObjectType.CUBE, method=ObjectSpawnMethod.USER,
    position_x=0, position_y=0, position_z=0, orientation_x=0, orientation_y=0, orientation_z=0, orientation_w=0):
        super(SpawnObjectRequest, self).__init__(
            ('i', object_type.value),
            ('i', method.value),
            ('f', position_x),
            ('f', position_y),
            ('f', position_z),
            ('f', orientation_x),
            ('f', orientation_y),
            ('f', orientation_z),
            ('f', orientation_w),
        )


class RemoveObjectsRequest(PositionMessage):
    __tag__ = 'oRem'

    def __init__(self, ids=[]):
        # Examples: 
        #   - RemoveObjectsRequest([1,2,3]) removes ids 1, 2, and 3
        #   - RemoveObjectsRequest() removes all objects
        ids_with_encoding = [('i', id) for id in ids]
        super(RemoveObjectsRequest, self).__init__(*ids_with_encoding)


class ObjectsRequest(PositionMessage):
    __tag__ = 'oReq'

    def __init__(self):
        super( ObjectsRequest, self).__init__()


# METADATA INTERFACE

class MetadataMessage(AbstractMessage):
    __interface__ = Interface.METADATA

    def __init__(self, *message_contents):
        super(MetadataMessage, self).__init__(*message_contents)


class MetadataRequest(MetadataMessage):
    __tag__ = 'rMET'


# IMAGE INTERFACE

class ImageMessage(AbstractMessage):
    __interface__ = Interface.IMAGE

    def __init__(self, *message_contents):
        super(ImageMessage, self).__init__(*message_contents)


class DataRequest(ImageMessage):
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


class CameraInformationRequest(ImageMessage):
    __tag__ = 'gCaI'

    def __init__(self, camera=Camera.ALL):
        super(CameraInformationRequest, self).__init__(('i', camera.value))


class SetCameraParametersRequest(ImageMessage):
    __tag__ = 'sCaR'

    def __init__(self, 
                 camera=Camera.ALL,
                 height_in_pixels=320, 
                 width_in_pixels=480, 
                 field_of_view=60,
                 near_clip_plane=0.3,
                 far_clip_plane=50,
                 ):
        super(SetCameraParametersRequest, self).__init__(('i', camera.value),
                                                         ('i', height_in_pixels), 
                                                         ('i', width_in_pixels), 
                                                         ('f', field_of_view), 
                                                         ('f', near_clip_plane),
                                                         ('f', far_clip_plane),
                                                         )


class SetCameraPositionRequest(ImageMessage):
    __tag__ = 'sCaP'

    def __init__(self, camera=Camera.ALL, x=0, y=0, z=0):
        super(SetCameraPositionRequest, self).__init__(('i', camera.value), ('f', x), ('f', y), ('f', z))


class SetCameraOrientationRequest(ImageMessage):
    __tag__ = 'sCaQ'

    def __init__(self, camera=Camera.ALL, x=0, y=0, z=0, w=1):
        super(SetCameraOrientationRequest, self).__init__(('i', camera.value), ('f', x), ('f', y), ('f', z), ('f', w))


class DataResponse(object):
    def __init__(self, images=None, metadata=None):
        self.metadata = None
        self.images = []
        self.cameras = []
        self.types = []

        if images is not None:
            self._decode_images(images)
        if metadata is not None:
            self._decode_metadata(metadata)

    def _decode_images(self, images=None):
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
                img = Image.open(io.BytesIO(img))
            else:  # 'xRGB', 'xGRY', or 'xFLT
                img = img.reshape(img_height, img_width, -1).squeeze()
                img = np.flip(img, 0)  # flip vertically

            if img_type == 'xFLT':
                # decode an RGBA color image into a float32 image
                img = np.dot(img, np.asarray((1.0, 1.0/255.0, 1.0/(255.0*255.0), 1.0/(255.0*255.0*255.0)))).astype('float32')
                img /= 255.0

            images = images[img_payload_length:]

            self.images.append(img)
            self.cameras.append(cam_id)
            self.types.append(img_type)

    def _decode_metadata(self, metadata=None):
        # self.metadata = bytes(metadata).decode('utf-8')  # python 3
        self.metadata = metadata.tobytes().decode('utf-8')  # python 2/3


# STEP INTERFACE

class StepMessage(AbstractMessage):
    __interface__ = Interface.STEP

    def __init__(self, *message_contents):
        super(StepMessage, self).__init__(*message_contents)

class StepWithForce(StepMessage):
    __tag__ = 'fBff'

    def __init__(self, force_z=0, torque_y=0, force_x=0, duration=0):
        super(StepWithForce, self).__init__(('f', force_z), ('f', torque_y), ('f', force_x), ('f', duration))

class StepWithTransform(StepMessage):
    __tag__ = 'tlpt'

    def __init__(self, translate_x=0, translate_z=0, rotate_y=0):
        super(StepWithTransform, self).__init__(('f', translate_x), ('f', translate_z), ('f', rotate_y))
