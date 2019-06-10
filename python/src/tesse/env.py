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

import socket
import struct

from tesse.msgs import Interface
from tesse.msgs import DataResponse


class Env(object):
    def __init__(self, simulation_ip, own_ip, position_port=9000, metadata_port=9001, image_port=9002):
        self.simulation_ip = simulation_ip
        self.own_ip = own_ip
        self.position_port = position_port
        self.metadata_port = metadata_port
        self.image_port = image_port

    def get_port(self, msg):
        if msg.get_interface() == Interface.POSITION:
            port = self.position_port
        elif msg.get_interface() == Interface.METADATA:
            port = self.metadata_port
        else:
            port = self.image_port
        return port

    def send(self, msg):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # udp socket
        s.sendto(msg.encode(), (self.simulation_ip, self.get_port(msg)))
        s.close()

    def request(self, msg, timeout=1):
        # setup receive socket
        recv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        recv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        recv.settimeout(timeout)
        recv.bind((self.own_ip, self.get_port(msg)))
        recv.listen(1)

        # send request
        self.send(msg)

        # wait for a connection
        try:
            conn, addr = recv.accept()
            conn.setblocking(1)
        except socket.timeout:
            recv.close()
            return

        # get message tag
        tag = conn.recv(4).decode('utf-8')
        if tag not in ['mult', 'meta', 'cami', 'scni']:
            conn.close()
            recv.close()
            raise ValueError('Unknown tag received {}'.format(tag))

        # get maximum message payload length
        if tag == 'mult':
            header = conn.recv(8)
            payload_length_imgs = struct.unpack("I", header[:4])[0]
            payload_length_meta = struct.unpack("I", header[4:])[0]
            # TODO: remove this hack when metadata payload length is fixed
            if payload_length_meta == 1:
                payload_length_meta = 4
            max_payload_length = payload_length_imgs + payload_length_meta
        else:
            header = conn.recv(4)
            max_payload_length = struct.unpack("I", header)[0]

        # allocate payload buffer
        payload = bytearray(max_payload_length)

        # get payload
        total_bytes_read = 0
        payload_view = memoryview(payload)
        while total_bytes_read < max_payload_length:
            bytes_read = conn.recv_into(payload_view, max_payload_length - total_bytes_read)
            if bytes_read == 0:
                break
            payload_view = payload_view[bytes_read:]
            total_bytes_read += bytes_read
        payload_view = memoryview(payload)[:total_bytes_read]

        # close socket
        conn.close()
        recv.close()

        # parse payload buffer
        if tag == 'mult':
            imgs_payload = payload_view[:-payload_length_meta]
            meta_payload = payload_view[-payload_length_meta:]
            return DataResponse(images=imgs_payload, metadata=meta_payload)
        else:
            return DataResponse(metadata=payload_view)
