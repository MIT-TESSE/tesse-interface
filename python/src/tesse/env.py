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

import socket
import struct

from tesse.msgs import DataResponse


class Env(object):
    def __init__(self, simulation_ip, own_ip, request_port=9000, receive_port=9000):
        self.simulation_ip = simulation_ip
        self.own_ip = own_ip
        self.request_port = request_port
        self.receive_port = receive_port

    def send(self, msg):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # udp socket
        s.sendto(msg.encode(), (self.simulation_ip, self.request_port))
        s.close()

    def request(self, msg, timeout=15):
        # Setup receive socket
        recv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        recv.settimeout(timeout)
        recv.setblocking(1)
        recv.bind((self.own_ip, self.receive_port))
        recv.listen(1)

        # Send request
        self.send(msg)

        # Collect data and construct message
        conn, addr = recv.accept()
        data = bytearray(conn.recv(4))
        tag = data[0:4].decode("utf-8")

        if tag == 'mult':
            img_header_size = 32
            data.extend(conn.recv(8))
            payload_length_imgs = struct.unpack("I", data[4:8])[0]
            data.extend(conn.recv(payload_length_imgs + img_header_size*len(msg.cameras)))
            payload_length_meta = struct.unpack("I", data[8:12])[0]
            data.extend(conn.recv(payload_length_meta))

        elif tag == 'meta' or tag == 'cami' or tag == 'scni':
            data.extend(conn.recv(4))
            data_length = struct.unpack("I", data[4:8])[0]
            data = conn.recv(data_length)

        else:
            raise Exception("Unknown tag received: {}.".format(tag))

        recv.close()
        conn.close()

        if tag == 'mult':
            return DataResponse().decode(data)
        else:
            return data.decode("utf-8")
