#!/usr/bin/env python
# coding: utf-8

# Distribution authorized to U.S. Government agencies and their contractors. Other requests for 
#  this document shall be referred to the MIT Lincoln Laboratory Technology Office.
# 
# This material is based upon work supported by the Under Secretary of Defense for Research and 
# Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions
# or recommendations expressed in this material are those of the author(s) and do not necessarily 
# reflect the views of the Under Secretary of Defense for Research and Engineering.
# 
# © 2019 Massachusetts Institute of Technology.
# The software/firmware is provided to you on an As-Is basis
# 
#  Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 
# or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work 
# are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other 
# than as specifically authorized by the U.S. Government may violate any copyrights that exist in 
# this work.
# **************************************************************************************************

# In[2]:


import socket
import numpy as np
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random
import time
import struct
import math


# In[11]:


class simple_tesse_interface:
    def __init__(self, self_ip, client_ip, request_port=9000, receive_port=9001):
        '''
        This class represents and interface object to the Unity TESSE simulation environment.
        It will create the necessary socket objects required to interface to the 
        simulation environment and release the resources on destruction.
        
        parameters:
            self_ip - ip of the machine that this class is instaniated on
            client_ip - ip of the machine where the simulation is running
            request_port - listen port of the simulation
            receive_port - send port of the simulation
        '''
        
        self.self_ip = self_ip
        self.client_ip = client_ip
        self.request_port = request_port
        self.receive_port = receive_port
        
        # setup socket junk
        self.receive_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # a tcp socket
        self.receive_socket.settimeout(15)
        self.receive_socket.bind((self.self_ip, self.receive_port))
        receive_socket.listen(1)
        
        self.request_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # a udp socket
    
    def teleport(self, x_dist, z_dist, dtheta_y):
        '''
        This function moves the agent 'z_dist' in the z direction (agent-frame),
        'x_dist' in the x direction (agent-frame) and rotates the agent 'dtheta_y' 
        degrees about the y axis of the agent.
        
        Parameters:
            x_dist - distance for the agent to move in (UnityMeters) along 
                     the agent-frame x axis, range [FLT_MIN, FLT_MAX]
            z_dist - distance for the agent to move in (UnityMeters) along 
                     the agent-frame z axis, range [FLT_MIN,FLT_MAX]
            dtheta_y - degrees to rotate the agent around its y-axis, 
                       range [FLT_MIN,FLT_MAX]
        This request does not return a response from the simulation
        '''
        tag = 'TLPT'
        payload = bytearray()
        payload.extend( tag.encode() )
        payload.extend( struct.pack("f", x_dist) )
        payload.extend( struct.pack("f", z_dist ) )
        payload.extend( struct.pack("f", dtheta_y) )
                       
        self.request_socket.sendto( payload, (self.client_ip, self.request_port) ) # send to the simulation
        self.request_socket.close()
    
    def add_force(self, z_force, y_torque):
        '''
        This function adds a force to the simulation agent
        
        parameters:
            z_force - force to add to the z axis of the agent in (UnityNewtons?)
                      valid range is [FLT_MIN,FLT_MAX] - push these limits at your own peril
            y_torque - applies torque around the y axis of the agent in (UnityN*m?)
                       valid range is [FLT_MIN,FLT_MAX]
        This request does not send a response back from the simulation.
        '''
        tag = 'xBFF'
        payload = bytearray()
        payload.extend( tag.encode() )
        payload.extend( struct.pack("f", z_force) )
        payload.extend( struct.pack("f", y_torque) )
        
        self.request_socket.sendto( payload, (self.client_ip, self.request_port) ) # send to the simulation
        self.request_socket.close()
        
    def request_images( ids, get_metadata=False):
        '''
        This function requests images from the simulation from each camera
        specified in the ids list. 'get_metadata' specifies if simulation
        returns the metadata associated with the frames to be returned. The metadata 
        returned with the images is frame-time perfect with the images (e.g. the 
        metadata is from the exact time the images were collected).
        
        parameters
        ids - list of camera ids, valid range is [0, max_camera_id] max_camera_id can
              be found using the camera parameters function with a camera_id of -1. 
              (this will be added at a non-hair-on-fire date).
        get_metadata - boolean if metadata is required or not, valid range is [true,false]
        
        This request sends a response containing each requested image and metadata if requested.
        '''
        if( get_metadata ):
            tag = 'tIMG' #tag for image with metadata request
        else:
            tag = 'rIMG' #tag for image without metadata
        
        payload = bytearray()
        payload.extend( tag.encode() )
        
        for i in range(len(ids)):
            payload.extend( bytearray(struct.pack('I', i)) ) # camera id 
            # image compression flag, 0 if uncompressed, >0 compressed
            payload.extend( bytearray(struct.pack('I', 0)) )
            # single channel image flag, 0 if 3 channel, >0 single channel
            payload.extend( bytearray(struct.pack('I', 0)) ) 
        
        self.request_socket.sendto( payload, (self.client_ip, self.request_port))
        self.request_socket.close()
        
        # from here there is a bunch of networking crap that handles the response
        #from the simulation - the basic gist is that the simulation returns a
        #header that contains a tag and a 4-byte int of how much data there is in the 
        #payload (in bytes), then it grabs those bytes and formats them based on the 
        #header information. If you are curious wait for a later release with better d
        #docs lol.
        
        try:
            # first, get the image header info
            payload_header_size = 12
            img_header_size = 32
            conn, addr = self.receive_socket.accept()
            data = conn.recv(payload_header_size)

            # ensure this is a valid tag
            tag = data[0:4].decode("utf-8")
            payload_length_imgs = struct.unpack("I",data[4:8])[0]
            payload_length_meta = struct.unpack("I",data[8:12])[0]

            imgs = []
            cam_ids = []

            if( tag == "mult" ):
                total_data_received = 0

                while( total_data_received < payload_length_imgs ):
                    # get the header payload
                    #print("fetching data...")
                    data = conn.recv( img_header_size )
                    total_data_received += len(data)
                    if( data[0:4].decode("utf-8") == "uImG" ):
                        # get the rest of the image metadata
                        img_payload_length = struct.unpack("I",data[4:8])[0]
                        img_width = struct.unpack("I",data[8:12])[0]
                        img_height = struct.unpack("I",data[12:16])[0]
                        cam_id = struct.unpack("I",data[16:20])[0]
                        img_type = data[20:24].decode("utf-8")

                    img_payload = bytearray()
                    while( len(img_payload) < img_payload_length ):
                        img_payload.extend( conn.recv(img_payload_length - len(img_payload)) )

                    total_data_received += len(img_payload)

                    if( img_type == 'xRGB' ):
                        img = np.flip(np.ndarray((img_height,img_width,3),buffer=img_payload,dtype='uint8'),0)
                    elif( img_type == 'xGRY' ):
                        img = np.flip(np.ndarray((img_height,img_width),buffer=img_payload,dtype='uint8'),0)
                    elif( img_type == 'cRGB'):
                        ndarr = np.frombuffer(img_payload, dtype=np.uint8)
                        img = cv2.imdecode(ndarr, cv2.IMREAD_UNCHANGED)
                        img = img[:,:,[2,1,0]]

                    imgs.append(img)
                    cam_ids.append(cam_id)

                meta_payload = bytearray()
                if( total_data_received > payload_length_imgs ):
                    print("already got the metadata, just need to extract it")
                    if( total_data_received == (payload_length_imgs + payload_length_meta) ):
                        meta_payload = img_payload[img_payload_length:]
                    elif( total_data_received < (payload_length_imgs + payload_length_meta) ):
                        print("need to get the rest of the metadata")
                        data = conn.recv( (payload_length_imgs + payload_length_meta) - total_data_received )
                        meta_payload = img_payload[img_payload_length:]
                        meta_payload.extend(data)
                elif( total_data_received == payload_length_imgs ):
                    data = conn.recv( payload_length_meta )
                    meta_payload.extend(data)

                conn.close()
        except:
            print("failed!")
            self.receive_socket.close()
            conn.close()
                
            return imgs, meta_payload
        
    def get_metadata():
        '''
        This function gets metadata from the agent.
        
        parameters
            This function takes no inputs
            
        This function returns the metadata for the agent including:
        postion (x,y,z) in Unity coordinates, agent quaternion in Unity convention,
        velocity, acceleration of agent (in body frame), angular rates and acclerations,
        and a timestamp (wall time seconds since start of game).
        '''
        tag = 'rMET' # tag for metadata request

        payload = bytearray()
        payload.extend( tag.encode() )

        # listen for response on tcp socket
        payload_header_size = 8
        conn, addr = self.receive_socket.accept()
        data = conn.recv(payload_header_size)

        try:
            # process header information
            tag = data[0:4].decode("utf-8")
            payload_length_meta = struct.unpack("I",data[4:8])[0]
            if( tag == "meta" ):
                # process metadata payload
                meta_payload = bytearray()
                while( len(meta_payload) < payload_length_meta ):
                    data = conn.recv( payload_length_meta )
                    meta_payload.extend(data)
        except:
            print("failed!")
            receive_socket.close()
            conn.close()

        print(meta_payload.decode("utf-8"))
        conn.close()
        
        return meta_payload


# In[ ]:




