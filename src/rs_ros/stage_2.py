import sys
import time
import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import Imu as msg_Imu
import numpy as np
import inspect
import ctypes
import struct
import os
import threading
import matplotlib
import matplotlib.pyplot as plt
import cv2
from rclpy.executors import MultiThreadedExecutor

from rs_ros.rs_listener import CWaitForMessage
from rs_ros.stm_action import MainActionClient
from rs_ros.locate_cube import cube_center_dist

if (os.getenv('ROS_DISTRO') != "dashing"):
    import tf2_ros

def exec_stage2(msg_retriever):
    action_client = MainActionClient()

    def move(cmd):
        try:
            action_client.send_goal( cmd )
            rclpy.spin_once(action_client)
            time.sleep(1.0)
        except RuntimeError as e:
            print(e)

    def ready_stance():
        time.sleep(1.0)
        move("f075")
        time.sleep(1.0)
        move("t000")
        time.sleep(1.0)
        move("t090")

    def grab():
        move("l085")
        move("t045")
        move("f030")
        move("g000")

    def dump():
        move("l010")
        move("t000")
        time.sleep(1.0)
        move("t-60")
        move("f110")
        move("g001")
    
    def openfunnel():
        move("r001")

    def closefunnel():
        move("r000")

    def opendoor():
        move("m001")

    def closedoor():
        move("m000")

    def finilize():
        move("t-90")
        move("f090")


    time.sleep(1.0)
    print("")
    print("=============")
    print("START Stage 2")
    print("=============")
    
    move("w090")
    time.sleep(4.0)
    
    move("r001")
    move("f075")
    move("t090")
    move("g001")
    move("i000")

    print("")
    print("=============")
    print(" Initialized ")
    print("=============")

    move("q040")
    # grab()
    # dump()
    ready_stance()
    
    cube = ["B", "G", "R"]

    for c in range(3):
        c_img = msg_retriever.func_data['colorStream']['data']
        dep = np.zeros(1)
        try:
            if not msg_retriever.depth_failed:
                dep = msg_retriever.func_data['alignedDepthColor']['data']
                dep3 = np.expand_dims(dep, axis=2)
                dep3 = np.concatenate((dep3, dep3, dep3), axis=2)
                c_img = np.where(dep3 != 1500, c_img, 0 )
        except (KeyError, ValueError) as e:
            print(e)
        cen = cube_center_dist( c_img, dep, cube[c])
        while cen == (0, 0, 0, 0, 0, 0):
            time.sleep(0.5)
            c_img = msg_retriever.func_data['colorStream']['data']
            dep = np.zeros(1)
            try:
                if not msg_retriever.depth_failed:
                    dep = msg_retriever.func_data['alignedDepthColor']['data']
                    dep3 = np.expand_dims(dep, axis=2)
                    dep3 = np.concatenate((dep3, dep3, dep3), axis=2)
                    c_img = np.where(dep3 != 1500, c_img, 0 )
            except (KeyError, ValueError) as e:
                print(e)
            cen = cube_center_dist( cimg, dep, cube[c])

        while abs( cen[4] ) > 18:
            est_dist = cen[4] / 10.0
            abs_dist = round( abs(est_dist) )
            abs_dist = f'{(abs_dist):03}'
            fix_dist = "001"

            direc = 'a' if est_dist < 0 else 'd'
            print(direc + fix_dist)
            
            move(direc + str(abs_dist))

            c_img = msg_retriever.func_data['colorStream']['data']
            dep = np.zeros(1)
            try:
                dep = msg_retriever.func_data['alignedDepthColor']['data']
                dep3 = np.expand_dims(dep, axis=2)
                dep3 = np.concatenate((dep3, dep3, dep3), axis=2)
                c_img = np.where(dep3 != 1500, c_img, 0 )
            except (KeyError, ValueError) as e:
                print(e)
            cen = cube_center_dist( c_img, dep, cube[c])
            while cen == (0, 0, 0, 0, 0, 0):
                time.sleep(0.5)
                c_img = msg_retriever.func_data['colorStream']['data']
                dep = np.zeros(1)
                try:
                    if not msg_retriever.depth_failed:
                        dep = msg_retriever.func_data['alignedDepthColor']['data']
                        dep3 = np.expand_dims(dep, axis=2)
                        dep3 = np.concatenate((dep3, dep3, dep3), axis=2)
                        c_img = np.where(dep3 != 1500, c_img, 0 )
                except (KeyError, ValueError) as e:
                    print(e)
                cen = cube_center_dist( cimg, dep, cube[c])
        while abs( cen[3] ) - 230 > 30:
            d_dist = round( abs(cen[3] - 230) / 10)
            d_dist = f'{(d_dist):03}'
            if cen[2] > 600:
                move("s010")
            if cen[3] > 240:
                move("w" + d_dist)
            else: 
                move("s" + d_dist)
        grab()
        dump()
        move("s020")
        time.sleep(1.0)
        ready_stance()
