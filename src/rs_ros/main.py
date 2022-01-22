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

import RPi.GPIO as GPIO

from rs_ros.rs_listener import CWaitForMessage
from rs_ros.stm_action import MainActionClient
from rs_ros.find_color_center import color_center_dist
from rs_ros.stage_1 import exec_stage1
from rs_ros.stage_2 import exec_stage2

if (os.getenv('ROS_DISTRO') != "dashing"):
    import tf2_ros


def main():
    interrupt = 22
    input1 = 27
    input2 = 17
    stage = 1

    def switch_stage(channel):
        bit1 = GPIO.input(input1)
        bit2 = GPIO.input(input2)
        stage = (bit2 == GPIO.HIGH) * 2 + (bit1 == GPIO.HIGH)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(interrupt, GPIO.IN)
    GPIO.setup(input1, GPIO.IN)
    GPIO.setup(input2, GPIO.IN)
    GPIO.add_event_detect(interrupt, GPIO.RISING, callback=switch_stage, bouncetime=10)

    themes = ['colorStream', 'alignedDepthColor']
    msg_params = {}

    rclpy.init()

    executor = MultiThreadedExecutor()
    
    lock = threading.Lock()
    msg_retriever = CWaitForMessage(lock, themes, msg_params)
    
    executor.add_node(msg_retriever)
    executor_thread = threading.Thread(target=msg_retriever.wait_for_messages, daemon=True, args = (themes, executor))
    executor_thread.start()
    
    time.sleep(3.0)

    print("Calling Stage 1")
    # exec_stage1(msg_retriever)
    print("Stage 1 exited")

    print("Calling Stage 2")
    exec_stage2(msg_retriever)
    print("Stage 2 exited")

    print("Calling Stage 3")
    exec_stage3(msg_retriever)
    print("Stage 3 exited")

    executor_thread.join()
