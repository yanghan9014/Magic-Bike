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
#import matplotlib
#import matplotlib.pyplot as plt
import cv2
from rclpy.executors import MultiThreadedExecutor

from rs_ros.rs_listener import CWaitForMessage
from rs_ros.stm_action import MainActionClient
# from rs_ros.yolov5.inference import detect
# import torch
#from rs_ros.lane_detection_wrapper import *
# from rs_ros.find_color_center import color_center_dist

if (os.getenv('ROS_DISTRO') != "dashing"):
    import tf2_ros

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    lock = threading.Lock()

    # model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

    print("Try to listen")
    msg_params = {}
    themes = ['alignedDepthColor', 'colorStream']
    msg_retriever = CWaitForMessage(lock, themes, msg_params)
    time.sleep(3.0)

    executor.add_node(msg_retriever)
    executor_thread = threading.Thread(target=msg_retriever.wait_for_messages, daemon=True, args = (themes, executor))
    executor_thread.start()

    action_client = MainActionClient()
    print("=======================")
    print(" Activate! ")
    print("=======================")
    activate(msg_retriever, action_client)
    action_client.destroy_node()

    #=== Terminating... ===
    executor_thread.join()

def activate(msg_retriever, action_client):

    #net = parsingNet(pretrained = True).cuda() # we dont need auxiliary segmentation in testing
    #state_dict = torch.load('culane_18.pth', map_location = 'cpu')['model']
    #compatible_state_dict = {}
    #for k, v in state_dict.items():
    #    if 'module.' in k:
    #        compatible_state_dict[k[7:]] = v
    #    else:
    #        compatible_state_dict[k] = v

    #net.load_state_dict(compatible_state_dict, strict = False)

    def move(cmd):
        try:
            action_client.send_goal( cmd )
            rclpy.spin_once(action_client)
            time.sleep(0.7)
        except RuntimeError as e:
            print(e)

    def magic_init():
        print("=======================")
        print("Init bike position")
        print("=======================")
        # move("s001")
        # move("f000")
        time.sleep(2.0)
        move("w020")
        time.sleep(1.0)
        move("s000")
        print("fin")

    def turn(degree):
        if degree >= 0:
            move('e0' + str(abs(degree)).zfill(2))
        else:
            move('q0' + str(abs(degree)).zfill(2))
    def right90():
        stop()
        walk()
        turn(30)
        time.sleep(2.0)
        turn(0)

    def left90():
        stop()
        walk()
        turn(-30)
        time.sleep(2.0)
        turn(0)

    def too_close_ratio(dist_thresh=1200):
        try:
            #if not msg_retriever.depth_failed:
                dep = msg_retriever.func_data['alignedDepthColor']['data']
                dist_map = np.where(dep[:300, :] < dist_thresh, 1, 0)
                too_close_area = dist_map.sum()
                # print(too_close_area*1.0/(640*480), end=' ')
                return too_close_area * 1.0 / (640*480)
        except Exception as e:
            print("Error:", e)
            return 1.0

    def collision_avoidance(too_close_ratio_thresh = 0.1):
        brake = True
        while(True):
            if(too_close_ratio() > too_close_ratio_thresh):
                if not brake:
                    brake = True
                    print("too close! Braking...")
                    move('s001')
                    time.sleep(2.0)
            elif brake:
                brake = False
                print("Releasing brake")
                move('s000')


    def lane_detection():
        print(msg_retriever.func_data['colorStream'].keys())
        c_img = msg_retriever.func_data['colorStream']['data']
        #c_img = c_img.cuda()
        #with torch.no_grad():
        #    out = net(c_img)
        #if len(out) == 2:
        #    out, seg_out = out

        #t = time.localtime()
        #timestamp = time.strftime("%H:%M:%S", t)

        #generate_lines(out,c_img.shape, names=timestamp, localization_type = 'rel', flip_updown = True)


    def stalk(color = 'R'):
      print("=======================")
      print(" Start slaking color", color)
      print("=======================")
        
      
      while(True):
        #    c_img = msg_retriever.func_data['colorStream']['data']
        #    results = model(c_img)
        #    results = results.pandas().xyxy[0].to_dict()
        #    if 'person' in results['name'].values():
        #        print("Person detected")
        boundaries = {
        "R": ([170, 80, 10], [180, 255, 255]),
        "r": ([0, 100, 50], [3, 255, 255]),
        "G": ([53, 80, 50], [77, 255, 255]),
        "B": ([100, 100, 50], [112, 255, 255]),
        "Y": ([6, 100, 50], [20, 255, 255]),
        "K": ([0, 0, 0], [180, 255, 30]),
        "W": ([0, 0, 200], [180, 255, 255])
        }
        bd = boundaries[color]
        bd = np.array(bd, dtype='uint8')

        img = msg_retriever.func_data['colorStream']['data']
        HSV_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        mask = cv2.inRange(HSV_img, bd[0], bd[1])
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        tot = np.sum(mask)

        avg_m = np.sum(mask, axis=1)
        avg_n = np.sum(mask, axis=0)

        index_m = np.arange(mask.shape[0])
        index_n = np.arange(mask.shape[1])

        try:
            cen_coords = np.array( [round( np.dot(avg_m, index_m) / tot ), round( np.dot(avg_n, index_n) / tot )], dtype='int32')
            # print("x coords: ", cen_coords[1])

        # depth_img = msg_retriever.func_data['alignedDepthColor']['data']
        # color_dep = np.where(mask != 0, depth_img, 0)
        # color_dep = round( np.sum(color_dep)*255 / tot )
        # color_dep = min(50, color_dep)
        # print("Avg color ", color, " depth:", color_dep)
            delta_angle = np.arctan((cen_coords[1] - 320)*0.8 / 320) * 180. / np.pi
            delta_angle = round(delta_angle)
            delta_angle = max(delta_angle, -90)
            delta_angle = min(delta_angle, 90)
            turn( delta_angle )
            time.sleep(1.0)
        except Exception as e:
            # print(e)
            print("No ", color, " found", end=' ')
            pass

    lock = threading.Lock()
    col = threading.Thread(target = collision_avoidance)
    # sta = threading.Thread(target = stalk)

    # sta.start()
    magic_init()

    col.start()
    move("w020")
    stalk()

    #lane_detection()

    col.join()
    # sta.join()


