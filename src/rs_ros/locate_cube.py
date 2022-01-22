import numpy as np
import cv2
from PIL import Image
import matplotlib.pyplot as plt

def cube_center_dist(RGB_img, depth_img, color):
    boundaries = {
        "R": ([170, 100, 50], [180, 255, 255]),
        "r": ([0, 100, 50], [3, 255, 255]),
        "G": ([53, 80, 50], [77, 255, 255]),
        "B": ([100, 100, 50], [112, 255, 255]),
        # "Y": ([6, 100, 50], [20, 255, 255]),
        "K": ([0, 0, 0], [180, 255, 30]),
        "W": ([0, 0, 200], [180, 255, 255])
    }
    bdW = boundaries["W"]
    bdW = np.array(bdW, dtype='uint8')
    bdr = boundaries["r"]
    bdr = np.array(bdr, dtype='uint8')
    bd = boundaries[color]
    bd = np.array(bd, dtype='uint8')

    HSV_img = cv2.cvtColor(RGB_img, cv2.COLOR_RGB2HSV)

    mask = cv2.inRange(HSV_img, bd[0], bd[1])
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    maskW = cv2.inRange(HSV_img, bdW[0], bdW[1])
    maskW = cv2.morphologyEx(maskW, cv2.MORPH_CLOSE, kernel, iterations=2)

    maskr = cv2.inRange(HSV_img, bdr[0], bdr[1])
    maskr = cv2.morphologyEx(maskr, cv2.MORPH_CLOSE, kernel, iterations=2)
    
    if color == "R":
        mask = mask + maskr
    # mask = mask + maskW
    # cv2_imshow(mask)
    tot = np.sum(mask)
    if tot == 0:
        print("[Runtime Error] No ", color, " region found" )
        return (0, 0, 0, 0, 0, 0)

    avg_m = np.sum(mask, axis=1)
    avg_n = np.sum(mask, axis=0)

    index_m = np.arange(mask.shape[0])
    index_n = np.arange(mask.shape[1])

    avg_m = round( np.dot(avg_m, index_m) / tot )
    avg_n = round( np.dot(avg_n, index_n) / tot )

    color_dep = 0
    dist_n = 80 / 2.2 * (avg_n - 320) / 320 
    # dist = 40 if (avg_n - 320) > 0 else -40

    if depth_img.size == 307200:
        color_dep = np.where(mask != 0, depth_img, 0)
        color_dep = round( np.sum(color_dep) * 255 / tot )
        color_dep = min(300, color_dep)
        print("Avg color ", color, " depth:", color_dep)
        dist_n = color_dep / 2.2 * (avg_n - 320) / 320
        dist_m = color_dep / 2.2 * (avg_m - 240) / 240
        print("Estimated horizontal distance:", dist_n, "mm")
        dist_n = min(200, dist_n)
        dist_n = max(-200, dist_n)
    return (avg_m, avg_n, mask, color_dep, dist_n, dist_m)

