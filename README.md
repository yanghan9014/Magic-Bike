# Bizzarely-Balancing-Bike (a spooky bike that could stalk you)
Embedded System Final Report
---

## Motivation & ideas
We originally intend to build a self-balancing & self-navigating bike just like the XUAN-Bike (https://github.com/peng-zhihui/XUAN), but found self-balancing far too difficult. We then shift our focus solely onto self-navigating.

## Method
### Development boards & motors
- STM32L4
- Jetson Nano
- RealSense depth camera D435
- 45kg-cm PWM controlled servo motor * 2
- Electric bike motor

### Hardware installation
We make use of 3D printers and laser cutter to make various kind of mounts
- Install electric motor on the back wheel
- Weld training wheels onto the bike frame
- Handlebar control:  servo motor + belt
- Brake control: servo motor + spool
- Attach RealSense camera

### How To Track A Person
- Version 1 – Track the person in red
   1. Take a color image and convert RGB into HSV
   2. Create a mask of the red areas; calculate the average x coordinate of the mask
   3. The Horizontal Field of View is ≈69°; we can calculate ∆θ = arctan(∆x / (320 * cot(HFOV / 2)))
   4. Adjust the handlebar servo motor angle accordingly

- Version 2 - Track the closest person to the bike (not complete yet)
   - Employ YOLO v5 and get the bounding box coordinates of the closest person
   - We got YOLO v5 working on COLAB, but failed to install PyTorch with CUDA support on Jetson Nano
   - Once installation issues got resolved, version 2 would only require a minor fix

### How To Avoid Collision
- Get distance image from depth camera
- Crop out bottom ⅓ of the image (the floor is too close)
- Calculate the total area in which the distance < 1000m
- If the area reaches a certain threshold (5% of the full image), hold brake and cut off back wheel motor power
(We don't simply measure the closest distance to avoid false alarms)

### Implemented techniques (from Embedded System course)
- Docker
- WiFi AP
- Multi-threading
- PWM control
- Serial communication
- Streaming with GStreamer


## Problems & solutions
1. No analog DC signal on STM32 board
Analog DC signal (0.8V ~ 3.6V DC) is required to control the throttle, which is not supported on STM32 GPIOs
   - Solution:
      - Use PWM & RC low-pass filter to simulate analog voltage signal
2. RealSense camera delay
   - Solution: 
      - Slow down bike speed
      - Add delay between each "turn left" or "turn right" command

## Result & demo
Demo video link: https://www.youtube.com/watch?v=ACSJ1eG-BSU
## Future Prospects

## Reference
Xuan bike github: https://github.com/peng-zhihui/XUAN
Xuan bike YouTube: https://www.youtube.com/watch?v=kCL2d7wZjU8&t=507s&ab_channel=%E7%A8%9A%E6%99%96%E5%90%9B
