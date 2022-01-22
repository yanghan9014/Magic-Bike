# Bizzarely-Balancing-Bike (a spooky bike that could stalk you)
Embedded System Final Report   -- By 楊學翰 B08901054 陳韋旭 B08901181
---

## Introduction & Motivation
We originally intend to build a self-balancing & self-navigating bike just like the XUAN-Bike (https://github.com/peng-zhihui/XUAN), but found self-balancing far too difficult. We then shift our focus solely onto self-navigating.

In this project we established the pipeline of RGBD image processing and Jetson Nano-STM32 communication. Although we couldn't complete self-navigation in time, we demonstrated our work by making our bike follow a person (wearing red, for now).

<img src = https://i.imgur.com/egUaXOn.jpg width = 100%>

## Method
### Development boards & motors
- STM32L4
- Jetson Nano
- RealSense depth camera D435
- 45kg-cm PWM controlled servo motor (Fashion Star HP8-A45) * 2
- Electric bike motor

### Hardware installation
We make use of 3D printers and laser cutter to make various kind of mounts
- Install electric motor on the back wheel
- Weld training wheels onto the bike frame
- Handlebar control:  servo motor + belt
- Brake control: servo motor + spool
- Attach RealSense camera

<img src = https://i.imgur.com/9AMWQx9.jpg width = 80%>


### How To Track A Person
- Version 1 – Track the person in red
   1. Take a color image and convert RGB into HSV
      ```
      boundaries = {
        "R": ([170, 80, 10], [180, 255, 255]), ... }
      bd = boundaries[color]
      img = msg_retriever.func_data['colorStream']['data']
      HSV_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

      mask = cv2.inRange(HSV_img, bd[0], bd[1])
      kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
      mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
      ```
   2. Create a mask of the red areas; calculate the average x coordinate of the mask
   3. The Horizontal Field of View(HFOV) is ≈69°; we can calculate ∆θ = arctan(∆x / (320 * cot(HFOV / 2)))
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

### Power System
<img src = https://i.imgur.com/0Y6CQgn.png width = 80%>

### Control System
<img src = https://i.imgur.com/3ZdfIew.png width = 80%>

### Implemented techniques (from Embedded System course)
- Docker
- WiFi AP
- Multithreading
- PWM control
- Serial communication
- Streaming with GStreamer

#### Docker
- Solve dependency issues
- Used docker image from docker hub (https://hub.docker.com/r/osrf/ros2/)
- Enable auto-restart of docker container

#### WiFi AP
- Internet IP changes when Jetson Nano connects to a different network
- Solution: Make Jetson Nano a WiFi AP

#### Multithreading
- Collision avoidance & camera should be constantly on
- Camera
   - Camera input is divided into color and depth
   - Need two subscribers on two seperate threads
   ```
   executor.add_node(msg_retriever)
   executor_thread = threading.Thread(target=msg_retriever.wait_for_messages, daemon=True, args = (themes, executor))
   executor_thread.start()
   ```
- Collision avoidance
   - Create a separate thread to constantly monitor obstacles ahead of the bike
   ```
   col = threading.Thread(target = collision_avoidance)
   ...
   col.start()
   stalk()
   ...
   col.join()
   ```

#### PWM Control
- Use 50 Hz PWM to control Servo (duty cycle: 10%~20%)
- Use 50 kHZ PWM & low-pass filter to simulate DAC
   - Throttle signal: 0.8V ~ 3.6V DC
   - STM32 can only output digital signals or PWM

#### Serial Communication
- Between Jetson Nano & STM32
- Divide the tasks
   - STM32 is a real-time OS. Handles PWM timer to control servo motor
   - Jetson Nano has built-in GPU. Can accelerate image processing speed and provide streaming
   ```
   self.serial_port = serial.Serial(
      port="/dev/ttyACM0",
      baudrate=9600,
      timeout=0.5
   )
   ...
   self.serial_port.write(sendMessage.encode('ascii'))
   ```

#### GStreamer (not finished yet)
- We've successfully streamed with Pi Camera

```gst-launch-1.0   nvarguscamerasrc !       "video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1" !   nvvidconv flip-method=2 !    omxh264enc control-rate=2 bitrate=4000000 !      "video/x-h264, stream-format=byte-stream" !   rtph264pay mtu=1400 !    udpsink host=192.168.10.110 port=5000 sync=false async=false```
- But failed with RealSense
   - Realsense src is not fully supported on GStreamer yet


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

<a href="http://www.youtube.com/watch?feature=player_embedded&v=ACSJ1eG-BSU
" target="_blank"><img src="http://img.youtube.com/vi/ACSJ1eG-BSU/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

## Future Prospects
This project is an unfortunate simplification of an overly ambitious idea.
After this, we plan to...
1. Detect and avoid (or follow) certain objects (cars, people, animals...) on the road with YOLO v5.
2. Detect road edges (https://github.com/amusi/awesome-lane-detection), and apply projective transform to get a bird's-eye-view. Auto-correct bike direction if not parellel to the road.
3. Install GPS module and connect with path-finding API; complete fully automated navigation.


## Reference
Xuan bike github: https://github.com/peng-zhihui/XUAN
Xuan bike YouTube: https://www.youtube.com/watch?v=kCL2d7wZjU8&t=507s&ab_channel=%E7%A8%9A%E6%99%96%E5%90%9B
Road edge detection: https://github.com/amusi/awesome-lane-detection
