# Bizzarely-Balancing-Bike (a spooky bike that could stalk you)
Embedded System Final Report
---

## Motivation & ideas
We originally intend to build a self-balancing & self-navigating bike just like the XUAN-Bike (https://github.com/peng-zhihui/XUAN), but found self-balancing far too difficult. We then shift our focus solely onto self-navigating.

## Method
### How To Track A Person
- Version 1 – Track the person in red
- Version 2 - Track the closest person to the bike (not complete yet)

### How To Avoid Collision
- Get distance image from depth camera
- Crop out bottom ⅓ of the image (the floor is too close)
- If the closest distance < 1000mm, hold brake and cut off motor power

### Implemented techniques (from Embedded System course)
- Docker
- WiFi AP
- Multi-threading
- PWM control
- Serial communication
- Streaming with GStreamer


## Problems & solutions
1. No analog DC signal on STM32 board
- Solution:
2. 

## Result & demo
Demo video link: https://www.youtube.com/watch?v=ACSJ1eG-BSU
## Future Prospects

## Reference
Xuan bike github: https://github.com/peng-zhihui/XUAN
Xuan bike YouTube: https://www.youtube.com/watch?v=kCL2d7wZjU8&t=507s&ab_channel=%E7%A8%9A%E6%99%96%E5%90%9B
