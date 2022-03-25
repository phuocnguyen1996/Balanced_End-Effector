# Balanced End-Effector
This is a the Prototyping Project of Nguyen and Mehmet. It is a small 3-DOF end-effector combined with a 9-axis IMU sensor, which can be attached on any types of robot arms to maintain any orientation while moving or working.

## Hardware
The 3D model is inside the `3D_Printing_Model_Arm_V2` folder with all components. To create the algorithm, we modelled the end-effect as the DH parameters below:
| |theta |d |lamda |L |
|----------- |----------- | ----------- |-----------|-----------|
|1 |q1 |L1 |pi/2|0|
|2 |q2 + pi |0 |pi/2|0|
|3 |q3 + pi|L3 |pi/2|0|
|4 |q4 + pi|0 |0|0|

This project only focused on orientation, so we ignored the value L1 and L3

## Software
We used Arduino as the microcontroller. The code is inside the `Code` folder. We used the library `MPU9250_WE` to read the IMU sensor. 

First we set an initial desired orientation for the end-effector. Then we calculated the orientation deviation indirectly by using the rotational speeds around 3 axes and multiplying them with the sample time. Next, we did the inverse kinematics and finally controlled the motors to rotate to make the orientation back to the set values. The functions for calculating matrices was included in the code file.

Wiring:
- Control wire of Motor1 ----- Pin 10
- Control wire of Motor2 ----- Pin 9
- Control wire of Motor3 ----- Pin 11
- Sensor SCL ----- A5
- Sensor SDA ----- A4 

## Presentation and Video
The video for demonstration is inside the presention. In the video, we tested the end-effector twice: First individually, then with the Kuka 6-DOF arm.

## Disadvantages
The orientation was indirectly computed through the rotational speeds. We did not use all the functions of IMU sensor. The orientation should be computed in a more useful way. The calculation of matrices was also a problem because of it required a long time for multiplying. Another method for computation, like quaternion, to reduce the time 
