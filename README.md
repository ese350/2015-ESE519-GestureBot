# 2015-ESE519-GestureBot

Team Members:  
Aniruddha Rajshekar, 
Aditya Pinapala, 
Lanlan Pang 

Our project aims to design a gesture-based control system which lets a person use the motions of his arm to control the robotic arm.

Our project is done using two mbed MCUs and consists of two parts. First is the master/transmitter system, where we have the IMU sensors located on the arm of user (we used a glove to package our master system). The readings from the IMU sensor determine the direction of the movement of the arm. This signal is processed and sent to the communication module. The transmitter communication module sends corresponding instruction to receiver system. The receiver system recognizes the current gesture and processes the instruction in real-time to drive the robotic arm move according to the human arm gestures.
