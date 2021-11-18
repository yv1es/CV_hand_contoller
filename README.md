# CV_hand_controller
This repository includes one Python and one Arduino Script. These scripts controll a robotic hand (Demo video: https://youtu.be/K7vnqhRqCAE)

## controller.py
controller.py uses Googles MediaPipe ML solution(https://mediapipe.dev) to perform hand pose estimation. 
It than computes the angle of each finger and communitcates these angels to an Arduino via serial. 

## arduino_controll_script.ino
arduino_controll_script.ino is intended to run on an Arduino connected via USB to the Computer running the controller.py script. 
It receives the angles for each finger computed by the controller.py script and drives the servo motors accordingly. 
