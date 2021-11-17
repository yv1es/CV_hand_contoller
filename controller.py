import mediapipe as mp
import cv2 
import numpy as np 
from matplotlib import pyplot as plt
import serial


# serial port where tha arduino is connected the port may change!
port = 'COM8'

# create a serial object 
arduino = serial.Serial(port=port, baudrate=9600, timeout=0.01)


#
#   This function takes an list containing the angles of all the fingers 
#   and sends it to the arduino via serial 
#
def set_angles(angles):
   
    # declare the message string 
    msg = ''
    
    # add 0 padding to the angles with less than 3 digits 
    for angle in angles:
        a = str(angle)
        while len(a) < 3:
            a = '0' + a
        msg += a
    
    # add start and end symbol
    msg = '<' + msg + '>'

    # write the message string to serial 
    print("Sending: ", msg)    
    for c in msg:
        arduino.write(bytes(c, 'utf-8'))
    
    # read debug data from serial 
    data = arduino.readline()
    print("Receiving: ", data)


#
#   take a value from one range and maps it into a new range
#
def translate(value, leftMin, leftMax, rightMin, rightMax):

    # Figure out how the size of each range
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Normalize the value 
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Scale the value 
    return rightMin + (valueScaled * rightSpan)

#
#   This function computes the angles of each finger and renders them on to the image
#
def compute_finger_angles(image, results, joint_list):

    angles = []

    for hand in results.multi_hand_landmarks:
        for i, joint in enumerate(joint_list):
            a = np.array([hand.landmark[joint[0]].x, hand.landmark[joint[0]].y])
            b = np.array([hand.landmark[joint[1]].x, hand.landmark[joint[1]].y])
            c = np.array([hand.landmark[joint[2]].x, hand.landmark[joint[2]].y])

            rad = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
            angle = np.abs(rad*180.0/np.pi)

            if angle > 180:
                angle = 360 - angle
            
            if i == 0:
                angle = np.interp(angle,[90,180],[0, 200])
                angle = min(180, angle)
            else:
                angle = np.interp(angle,[30,180],[0, 180])
                angle = min(180, angle)

            angles.append(int(angle))
            cv2.putText(image, str(round(angle, 2)), tuple(np.multiply(b, [640, 480]).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 30, 30), 2, cv2.LINE_AA)
    return image, angles




# Setup mediapipe
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

# Setup webcam feed
cap = cv2.VideoCapture(0)

# uncomment this to record the webcam feed 
#fourcc = cv2.VideoWriter_fourcc(*'XVID')
#out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640,480))

# define wich landmarks should be considered for the fingers angles 
joint_list = [ [4, 3, 2], [7, 6, 5], [11, 10, 9], [15, 14, 13], [19, 18, 17]]


with mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.5, max_num_hands=1) as hands:
    while cap.isOpened():

        # capture webcam image and convert it from BGR to RGB
        ret, frame = cap.read()
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Flip the image horizontally
        image = cv2.flip(image, 1)
        
        # detect the hand with MediaPipe
        image.flags.writeable = False
        results = hands.process(image)
        image.flags.writeable = True

        # render detections
        if results.multi_hand_landmarks:
            for num, hand in enumerate(results.multi_hand_landmarks):
                
                # render the detected landmarks 
                mp_drawing.draw_landmarks(image, hand, mp_hands.HAND_CONNECTIONS, 
                                            mp_drawing.DrawingSpec(color=(0, 0, 155), thickness=2, circle_radius=4),
                                            mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2))

            # compute the angles and send them to the Aruino
            image, angles = compute_finger_angles(image, results, joint_list)
            set_angles(angles)
        
        # convert the image back to RGB
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imshow('Hand Tracking', image)

        # uncomment this to record the webcam feed 
        #out.write(image)

        # end the loop if <q> is pressed
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break




# close webcam feed
cap.release()
#uncomment this to record the webcam feed
#out.release()
cv2.destroyAllWindows()