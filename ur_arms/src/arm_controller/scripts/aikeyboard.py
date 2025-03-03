#! /usr/bin/env python3.8
# -*- coding: utf-8 -*-

import cv2
import mediapipe as mp
import time

class Keybutton():
    
    def __init__(self):
        self.key_button_list = [["q", "w", "e", "r", "t", "y", "u", "i", "o", "p"],
                                ["a", "s", "d", "f", "g", "h", "j", "k", "l", ";"],
                                ["z", "x", "c", "v", "b", "n", "m", ",", ".", "/"]]
        self.initial_x = 100
        self.initial_y = 100
        self.button_height = 60
        self.button_width = 60
        self.button_pos = [[(self.initial_x * j + 5, self.initial_y * i + 5) for j in range(len(self.key_button_list[0]))] for i in range(len(self.key_button_list))] 

    def show_button(self, img):

        for i in range(len(self.key_button_list)):
            for j in range(len(self.key_button_list[i])):
                cv2.rectangle(img, (self.initial_x*j+5, self.initial_y*i+5), (self.initial_x*j+self.button_width+5, self.initial_y*i+self.button_height+5), (255, 0, 55), cv2.FILLED)
                cv2.putText(img, str(self.key_button_list[i][j]),(self.initial_x*j+15,self.initial_y*i+50),
                            cv2.FONT_HERSHEY_PLAIN,4,(255,255,255),4)
                self.button_pos[i][j] = [self.initial_x*j+5,self.initial_y*i+5]

class SimpleAiKeyboard():

    def __init__(self):
        self.key_button = Keybutton()

        self.pTime = 0
        self.cap = cv2.VideoCapture(1)
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(min_detection_confidence=0.85, min_tracking_confidence=0.1)
        self.mpDraw = mp.solutions.drawing_utils
        self.handLmsStyle = self.mpDraw.DrawingSpec(color=(0, 0, 255), thickness=3)
        self.handConStyle = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=5)

        self.figure_pos = [(0, i) for i in range(21)]

    def show_fps(self):    
        cTime = time.time()
        fps = 1/(cTime-self.pTime)
        self.pTime = cTime
        cv2.putText(self.img, f"FPS : {int(fps)}", (1100, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3) #1100

    def figure_detect(self):
        self.success, self.img = self.cap.read()
        self.img = cv2.flip(self.img,1)
        if self.success:
            imgRGB = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
            result = self.hands.process(imgRGB)

            imgHeight = self.img.shape[0]
            imgWidth = self.img.shape[1]

            if result.multi_hand_landmarks:
                for handLms in result.multi_hand_landmarks:
                    self.mpDraw.draw_landmarks(self.img, handLms, self.mpHands.HAND_CONNECTIONS, self.handLmsStyle, self.handConStyle)
                    for i, lm in enumerate(handLms.landmark):
                        xPos = int(lm.x * imgWidth)
                        yPos = int(lm.y * imgHeight)
                        cv2.putText(self.img, str(i), (xPos-25, yPos+5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2)
                        self.figure_pos[i] = (xPos,yPos)

            
            self.key_button.show_button(self.img)
            self.show_fps()
            cv2.imshow('img', self.img)
    
    def press_key(self):
        while True:
            self.figure_detect()
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

if __name__ == "__main__":
    try:
        node = SimpleAiKeyboard()
        node.press_key()

    except KeyboardInterrupt:
        node.cap.release()
        cv2.destroyAllWindows()
