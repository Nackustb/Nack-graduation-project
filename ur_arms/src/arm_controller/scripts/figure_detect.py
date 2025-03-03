#! /usr/bin/env python3.8
# -*- coding: utf-8 -*-

import cv2
import mediapipe as mp
import time

class FigureDetect():

    def __init__(self):
        self.pTime = 0
        self.cap = cv2.VideoCapture(1)
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.5)
        self.mpDraw = mp.solutions.drawing_utils
        self.handLmsStyle = self.mpDraw.DrawingSpec(color=(0, 0, 255), thickness=3)
        self.handConStyle = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=5)

    def show_fps(self):    
        cTime = time.time()
        fps = 1/(cTime-self.pTime)
        self.pTime = cTime
        cv2.putText(self.img, f"FPS : {int(fps)}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

    def run(self):

        while True:
            self.success, self.img = self.cap.read()
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
                            # print(i,xPos,yPos)
                            cv2.putText(self.img, str(i), (xPos-25, yPos+5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2)

                            # if i == 4:
                            #     cv2.circle(img,(xPos,yPos),20,(166,66,66),cv2.FILLED)

                self.show_fps()
                cv2.imshow('img', self.img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
if __name__ == "__main__":
    try:
        node = FigureDetect()
        node.run()

    except KeyboardInterrupt:
        node.cap.release()
        cv2.destroyAllWindows()
