#!/usr/bin/env python3

import rospy
import time
import cv2
import os
import numpy as np
import imutils
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from control_method.msg import koor

class Vision:
    def __init__(self):
        rospy.loginfo_once("inisiasi")
        rospy.init_node('kamera_1', anonymous=False)
        self.kirim_kamera1 = rospy.Publisher('/kamera1/image_raw', Image, queue_size=10)
        self.koor_kamera1 = rospy.Publisher('/koordinat/kamera1', koor, queue_size=10)

        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.img_cb)
        self.hsvset = 'SETTINGAN HSV & FRAME'
        self.dir = "~/Documents/saved"
        self.bridge = CvBridge()
        self.move = koor()
        self.fps = time.time()
        self.frame = None

    def img_cb(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            rospy.loginfo_once("Launch Cam 1")
            self.frame = cv2.resize(self.frame, (320, 240))
            if self.frame is not None:
                rospy.loginfo_once("connect 1")
                self.camera = True
            else:
                rospy.logwarn("Tidak ada data gambar.")
        except CvBridgeError as e:
            print(e)
    
        # Konfigurasi HSV
        rh = 0
        rs = 118
        rv = 175
        th = 33
        ts = 255
        tv = 255
        
        # new hsv
        # rh = 10
        # rs = 30
        # rv = 30
        # th = 30
        # ts = 255
        # tv = 255

        detek = None
        tolerance = 50
        blurred_frame = cv2.GaussianBlur(self.frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), "uint8")
        low = np.array([rh, rs, rv])
        high = np.array([th, ts, tv])
        mask = cv2.inRange(hsv, low, high)
        erode = cv2.erode(mask, kernel, iterations=1)
        dilate = cv2.dilate(erode, kernel, iterations=3)
        cnts = cv2.findContours(dilate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        mov = self.move

        for c in cnts:
            M = cv2.moments(c)
            if M["m00"] > 1000:
                ((a, b), radius) = cv2.minEnclosingCircle(c)
                x = int(M["m10"] / M["m00"])
                y = int(M["m01"] / M["m00"])
                if detek and abs(x - detek[0]) < tolerance and abs(y - detek[1]) < tolerance:
                    continue
                else:
                    cv2.circle(self.frame, (x, y), 3, (0, 0, 0), -1)
                    cv2.circle(self.frame, (int(a), int(b)), int(radius), (0, 255, 255), 2)
                    detek = (x, y)
                kx = a - 160
                ky = b - 120
                mov.posx = round(kx)
                mov.posy = round(ky)
                
        fps_time = time.time()
        fps = 1.0 / (fps_time - self.fps)
        self.fps = fps_time

        cv2.line(self.frame, (160, 0), (160, 240), (255, 255, 255), 1)
        cv2.line(self.frame, (0, 120), (320, 120), (255, 255, 255), 1)
        cv2.putText(self.frame, f'FPS: {fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("Kamera Utama", self.frame)
        cv2.waitKey(3)
        try:
            self.kirim_kamera1.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))
            self.koor_kamera1.publish(mov)
        except Exception as e:
            rospy.logerr('ERROR 1')
            time.sleep(3)
            rospy.logerr(e)

#--------------- MAIN LOOP ------------------------------
def main():
    hsv= Vision()
    # hsv.trackbars()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("Shutting down")
