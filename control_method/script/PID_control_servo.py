#!/usr/bin/env python3

import rospy
import cv2
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from control_method.msg import koor

class PID_Controller:
    def __init__(self):
        rospy.init_node('pid', anonymous=True)
        rospy.Subscriber("/kamera1/image_raw", Image, self.callback)
        rospy.Subscriber("koordinat/kamera1", koor, self.data)
        self.pub_x = rospy.Publisher('/servo_x', Int16, queue_size=10)
        self.pub_y = rospy.Publisher('/servo_y', Int16, queue_size=10)
        self.bridge = CvBridge()
        self.koorx = 0.0
        self.koory = 0.0
        self.prev_errorx = 0.0
        self.total_errorx = 0.0
        self.prev_errory = 0.0
        self.total_errory = 0.0
        self.angel_x = 0.0
        self.angel_y = 0.0
        self.filter_x = 90
        self.filter_y = 90

    def smooth(self, prev, current, alpha=0.2):
        return alpha * current + (1 - alpha) * prev

    def data(self, msg):
        self.koorx = msg.posx
        self.koory = msg.posy
        rospy.loginfo(f"Koordinat: x={self.koorx}, y={self.koory}")

    def pid(self, targetx=320, targety=240, Kp=0.1, Ki=0.0001, Kd=0.0010):
        limit = 15

        # PID X
        errorx = targetx - self.koorx
        if abs(errorx) < limit:
            errorx = 0
        Px = Kp * errorx
        self.total_errorx += errorx
        Ix = Ki * self.total_errorx
        Ix = max(min(Ix, 0.05), -0.05)
        Dx = Kd * (errorx - self.prev_errorx)
        self.prev_errorx = errorx
        servo_x = Px + Ix + Dx
        servo_x = max(min(servo_x, 1.0), -1.0)
        self.angel_x = servo_x
        rospy.loginfo(f"PID servo_x: {servo_x}")

        # PID Y
        errory = targety - self.koory
        if abs(errory) < limit:
            errory = 0
        Py = Kp * errory
        self.total_errory += errory
        Iy = Ki * self.total_errory
        Iy = max(min(Iy, 0.05), -0.05)
        Dy = Kd * (errory - self.prev_errory)
        self.prev_errory = errory
        servo_y = Py + Iy + Dy
        servo_y = max(min(servo_y, 1.0), -1.0)
        self.angel_y = servo_y
        rospy.loginfo(f"PID servo_y: {servo_y}")

    def remap(self, pid_output):
        center = 90
        scale = 90
        angle = center + int(pid_output * scale)
        angle = max(0, min(180, angle))
        return angle

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.koorx is not None and self.koory is not None:
            Kp = rospy.get_param("/pid/Kp", 0.005)
            Ki = rospy.get_param("/pid/Ki", 0.0001)
            Kd = rospy.get_param("/pid/Kd", 0.01)
            rospy.loginfo(f"PID VALUES: Kp={Kp}, Ki={Ki}, Kd={Kd}")
            self.pid(Kp=Kp, Ki=Ki, Kd=Kd)

            raw_servox = self.remap(self.angel_x)
            raw_servoy = self.remap(self.angel_y)
            max_delta = 2
            delta_x = raw_servox - self.filter_x
            delta_y = raw_servoy - self.filter_y
            if abs(delta_x) > max_delta:
                delta_x = max_delta if delta_x > 0 else -max_delta
            if abs(delta_y) > max_delta:
                delta_y = max_delta if delta_y > 0 else -max_delta
            self.filter_x += delta_x
            self.filter_y += delta_y
            servo_x = int(max(0, min(180, self.filter_x)))
            servo_y = int(max(0, min(180, self.filter_y)))
            rospy.loginfo(f"Publish servo x: {servo_x}, servo y: {servo_y}")
            print("\n")
            self.pub_x.publish(Int16(servo_x))
            self.pub_y.publish(Int16(servo_y))

if __name__ == "__main__":
    try:
        tracker = PID_Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")
