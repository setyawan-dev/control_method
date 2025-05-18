#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
import casadi as ca
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from control_method.msg import koor

class MPCController:
    def __init__(self, servx=0.0, servy=0.0):
        self.servx = servx
        self.servy = servy

    def mpc(self, koorx, koory):
        if koorx is None or koory is None:
            return 0.0, 0.0
        horizon = 10
        dt = 0.05
        u = ca.MX.sym('u', horizon * 2)
        x = self.servx
        y = self.servy
        error = 0
        alpha = 0.2
        weight = 10         # bobot error lebih besar
        for r in range(horizon):
            x += u[2 * r] * dt
            y += u[2 * r + 1] * dt
            error += weight * ((koorx - x) ** 2 + (koory - y) ** 2)
            if r > 0:
                error += alpha * ((u[2*r] - u[2*r-2])**2 + (u[2*r+1] - u[2*r-1])**2)
        nlp = {'x': u, 'f': error}
        solver = ca.nlpsol('solver', 'ipopt', nlp)
        lbx = [-10.0] * (horizon * 2)
        ubx = [10.0] * (horizon * 2)
        x0 = [0.0] * (horizon * 2)
        result = solver(x0=x0, lbx=lbx, ubx=ubx)
        optimal_u = result['x'].full().flatten()
        self.servx += optimal_u[0] * dt
        self.servy += optimal_u[1] * dt
        print(f"Optimal control input: {optimal_u[:2]}, Updated servo pos: ({self.servx:.2f}, {self.servy:.2f})")
        self.servo_x_pos = optimal_u[0]
        self.servo_y_pos = optimal_u[1]
        print(f"Optimal control input: {optimal_u[:2]}, Updated servo pos: ({self.servo_x_pos}, {self.servo_y_pos})")
        return self.servx, self.servy



class ColorTracker:
    def __init__(self):
        self.bridge = CvBridge()
        self.controller = MPCController()
        self.pub_x = rospy.Publisher('/servo_x', Int16, queue_size=10)
        self.pub_y = rospy.Publisher('/servo_y', Int16, queue_size=10)
        self.koorx = None
        self.koory = None

    def data(self, msg):
        self.koorx = msg.merahx
        self.koory = msg.merahy

    def callback(self, msg):
        servo_x = 90
        servo_y = 90
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.koorx is not None and self.koory is not None:
            servo_x, servo_y = self.controller.mpc(self.koorx, self.koory)
            servo_x = max(0, min(180, int(servo_x)))
            servo_y = max(0, min(180, int(servo_y)))
            if abs(servo_x - 90) < 1 and abs(servo_y - 90) < 1:
                servo_x = 90
                servo_y = 90
            if servo_x == 0 and servo_y == 0:
                servo_x = 90
                servo_y = 90
            self.pub_x.publish(int(servo_x))
            self.pub_y.publish(int(servo_y))

    def start_tracking(self):
        rospy.init_node('color_tracker', anonymous=True)
        rospy.Subscriber("/kamera1/image_raw", Image, self.callback)
        rospy.Subscriber("koordinat/kamera1", koor, self.data)
        rospy.spin()


if __name__ == "__main__":
    tracker = ColorTracker()
    tracker.start_tracking()
