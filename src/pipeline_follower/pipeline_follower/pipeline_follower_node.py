#!/usr/bin/env python3

import os
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from custom_msgs.msg import Commands

# ================= PWM =================
PWM_NEUTRAL = 1500
PWM_MIN = 1100
PWM_MAX = 1900

# ================= CONTROL GAINS =================
K_ANGLE = 1.5
K_X = 0.8
FORWARD_PWM = 1600

ANGLE_THRESHOLD = 10        # deg
CENTER_THRESHOLD = 0.1      # normalized

# ================= PIPELINE =================
PIPE_HSV_LOW  = np.array([45, 80, 60], dtype=np.uint8)
PIPE_HSV_HIGH = np.array([90, 255, 255], dtype=np.uint8)

# ================= ARUCO =================
ARUCO_DICT_ID = cv2.aruco.DICT_4X4_100
LOG_PATH = os.path.join(os.path.expanduser('~'), 'markers.txt')


def clamp_pwm(val):
    return int(max(PWM_MIN, min(PWM_MAX, val)))


class PipelineFollowerNode(Node):

    def __init__(self):
        super().__init__('pipeline_follower_node')

        self.declare_parameter('pwm_neutral', PWM_NEUTRAL)
        self.pwm_neutral = self.get_parameter('pwm_neutral').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_img = self.create_subscription(
            Image,
            '/bluerov2/camera_bottom/image_color',
            self.image_callback,
            qos
        )

        self.pub_cmd = self.create_publisher(
            Commands,
            '/master/commands',
            10
        )

        self.pub_debug = self.create_publisher(
            Image,
            '/pipeline_follower/debug_image',
            10
        )

        self.bridge = CvBridge()

        self.detected_ids = set()
        self.log_file = open(LOG_PATH, 'w')

        self.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT_ID)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.get_logger().info("Pipeline follower started")

    # =========================================================
    # MAIN LOOP
    # =========================================================
    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]

        detected, centroid, norm_x, angle, mask = self.detect_pipeline(frame)
        self.detect_aruco(frame)

        # ================= CONTROL =================

        if not detected:
            # SEARCH
            forward = self.pwm_neutral
            lateral = self.pwm_neutral
            yaw     = clamp_pwm(self.pwm_neutral + 80)

        else:
            angle_error = angle
            x_error = norm_x

            # 1. ALIGN
            if abs(angle_error) > ANGLE_THRESHOLD:
                yaw = clamp_pwm(self.pwm_neutral - K_ANGLE * angle_error)
                lateral = self.pwm_neutral
                forward = self.pwm_neutral

            # 2. CENTER
            elif abs(x_error) > CENTER_THRESHOLD:
                yaw = self.pwm_neutral
                lateral = clamp_pwm(self.pwm_neutral - K_X * x_error * 300)
                forward = self.pwm_neutral

            # 3. FOLLOW (continuous correction)
            else:
                yaw = clamp_pwm(self.pwm_neutral - K_ANGLE * angle_error)
                lateral = clamp_pwm(self.pwm_neutral - K_X * x_error * 300)
                forward = FORWARD_PWM

        thrust = self.pwm_neutral

        self.publish_cmd(forward, lateral, yaw, thrust)

        # ================= DEBUG =================
        debug = frame.copy()

        if detected:
            cx, cy = centroid
            cv2.circle(debug, (cx, cy), 8, (0, 0, 255), -1)

            cv2.arrowedLine(debug,
                            (w//2, h//2),
                            (cx, cy),
                            (0, 255, 0), 2)

            cv2.putText(debug, f"yaw_err: {angle:.1f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255,255,255),2)

            cv2.putText(debug, f"strafe_err: {norm_x:.2f}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255,255,255),2)

        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))

    # =========================================================
    # PIPELINE DETECTION (FROM YOUR DETECTOR LOGIC)
    # =========================================================
    def detect_pipeline(self, frame):

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, PIPE_HSV_LOW, PIPE_HSV_HIGH)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return False, None, 0, 0, mask

        largest = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest) < 1000:
            return False, None, 0, 0, mask

        M = cv2.moments(largest)
        if M["m00"] == 0:
            return False, None, 0, 0, mask

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        norm_x = (cx - frame.shape[1] / 2) / (frame.shape[1] / 2)

        [vx, vy, _, _] = cv2.fitLine(largest, cv2.DIST_L2, 0, 0.01, 0.01)
        angle = np.degrees(np.arctan2(float(vy[0]), float(vx[0])))

        return True, (cx, cy), norm_x, angle, mask

    # =========================================================
    # ARUCO DETECTION
    # =========================================================
    def detect_aruco(self, frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is not None:
            for marker_id in ids.flatten():
                marker_id = int(marker_id)

                if marker_id not in self.detected_ids:
                    self.detected_ids.add(marker_id)

                    with open(LOG_PATH, "w") as f:
                        f.write(",".join(map(str, self.detected_ids)))

                    self.get_logger().info(f"Marker detected: {marker_id}")

    # =========================================================
    # COMMAND PUBLISH
    # =========================================================
    def publish_cmd(self, forward, lateral, yaw, thrust):

        cmd = Commands()
        cmd.pitch = self.pwm_neutral
        cmd.roll = self.pwm_neutral
        cmd.yaw = yaw
        cmd.lateral = lateral
        cmd.forward = forward
        cmd.thrust = thrust
        cmd.arm = True
        cmd.mode = "MANUAL"

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PipelineFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
