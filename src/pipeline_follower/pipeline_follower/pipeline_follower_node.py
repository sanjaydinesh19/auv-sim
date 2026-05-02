#!/usr/bin/env python3

import os
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

MAX_THRUST   = 20.0
HSV_LOW      = np.array([40, 50, 50],   dtype=np.uint8)
HSV_HIGH     = np.array([90, 255, 255], dtype=np.uint8)
ARUCO_DICT_ID = cv2.aruco.DICT_4X4_100
LOG_PATH     = os.path.join(os.path.expanduser('~'), 'aruco_log.txt')
FRAME_W, FRAME_H = 640, 480
FRAME_CX     = FRAME_W // 2
FRAME_CY     = FRAME_H // 2

SURGE_MAX    = 5.0
SURGE_MIN    = 1.5

YAW_KP       = 5.0
YAW_KI       = 0.01
YAW_KD       = 0.8

X_WEIGHT     = 0.6
ANGLE_WEIGHT = 0.4


def clamp(value, limit=MAX_THRUST):
    return max(-limit, min(limit, value))


class PipelineFollowerNode(Node):

    def __init__(self):
        super().__init__('pipeline_follower_node')

        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_img = self.create_subscription(
            Image,
            '/bluerov2/camera_bottom/image_color',
            self.image_callback,
            cam_qos
        )

        self.pub_thrust = self.create_publisher(
            Float64MultiArray,
            '/bluerov2/thrusters',
            10
        )

        self.pub_debug = self.create_publisher(
            Image,
            '/pipeline_follower/debug_image',
            10
        )

        self.bridge       = CvBridge()
        self.detected_ids = set()
        self.log_file     = open(LOG_PATH, 'a')

        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.yaw_integral = 0.0
        self.yaw_prev     = 0.0
        self.prev_time    = self.get_clock().now()

        self.get_logger().info('PipelineFollowerNode started')

    def destroy_node(self):
        self._publish([0.0] * 8)
        self.log_file.close()
        super().destroy_node()

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        frame = cv2.resize(frame, (FRAME_W, FRAME_H))

        now = self.get_clock().now()
        dt  = max((now - self.prev_time).nanoseconds / 1e9, 0.001)
        self.prev_time = now

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))

        aruco_ids = self._detect_aruco(frame)
        M         = cv2.moments(mask)
        pipe_area = M['m00']
        pipe_seen = pipe_area > 50000

        surge = yaw = 0.0
        cx = cy = None
        angle_deg = 0.0
        combined_error = 0.0

        if pipe_seen:
            cx        = int(M['m10'] / pipe_area)
            cy        = int(M['m01'] / pipe_area)
            x_error   = (cx - FRAME_CX) / float(FRAME_CX)
            angle_deg = self._compute_angle_deg(mask)
            angle_norm = angle_deg / 90.0

            combined_error = (X_WEIGHT * x_error) + (ANGLE_WEIGHT * angle_norm)

            self.yaw_integral = max(-5.0, min(5.0,
                                self.yaw_integral + combined_error * dt))
            yaw_d             = (combined_error - self.yaw_prev) / dt
            self.yaw_prev     = combined_error

            yaw = clamp(combined_error      * YAW_KP
                        + self.yaw_integral * YAW_KI
                        + yaw_d             * YAW_KD)

            error_magnitude = abs(combined_error)
            surge = max(SURGE_MIN,
                        SURGE_MAX * (1.0 - min(error_magnitude, 1.0)))

            self.get_logger().info(
                f'x_err={x_error:+.3f} ang={angle_deg:+.1f} '
                f'combined={combined_error:+.3f} yaw={yaw:+.2f} surge={surge:.2f}',
                throttle_duration_sec=0.5
            )
        else:
            self.yaw_integral = 0.0
            self.yaw_prev     = 0.0
            self.get_logger().warn('PIPELINE NOT DETECTED',
                                   throttle_duration_sec=2.0)

        self._mix_and_publish(surge, yaw)

        debug = self._draw_debug(frame, mask, cx, cy, angle_deg,
                                  aruco_ids, pipe_area, combined_error, surge)
        try:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            dbg_msg.header = msg.header
            self.pub_debug.publish(dbg_msg)
        except CvBridgeError:
            pass

    def _mix_and_publish(self, surge, yaw):
        t = [
            clamp( surge + yaw),
            clamp(-surge + yaw),
            clamp( surge - yaw),
            clamp(-surge - yaw),
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        self._publish([float(x) for x in t])

    def _publish(self, data):
        msg = Float64MultiArray()
        msg.data = data
        self.pub_thrust.publish(msg)

    @staticmethod
    def _compute_angle_deg(mask):
        pts = cv2.findNonZero(mask)
        if pts is None or len(pts) < 10:
            return 0.0
        pts = pts.reshape(-1, 2).astype(np.float32)
        _, eigvec = cv2.PCACompute(pts, mean=None)
        dx, dy = float(eigvec[0, 0]), float(eigvec[0, 1])
        angle = math.degrees(math.atan2(dx, dy))
        if angle > 90:
            angle -= 180
        elif angle < -90:
            angle += 180
        return angle

    def _detect_aruco(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)
        found = []
        if ids is not None:
            for mid in ids.flatten():
                mid = int(mid)
                found.append(mid)
                if mid not in self.detected_ids:
                    self.detected_ids.add(mid)
                    self.log_file.write(f'ArUco ID {mid}\n')
                    self.log_file.flush()
                    self.get_logger().info(f'New ArUco marker: {mid}')
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        return found

    def _draw_debug(self, frame, mask, cx, cy, angle_deg,
                    aruco_ids, pipe_area, combined_error, surge):
        debug = frame.copy()
        green = np.zeros_like(debug)
        green[:, :, 1] = mask
        debug = cv2.addWeighted(debug, 1.0, green, 0.35, 0)

        cv2.drawMarker(debug, (FRAME_CX, FRAME_CY),
                       (255, 0, 0), cv2.MARKER_CROSS, 24, 2)

        if cx is not None:
            cv2.circle(debug, (cx, cy), 10, (0, 0, 255), -1)
            cv2.line(debug, (FRAME_CX, FRAME_CY), (cx, cy), (0, 255, 255), 2)

            length    = 60
            angle_rad = math.radians(angle_deg)
            adx = int(length * math.sin(angle_rad))
            ady = int(length * math.cos(angle_rad))
            cv2.arrowedLine(debug,
                            (cx - adx, cy - ady), (cx + adx, cy + ady),
                            (0, 140, 255), 2, tipLength=0.3)

            err_pct = min(abs(combined_error), 1.0)
            col     = (0, int(255 * (1 - err_pct)), int(255 * err_pct))
            cv2.putText(debug, f'surge={surge:.1f}  err={combined_error:+.3f}',
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)

            x_error = (cx - FRAME_CX) / float(FRAME_CX)
            cv2.putText(debug,
                        f'x_err={x_error:+.3f}  ang={angle_deg:+.1f}  area={int(pipe_area)}',
                        (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        else:
            cv2.putText(debug, 'PIPELINE NOT DETECTED',
                        (100, FRAME_CY), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2)

        if aruco_ids:
            cv2.putText(debug, f'ArUco: {aruco_ids}',
                        (10, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return debug


def main(args=None):
    rclpy.init(args=args)
    node = PipelineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Float64MultiArray()
        stop.data = [0.0] * 8
        node.pub_thrust.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
