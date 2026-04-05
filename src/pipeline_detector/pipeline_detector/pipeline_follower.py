import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from mavros_msgs.msg import OverrideRCIn # Mimics joystick signals
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np

class PipelineFollower(Node):
    def __init__(self):
        super().__init__('pipeline_follower')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image,
            '/bluerov2/camera_bottom/image_color',
            self.listener_callback,
            qos
        )

        # Publisher to RC Override (Channels 1-8)
        # Channel 5 = Forward/Back, Channel 4 = Yaw (Rotation)
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.bridge = CvBridge()
        
        self.get_logger().info("Pipeline Follower")

    def listener_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([0, 0, 40]), np.array([180, 50, 120]))

            M = cv2.moments(mask)
            rc_msg = OverrideRCIn()
            # Initialize all 18 channels to 'no change' (value: 0 or 1500)
            rc_msg.channels = [1500] * 18 

            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                width = cv_image.shape[1]
                norm_error = (cx - (width / 2)) / (width / 2)

                # --- RC MAPPING ---
                # 1500 is Neutral. 1100-1900 is the range.
                forward_val = 1650  # Moves front
                steer_val = 1500 + int(norm_error * 200) # Turns

                rc_msg.channels[4] = forward_val # Channel 5 (Index 4)
                rc_msg.channels[3] = steer_val   # Channel 4 (Index 3)
                
                self.rc_pub.publish(rc_msg)
                cv2.circle(cv_image, (cx, int(cv_image.shape[0]/2)), 7, (0, 255, 0), -1)

            cv2.imshow("Follower Debug", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PipelineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
