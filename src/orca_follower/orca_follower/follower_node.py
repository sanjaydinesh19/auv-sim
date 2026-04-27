import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class OrcaFollower(Node):
    def __init__(self):
        super().__init__('orca_follower')
        
        # ROS 2 Setup
        self.subscription = self.create_subscription(Image, '/bluerov2/camera_bottom/image_color', self.image_callback, 10)
        self.thruster_pub = self.create_publisher(Float64MultiArray, '/bluerov2/thrusters', 10)
        self.bridge = CvBridge()

        # Constraints
        self.MAX_THRUST = 20.0
        
        # Vision Settings (Tuned for the Lime Green pipe in Stonefish)
        self.hsv_lower = np.array([40, 50, 50])
        self.hsv_upper = np.array([90, 255, 255])

        self.get_logger().info("PIPELINE FOLLOWER: Optimized 6-DOF Control Active.")

    def clamp(self, value, limit):
        """Prevents thruster values from exceeding the physical limit."""
        return max(-limit, min(limit, value))

    def image_callback(self, msg):
        # 1. Image Acquisition
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image = cv2.resize(cv_image, (640, 480))
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 2. Pipeline Detection
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        M = cv2.moments(mask)

        # Movement Variables
        surge = 0.0   # Forward/Backward
        sway = 0.0    # Left/Right (Strafe)
        heave = 0.0   # Up/Down (Dive)
        yaw = 0.0     # Rotation (Left/Right)
        pitch = 0.0   # Tipping nose up/down
        roll = 0.0    # Tipping side to side

        if M['m00'] > 500:
            # 3. Control Logic
            cx = int(M['m10'] / M['m00'])
            error = (cx - 320) / 320.0  # -1.0 (Left) to 1.0 (Right)
            
            # --- PERFORMANCE TUNING ---
            surge = 4.5             # Slower forward speed allows for sharper turns
            yaw = error * 16.0      # Increased gain to pull pipe back to center
            heave = 4.0             # Keep vertical pressure to stay near floor
            
            self.get_logger().info(f"TRACKING: Error {error:.2f} | Yaw {yaw:.2f}")

            # Optional: Show a dot on the detected center for debugging
            cv2.circle(cv_image, (cx, 240), 10, (0, 0, 255), -1)
        else:
            # 4. Search Mode (If pipe is lost, sink slowly)
            heave = 6.0
            self.get_logger().warn("PIPELINE LOST: Sinking to search...")

        # 5. The 6-DOF Mapping (Based on your provided logic)
        thruster_values = [
            self.clamp(surge + yaw + sway, self.MAX_THRUST),    # t0
            self.clamp(surge - yaw - sway, self.MAX_THRUST),    # t1
            self.clamp(-surge - yaw + sway, self.MAX_THRUST),   # t2
            self.clamp(-surge + yaw - sway, self.MAX_THRUST),   # t3
            self.clamp(heave + pitch + roll, self.MAX_THRUST),  # t4
            self.clamp(heave + pitch - roll, self.MAX_THRUST),  # t5
            self.clamp(heave - pitch + roll, self.MAX_THRUST),  # t6
            self.clamp(heave - pitch - roll, self.MAX_THRUST)   # t7
        ]

        # 6. Publish to the robot
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(x) for x in thruster_values]
        self.thruster_pub.publish(cmd_msg)

        # Show the processed view (useful for monitoring)
        cv2.imshow("Orca Vision", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = OrcaFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Emergency Thruster Cut-off
        stop_msg = Float64MultiArray()
        stop_msg.data = [0.0] * 8
        node.thruster_pub.publish(stop_msg)
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
