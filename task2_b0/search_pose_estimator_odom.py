import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleOdometry
import math
import numpy as np
import cv2  # Make sure OpenCV is installed

class PoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')

        self.declare_parameter('fov_deg', 57.0)
        self.declare_parameter('fov_deg_h', 87.0)
        self.fov = math.radians(self.get_parameter('fov_deg').value)
        self.fov_h = math.radians(self.get_parameter('fov_deg_h').value)

        self.active = False
        self.initial_yaw = None
        self.current_yaw = None
        self.current_pitch = None
        self.height = None
        self.global_x = 0.0
        self.global_y = 0.0
        self.global_z = 0.0
        self.collected_positions = []
        self.max_flir_detections = 0

        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.subscription_flag = self.create_subscription(
            String,
            '/detector_flag',
            self.flag_callback,
            10)

        self.subscription_odometry = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            reliable_qos)

        self.subscription_centroids = self.create_subscription(
            String,
            '/detection_result',
            self.centroid_callback,
            10)

        self.publisher_pose = self.create_publisher(String, '/pose_targets', 10)

    def flag_callback(self, msg):
        if msg.data == 'TASK2B_0':
            self.active = True
            self.collected_positions.clear()
            self.max_flir_detections = 0

            if self.current_yaw is not None:
                self.initial_yaw = self.current_yaw
                self.get_logger().info(f'Initial yaw set to {self.initial_yaw:.2f} degrees.')
            else:
                self.initial_yaw = None
                self.get_logger().warn('Initial yaw not set. Waiting for odometry...')

            self.get_logger().info('Pose estimation started.')

        elif msg.data == 'OFF':
            self.active = False

            if len(self.collected_positions) >= 2:
                try:
                    pts = np.array(self.collected_positions, dtype=np.float32)
                    rect = cv2.minAreaRect(pts)
                    box = cv2.boxPoints(rect)

                    final_msg = ','.join([f"{pt[0]:.3f},{pt[1]:.3f}" for pt in box])
                    self.publisher_pose.publish(String(data=final_msg))
                    self.get_logger().info('Rectangle corners published.')
                except Exception as e:
                    self.get_logger().error(f'Failed to compute rectangle: {e}')
            else:
                self.get_logger().warn('Not enough points to form a rectangle.')

    def odometry_callback(self, msg):
        qw, qx, qy, qz = msg.q  # PX4: [w, x, y, z]

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        self.current_yaw = math.degrees(yaw)
        self.current_pitch = math.degrees(pitch)

        self.global_x = msg.position[0]
        self.global_y = msg.position[1]
        self.global_z = msg.position[2]
        self.height = -self.global_z

    def centroid_callback(self, msg):
        if not self.active or self.height is None or self.current_pitch is None or self.current_yaw is None or self.initial_yaw is None:
            return

        parts = [p.strip() for p in msg.data.split(',')]
        if len(parts) % 4 != 0:
            return

        current_flir_count = 0

        for i in range(0, len(parts), 4):
            camera_name, cy_str, cx_str, class_id = parts[i:i+4]
            try:
                cx = float(cx_str)
                cy = float(cy_str)
            except ValueError:
                continue

            if camera_name == 'down':
                current_flir_count += 1

            elif camera_name == 'forward' and 0.4 <= cx <= 0.6:
                angle_deg = 45 + self.current_pitch + (0.5 - cy) * math.degrees(self.fov)
                angle_rad = math.radians(90 - angle_deg)

                dx = self.height / math.tan(angle_rad)

                y_angle_rad = (cx - 0.5) * self.fov_h
                slant_range = math.sqrt(self.height**2 + dx**2)
                dy = math.tan(y_angle_rad) * slant_range

                x_local = dx
                y_local = dy

                yaw_rad = math.radians(self.current_yaw)
                x_world = self.global_x + x_local * math.cos(yaw_rad) - y_local * math.sin(yaw_rad)
                y_world = self.global_y + x_local * math.sin(yaw_rad) + y_local * math.cos(yaw_rad)

                self.collected_positions.append((x_world, y_world))

        if current_flir_count > self.max_flir_detections:
            self.max_flir_detections = current_flir_count


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
