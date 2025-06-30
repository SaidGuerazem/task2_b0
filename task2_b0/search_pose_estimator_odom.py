import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleOdometry
import math

class PoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')

        self.declare_parameter('fov_deg', 57.0)
        self.fov = math.radians(self.get_parameter('fov_deg').value)

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
            self.initial_yaw = None
            self.collected_positions.clear()
            self.max_flir_detections = 0
            self.get_logger().info('Pose estimation started.')
        elif msg.data == 'OFF':
            self.active = False
            for _ in range(self.max_flir_detections):
                self.collected_positions.append((self.global_x, self.global_y))
            final_msg = ','.join([f"{x:.3f},{y:.3f}" for x, y in self.collected_positions])
            self.publisher_pose.publish(String(data=final_msg))
            self.get_logger().info('Pose estimation stopped and data published.')

    def odometry_callback(self, msg):
        q = msg.q
        siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
        cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        yaw = math.atan2(siny_cosp, cosy_cosp)

        sinp = 2 * (q[3] * q[1] - q[2] * q[0])
        pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

        self.current_pitch = math.degrees(pitch)
        self.current_yaw = math.degrees(yaw)

        if self.initial_yaw is None:
            self.initial_yaw = self.current_yaw

        self.global_x = msg.position[0]
        self.global_y = msg.position[1]
        self.global_z = msg.position[2]
        self.height = - self.global_z

    def centroid_callback(self, msg):
        if not self.active or self.height is None or self.current_pitch is None or self.current_yaw is None:
            # print(self.active, self.height, self.current_pitch, self.current_yaw)
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
                alpha_deg = 90 - angle_deg
                alpha_rad = math.radians(alpha_deg)

                dx = self.height / math.tan(alpha_rad)

                yaw_diff_rad = math.radians(self.current_yaw - self.initial_yaw)
                rel_x = dx * math.cos(yaw_diff_rad)
                rel_y = dx * math.sin(yaw_diff_rad)
                # print('Relative x.', rel_x)
                # print('Relative y.', rel_y)
                # print('alpha_deg', alpha_deg)
                # print('self.height', self.height)

                self.collected_positions.append((self.global_x + rel_x, self.global_y + rel_y))

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
