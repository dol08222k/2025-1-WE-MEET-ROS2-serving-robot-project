import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformListener
import math

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def odom_callback(self, msg):
        try:
            # odom → map 변환을 현재 시점으로 요청
            transform = self.tf_buffer.lookup_transform(
                'map',  # target_frame
                'odom',  # source_frame
                rclpy.time.Time()
            )

            # 현재 odom 상 위치 추출
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            # odom → map 변환 적용
            map_x = x + transform.transform.translation.x
            map_y = y + transform.transform.translation.y

            # 방향도 그대로 표시
            orientation_q = msg.pose.pose.orientation
            orientation_list = [
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w
            ]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            yaw_deg = math.degrees(yaw)

            self.get_logger().info(
                f'[MAP FRAME] 위치 x: {map_x:.2f}, y: {map_y:.2f}, θ: {yaw_deg:.1f}°'
            )

        except Exception as e:
            self.get_logger().warn(f'TF 변환 실패: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
