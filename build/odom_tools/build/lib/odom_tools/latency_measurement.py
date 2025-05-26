import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time
import csv
from datetime import datetime

class SensorLatencyMonitor(Node):
    def __init__(self):
        super().__init__('sensor_latency_monitor_with_plot_csv')

        # QoS 설정: 라이다는 BestEffort로 설정
        qos_profile_lidar = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # 구독자 생성
        self.create_subscription(CompressedImage, '/camera/image/compressed', self.image_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile_lidar)

        # 지연 시간 저장용 버퍼
        self.image_latencies = deque(maxlen=300)
        self.lidar_latencies = deque(maxlen=300)
        self.lock = threading.Lock()

        # CSV 파일 열기
        self.csv_file = open('latency_log.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'image_avg_latency_ms', 'lidar_avg_latency_ms'])

        # 5초마다 통계 출력 및 로그 저장
        self.create_timer(5.0, self.print_and_log_statistics)

    def compute_latency_ms(self, header_stamp):
        now = self.get_clock().now()
        msg_time = Time.from_msg(header_stamp)
        latency = now - msg_time
        return latency.nanoseconds / 1e6  # ms

    def image_callback(self, msg):
        latency = self.compute_latency_ms(msg.header.stamp)
        with self.lock:
            self.image_latencies.append(latency)

    def lidar_callback(self, msg):
        latency = self.compute_latency_ms(msg.header.stamp)
        with self.lock:
            self.lidar_latencies.append(latency)

    def print_and_log_statistics(self):
        with self.lock:
            image_avg = sum(self.image_latencies) / len(self.image_latencies) if self.image_latencies else 0.0
            image_max = max(self.image_latencies) if self.image_latencies else 0.0
            image_min = min(self.image_latencies) if self.image_latencies else 0.0

            lidar_avg = sum(self.lidar_latencies) / len(self.lidar_latencies) if self.lidar_latencies else 0.0
            lidar_max = max(self.lidar_latencies) if self.lidar_latencies else 0.0
            lidar_min = min(self.lidar_latencies) if self.lidar_latencies else 0.0

            self.get_logger().info(f'[Image] Avg: {image_avg:.2f} ms | Max: {image_max:.2f} ms | Min: {image_min:.2f} ms')
            self.get_logger().info(f'[Lidar] Avg: {lidar_avg:.2f} ms | Max: {lidar_max:.2f} ms | Min: {lidar_min:.2f} ms')

            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            self.csv_writer.writerow([timestamp, f'{image_avg:.2f}', f'{lidar_avg:.2f}'])

def run_plot(node):
    fig, ax = plt.subplots()
    ax.set_title("Sensor Latency (ms)")
    ax.set_xlabel("Samples")
    ax.set_ylabel("Latency (ms)")

    line1, = ax.plot([], [], label='Image Latency')
    line2, = ax.plot([], [], label='Lidar Latency')
    ax.legend()
    ax.set_ylim(0, 500)

    def update_plot(frame):
        with node.lock:
            line1.set_data(range(len(node.image_latencies)), list(node.image_latencies))
            line2.set_data(range(len(node.lidar_latencies)), list(node.lidar_latencies))
        ax.set_xlim(0, max(50, len(node.image_latencies)))
        return line1, line2

    ani = animation.FuncAnimation(fig, update_plot, interval=500)
    plt.tight_layout()
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = SensorLatencyMonitor()

    # 플롯은 메인 스레드에서 실행
    try:
        # ROS 2 노드는 스레드에서 실행
        ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        ros_thread.start()

        # 메인 스레드에서 matplotlib 실행
        run_plot(node)
    finally:
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
