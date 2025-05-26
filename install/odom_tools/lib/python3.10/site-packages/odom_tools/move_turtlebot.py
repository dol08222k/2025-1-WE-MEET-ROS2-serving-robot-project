import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from std_msgs.msg import String
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler
from tf2_ros import Buffer, TransformListener
from collections import deque
import math

class MoveTurtlebot(Node):
    def __init__(self):
        super().__init__('move_to_table')

        self.publisher_initialpose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.publisher_status = self.create_publisher(String, '/status', 10)
        self.create_subscription(String, '/status', self.status_callback, 10)
        self.create_subscription(String, '/camera/detected_objects', self.detected_callback, 10)

        self.action_arrival = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_target_name = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(2.0, self.initialpose_callback)
        self.no_object_count = 0

        self.table_targets = {
            "return": (0.00, 0.00, 0.0),
            "table1": (1.30, 0.97, 90.0),
            "table2": (1.25, -1.00, 90.0),
            "table3": (1.75, 0.97, -90.0),
            "table4": (1.75, -1.15, -90.0),
            "table5": (2.50, -0.09, -90.0)
        }

        self.cleaning_path = {
            "cleaning1": (1.14, 0.00, 0.0),
            "cleaning2": (1.17, 1.29, 0.0),
            "cleaning3": (1.57, 1.47, 0.0),
            "cleaning4": (1.85, -1.37, 0.0),
            "cleaning5": (1.33, -1.47, 0.0),
            "cleaning6": (1.24, -0.13, 0.0),
            "cleaning7": (2.38, -0.14, 0.0),
            "cleaning8": (2.24, 0.67, 0.0),
            "cleaning9": (0.08, 0.27, 0.0),
            "cleaning10": (0.00, 1.34, 0.0),
            "cleaning11": (1.06, 1.53, 0.0),
            "cleaning12": (2.37, 1.79, 0.0),
            "cleaning13": (2.84, -1.50, 0.0),
            "cleaning14": (0.30, -1.47, 0.0)
        }

        self.status = "init"
        self.command_queue = deque()
        self.publish_status()

    def initialpose_callback(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, 0)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        msg.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]

        self.publisher_initialpose.publish(msg)
        self.get_logger().info('초기 위치 퍼블리시 완료')
        self.status = "init"
        self.publish_status()
        self.timer.cancel()

    def publish_status(self):
        msg = String()
        msg.data = self.status
        self.publisher_status.publish(msg)
        self.get_logger().info(f" 상태 전송: {self.status}")

    def status_callback(self, msg):
        new_status = msg.data
        self.get_logger().info(f"상태 메시지 수신: {new_status}")

        if new_status == "cleaning":
            self.get_logger().info("🧹 청소 경로 시작")
            self.status = "cleaning1"
            self.command_queue = deque(["cleaning2", "cleaning3", "cleaning4", "cleaning5", "cleaning6", "cleaning7", "cleaning8", "cleaning9",
                                        "cleaning10", "cleaning11", "cleaning12", "cleaning13", "cleaning14", "return"])
            self.publish_status()
            self.execute_command("cleaning1")
            return

        valid_commands = {
            "table1", "table2", "table3", "table4", "table5",
            "serving_table1", "serving_table2", "serving_table3",
            "serving_table4", "serving_table5", "return", "cleaning"
        }

        if new_status not in valid_commands:
            self.get_logger().warn(f"무시됨: 유효하지 않은 명령 또는 상태 메시지 [{new_status}]")
            return

        if self.status == "moving_home":
            if self.command_queue:
                next_cmd = self.command_queue.popleft()
                self.execute_command(next_cmd)
            return

        if new_status == "return":
            if self.status in {"table_arrival", "serving_arrival"}:
                if self.command_queue:
                    self.execute_command(self.command_queue.popleft())
                else:
                    self.execute_command("return")
            else:
                self.command_queue.append("return")
            return

        if self.status.startswith("serving_table") or self.status == "serving_arrival":
            self.command_queue.append(new_status)
            return

        if self.status in {"init", "table_arrival"}:
            self.execute_command(new_status)
        else:
            self.command_queue.append(new_status)

    def detected_callback(self, msg):
        if self.status != "serving_arrival":
            return

        if msg.data == "no objects":
            self.no_object_count += 1
            if self.no_object_count >= 10:
                self.no_object_count = 0
                if self.command_queue:
                    self.execute_command(self.command_queue.popleft())
                else:
                    self.execute_command("return")
        else:
            self.no_object_count = 0

    def execute_command(self, cmd):
        if cmd == "return":
            self.status = "moving_home"
            self.current_target_name = "initial"
            self.publish_status()
            self.send_goal("initial", *self.table_targets["return"])
        elif cmd.startswith("serving_table"):
            self.status = f"moving_{cmd}"
            self.publish_status()
            self.send_goal(cmd, *self.table_targets[cmd.replace("serving_", "")])
        elif cmd in self.table_targets:
            self.status = f"moving_order_{cmd}"
            self.publish_status()
            self.send_goal(cmd, *self.table_targets[cmd])
        elif cmd in self.cleaning_path:
            self.status = cmd
            self.publish_status()
            self.send_goal(cmd, *self.cleaning_path[cmd])
        else:
            self.get_logger().warn(f" execute_command: 알 수 없는 명령: {cmd}")

    def send_goal(self, target_name, x, y, yaw_deg):
        if not self.action_arrival.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(' NavigateToPose 액션 서버 연결 실패')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        if target_name.startswith("cleaning"):
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
                q = transform.transform.rotation
                goal_msg.pose.pose.orientation = q
            except Exception as e:
                self.get_logger().warn(f"TF 변환 실패: {e} → yaw 0으로 대체")
                q = quaternion_from_euler(0, 0, 0)
                goal_msg.pose.pose.orientation.x = q[0]
                goal_msg.pose.pose.orientation.y = q[1]
                goal_msg.pose.pose.orientation.z = q[2]
                goal_msg.pose.pose.orientation.w = q[3]
        else:
            yaw_rad = math.radians(yaw_deg)
            q = quaternion_from_euler(0, 0, yaw_rad)
            goal_msg.pose.pose.orientation.x = q[0]
            goal_msg.pose.pose.orientation.y = q[1]
            goal_msg.pose.pose.orientation.z = q[2]
            goal_msg.pose.pose.orientation.w = q[3]

        self.current_target_name = target_name
        self.send_goal_future = self.action_arrival.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(f" 이동 시작 → {target_name} (x: {x}, y: {y})")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(' 좌표 전송 실패')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().status
        if result == 5:
            self.get_logger().warn(f'[{self.current_target_name}] 취소됨')
            if self.current_target_name == "initial":
                self.status = "init"
                self.publish_status()
            return

        self.get_logger().info(f' [{self.current_target_name}] 도착 완료')

        if self.current_target_name == "initial":
            self.status = "init"
        elif self.current_target_name in self.table_targets:
            self.status = "table_arrival"
        elif self.current_target_name.startswith("serving_table"):
            self.status = "serving_arrival"
        elif self.current_target_name in self.cleaning_path:
            if self.command_queue:
                self.execute_command(self.command_queue.popleft())
                return
            else:
                self.status = "init"
        else:
            self.status = "unknown"

        self.publish_status()
        if self.status == "init":
            self.get_logger().info(" 초기 위치 복귀 완료 (init 상태)")

def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtlebot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
