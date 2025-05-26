import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler
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

        self.timer = self.create_timer(2.0, self.initialpose_callback)

        self.no_object_count = 0  # 🔹 no object 연속 감지 횟수 카운터

        self.table_targets = {
            "return": (0.00, 0.00, 0.0),
            "table1": (1.30, 0.97, 90.0),   #주문테이블 함수
            "table2": (1.25, -1.00, 90.0),
            "table3": (1.75, 0.97, -90.0),
            "table4": (1.75, -1.15, -90.0),
            "table5": (2.50, -0.09, -90.0)
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
        self.timer.cancel()

    def publish_status(self):
        msg = String()
        msg.data = self.status
        self.publisher_status.publish(msg)
        self.get_logger().info(f" 상태 전송: {self.status}")

    def status_callback(self, msg):
        new_status = msg.data
        self.get_logger().info(f"상태 메시지 수신: {new_status}")

        valid_commands = {
            "table1", "table2", "table3", "table4", "table5",
            "serving_table1", "serving_table2", "serving_table3",
            "serving_table4", "serving_table5", "return"
        }

        if new_status not in valid_commands:
            self.get_logger().warn(f"무시됨: 유효하지 않은 명령 또는 상태 메시지 [{new_status}]")
            return

        if self.status == "moving_home":
            if self.command_queue:
                next_cmd = self.command_queue.popleft()
                self.get_logger().info(f"moving_home 상태 → 다음 명령 실행: {next_cmd}")
                self.execute_command(next_cmd)
            else:
                self.get_logger().info("moving_home 상태 → 명령 없음")
            return

        if new_status == "return":
            self.get_logger().info(f"📥 'return' 명령 수신됨. 현재 상태: {self.status}")
            if self.status in {"table_arrival", "serving_arrival"}:
                if self.command_queue:
                    next_cmd = self.command_queue.popleft()
                    self.get_logger().info(f"→ 다음 명령 실행: {next_cmd}")
                    self.execute_command(next_cmd)
                else:
                    self.get_logger().info("큐 없음 → 초기 위치 복귀")
                    self.execute_command("return")
            else:
                self.get_logger().info(f" 상태 [{self.status}] → return 명령 큐에 저장")
                self.command_queue.append("return")
            return

        if self.status.startswith("serving_table") or self.status == "serving_arrival":
            self.get_logger().info(f"서빙 중 → 명령 [{new_status}] 큐에 저장")
            self.command_queue.append(new_status)
            return

        if self.status in {"init", "table_arrival"}:
            self.get_logger().info(f" 상태 [{self.status}] → 명령 즉시 실행: {new_status}")
            self.execute_command(new_status)
        else:
            self.get_logger().info(f" 상태 [{self.status}] → 명령 큐에 저장: {new_status}")
            self.command_queue.append(new_status)

    def detected_callback(self, msg):
        detected_labels = msg.data

        if self.status != "serving_arrival":
            return

        if detected_labels == "no objects":
            self.no_object_count += 1
            self.get_logger().info(f"❌ 감지 실패 {self.no_object_count}회 연속")

            if self.no_object_count >= 10:
                self.get_logger().info("🔁 no object 10회 감지됨 → 복귀 또는 다음 명령 실행")
                self.no_object_count = 0
                if self.command_queue:
                    next_cmd = self.command_queue.popleft()
                    self.execute_command(next_cmd)
                else:
                    self.execute_command("return")
        else:
            self.get_logger().info("✅ 물체 감지됨 → 카운터 초기화")
            self.no_object_count = 0

    def execute_command(self, cmd):
        if cmd == "return":
            self.get_logger().info("초기 위치로 이동 시작 (return)")
            self.status = "moving_home"
            self.current_target_name = "initial"
            self.publish_status()
            self.send_goal("initial", *self.table_targets["return"])
        elif cmd.startswith("serving_table"):
            self.status = f"moving_{cmd}"
            table_key = cmd.replace("serving_", "")
            self.publish_status()
            self.send_goal(cmd, *self.table_targets[table_key])
        elif cmd in self.table_targets:
            self.status = f"moving_order_{cmd}"
            self.publish_status()
            self.send_goal(cmd, *self.table_targets[cmd])
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

        yaw_rad = math.radians(yaw_deg)
        q = quaternion_from_euler(0, 0, yaw_rad)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.current_target_name = target_name
        self.send_goal_future = self.action_arrival.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        self.get_logger().info(f" 이동 시작 → {target_name} (x: {x}, y: {y}, yaw: {yaw_rad:.2f} rad)")

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
        else:
            self.status = "unknown"

        self.publish_status()

        if self.status == "init":
            self.get_logger().info(" 초기 위치 복귀 완료 (init 상태) ")

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
