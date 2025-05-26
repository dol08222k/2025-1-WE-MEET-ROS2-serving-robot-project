import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler
import math

class MoveTurtlebot(Node):
    def __init__(self):
        super().__init__('move_to_table')

        # 부팅 시 초기 위치 설정
        self.publisher_initialpose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        timer_period = 2.0  # 2초 후 퍼블리시
        self.timer = self.create_timer(timer_period, self.initialpose_callback)

        self.action_arrival = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_target_name = None

        self.create_subscription(String, '/camera/detected_objects', self.detected_callback, 10)

        # status
        self.publisher_status = self.create_publisher(String, '/status', 10)
        self.create_subscription(String, '/status', self.status_callback, 10)

        # 각 테이블 좌표
        self.table_targets = {
            "return": (0.00, 0.00, 0.0),
            "table1": (1.32, 0.97, 90.0),
            "table2": (1.25, -1.00, 90.0), #(1.25, -1.00, 90.0),
            "table3": (1.75, 1.57, -90.0),
            "table4": (1.80, -1.00, -90.0),
            "table5": (2.50, -0.09, -90.0)
        }

        self.status = "init"    
        self.publish_status()
    
    def initialpose_callback(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        # 초기 위치 설정 (예: x=0.0, y=0.0, yaw=0도)
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, 0)  # yaw=0
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # Covariance(불확실성) 설정 - 낮은 값이면 확신이 높음
        msg.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]

        self.publisher_initialpose.publish(msg)
        self.get_logger().info('Published initial pose automatically')
        self.timer.cancel()  # 한 번 퍼블리시하고 타이머 끄기

    def publish_status(self):
        msg = String()
        msg.data = self.status
        self.publisher_status.publish(msg)

    def status_callback(self, msg):
        if msg.data == "return":
            self.status = "moving_home"
            self.publish_status()
            self.get_logger().info(f"키오스크 주문 완료, 초기 위치로 복귀")
            self.send_goal('initial', *self.table_targets[msg.data])  # 초기 위치로 복귀

        elif msg.data.startswith("serving_table"):
            self.status = msg.data
            table_key = msg.data.replace("serving_", "")
            
            self.get_logger().info(f"[{table_key}]로 서빙 요청")

            self.send_goal(msg.data, *self.table_targets[table_key])

        elif msg.data in self.table_targets:
            self.status = f"moving_{msg.data}" # moving_table1 처럼 형식
            self.publish_status()
            self.send_goal(msg.data, *self.table_targets[msg.data])
            self.get_logger().info(f"[{msg.data}]로 주문 요청")

    def detected_callback(self, msg):
        detected_labels = msg.data

        if self.status == "serving_arrival" and detected_labels == "no objects":
            self.get_logger().info("서빙 arrival 상태에서 감지된 객체가 없습니다. 초기 위치로 복귀 요청")
            
            # return 메시지를 /status로 publish
            return_msg = String()
            return_msg.data = "return"
            self.publisher_status.publish(return_msg)

    def send_goal(self, target_name, x, y, yaw_deg):
        if not self.action_arrival.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose 액션 서버 연결 실패')
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

        self.current_target_name = target_name  # 현재 목표 이름 저장

        # 액션 서버로 goal을 전송
        self.send_goal_future = self.action_arrival.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(f"[{target_name}]으로 이동 x: {x}, y: {y} yaw : {yaw_rad}")

    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('좌표 전송 실패')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().status

        if result == 5:  # Canceled
            self.get_logger().warn(f'[{self.current_target_name}] 취소됨')
        else:
            self.get_logger().info(f'[{self.current_target_name}] 도착 완료')

            if self.current_target_name == "initial":
                self.status = "init"
                self.publish_status()
            elif self.current_target_name in {"table1", "table2", "table3", "table4", "table5"}:
                self.status = "table_arrival"
                self.publish_status()
            elif self.current_target_name in {"serving_table1", "serving_table2", "serving_table3", "serving_table4", "serving_table5"}:
                self.status = "serving_arrival"
                self.publish_status()
            else:
                self.status = "unknown"
                self.publish_status()


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