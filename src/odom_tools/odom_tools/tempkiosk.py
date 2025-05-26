import sys
import webbrowser
import requests
import signal
from functools import partial
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QTextEdit, QSpacerItem, QSizePolicy
)
from PyQt5.QtCore import Qt

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
from collections import deque

class KioskNode(Node):
    def __init__(self):
        super().__init__('kiosk_node')
        self.status_pub = self.create_publisher(String, '/status', 10)
        self.create_subscription(String, '/status', self.status_callback, 10)
        self.status = "init"
        self.current_table = "unknown"

    def status_callback(self, msg):
        self.status = msg.data
        if self.status.startswith("moving_order_table"):
            self.current_table = self.status.replace("moving_order_table", "")
        self.get_logger().info(f"Current status: {self.status}")
        self.kiosk_ui.update_status()

    def send_return_request(self):
        msg = String()
        msg.data = "return"
        self.status_pub.publish(msg)

    def publish_status(self, status_msg):
        msg = String()
        msg.data = status_msg
        self.status_pub.publish(msg)
        self.get_logger().info(f"Published status: {status_msg}")


class KioskUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("서빙 로봇 키오스크")
        self.setGeometry(300, 300, 500, 500)
        self.menu_items = {"메뉴 1": 0, "메뉴 2": 0, "메뉴 3": 0}
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout()

        self.status_label = QLabel("위치 설정 대기중 . . .", self)
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font-size: 30px; font-weight: bold;")
        main_layout.addWidget(self.status_label)

        self.table_buttons_widget = QWidget()
        table_buttons_layout = QHBoxLayout(self.table_buttons_widget)
        self.table_buttons = []
        for i in range(1, 6):
            btn = QPushButton(f"테이블 {i}")
            btn.setMinimumHeight(40)
            btn.clicked.connect(partial(self.handle_serving_button, i))
            table_buttons_layout.addWidget(btn)
            self.table_buttons.append(btn)
        main_layout.addWidget(self.table_buttons_widget)
        self.table_buttons_widget.hide()

        self.menu_and_order_widget = QWidget()
        self.menu_and_order_layout = QHBoxLayout(self.menu_and_order_widget)
        menu_layout = QVBoxLayout()
        for item in self.menu_items.keys():
            row = QHBoxLayout()
            label = QLabel(item)
            plus_btn = QPushButton("+")
            minus_btn = QPushButton("-")
            plus_btn.clicked.connect(partial(self.modify_item, item, 1))
            minus_btn.clicked.connect(partial(self.modify_item, item, -1))
            row.addWidget(label)
            row.addWidget(plus_btn)
            row.addWidget(minus_btn)
            menu_layout.addLayout(row)

        self.order_display = QTextEdit()
        self.order_display.setReadOnly(True)
        self.update_order_display()
        self.order_button = QPushButton(" 주문 완료")
        self.order_button.clicked.connect(self.complete_order)

        right_layout = QVBoxLayout()
        right_layout.addWidget(QLabel("주문 내역"))
        right_layout.addWidget(self.order_display)
        right_layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        right_layout.addWidget(self.order_button, alignment=Qt.AlignRight | Qt.AlignBottom)

        self.menu_and_order_layout.addLayout(menu_layout)
        self.menu_and_order_layout.addLayout(right_layout)
        main_layout.addWidget(self.menu_and_order_widget)
        self.setLayout(main_layout)
        self.menu_and_order_widget.hide()

    def handle_serving_button(self, table_number):
        if self.ros_node.status != "init":
            self.status_label.setText(" 로봇이 대기 중이 아닙니다.")
            return
        self.status_label.setText(f" {table_number}번 테이블로 서빙 시작")
        self.ros_node.publish_status(f"serving_table{table_number}")
        self.disable_table_buttons()

    def disable_table_buttons(self):
        for btn in self.table_buttons:
            btn.setEnabled(False)

    def enable_table_buttons(self):
        for btn in self.table_buttons:
            btn.setEnabled(True)

    def update_status(self):
        status = self.ros_node.status
        if status == "init":
            self.status_label.setText("서빙할 테이블을 선택하세요")
            self.menu_and_order_widget.hide()
            self.table_buttons_widget.show()
            self.enable_table_buttons()
        elif status.startswith("moving_order_table"):
            table_num = status[-1]
            self.status_label.setText(f"{table_num}번 테이블로 주문 받으러 이동 중 . . .")
            self.menu_and_order_widget.hide()
            self.table_buttons_widget.hide()
        elif status == "table_arrival":
            self.status_label.setText("테이블에 도착했습니다. 주문을 선택해 주세요")
            self.menu_and_order_widget.show()
            self.table_buttons_widget.hide()
        elif status == "moving_home":
            self.status_label.setText("초기 위치로 복귀 중 . . .")
            self.menu_and_order_widget.hide()
            self.table_buttons_widget.hide()
        elif status.startswith("moving_serving_table"):
            table_num = status[-1]
            self.status_label.setText(f"{table_num}번 테이블로 서빙 중 . . .")
            self.menu_and_order_widget.hide()
            self.table_buttons_widget.hide()
        elif status == "serving_arrival":
            self.status_label.setText("주문하신 메뉴를 가져가주세요!")
            self.menu_and_order_widget.hide()
            self.table_buttons_widget.hide()
        else:
            table_num = status[-1]
            self.status_label.setText(f"{table_num}번 테이블로 추가 주문이 들어왔습니다")
            self.menu_and_order_widget.hide()
            self.table_buttons_widget.hide()

    def modify_item(self, item, change):
        self.menu_items[item] = max(0, self.menu_items[item] + change)
        self.update_order_display()

    def update_order_display(self):
        lines = [f"{item}: {count}개" for item, count in self.menu_items.items() if count > 0]
        self.order_display.setText("\n".join(lines) if lines else "선택된 메뉴 없음")

    def complete_order(self):
        order_data = {item: count for item, count in self.menu_items.items() if count > 0}
        if not order_data:
            self.order_display.setText("❗ 메뉴를 선택하세요")
            return

        self.ros_node.send_return_request()
        table = self.ros_node.current_table

        try:
            requests.post('http://localhost:5000/order', json={"table": table, "items": order_data})
        except Exception as e:
            self.order_display.setText(f"웹 전송 실패: {e}")
            return

        for item in self.menu_items:
            self.menu_items[item] = 0
        self.update_order_display()


def main(args=None):
    rclpy.init(args=args)
    node = KioskNode()
    app = QApplication(sys.argv)

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    ui = KioskUI(node)
    ui.show()
    node.kiosk_ui = ui

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()