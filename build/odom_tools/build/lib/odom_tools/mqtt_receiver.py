import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

class MQTTReceiver(Node):
    def __init__(self):
        super().__init__('mqtt_goal_receiver')

        # 발행 테이블 토픽
        self.table_num_pub = self.create_publisher(String, '/status', 10)

        # MQTT 설정
        self.mqtt_broker = "localhost"
        self.mqtt_port = 1883
        self.mqtt_topic = "/table"

        self.valid_tables = {"table1", "table2", "table3", "table4", "table5"}

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('MQTT broker 연결 성공')
            client.subscribe(self.mqtt_topic)
        else:
            self.get_logger().error(f'MQTT broker 연결 실패, return code : {rc}')

    def on_message(self, client, userdata, msg):
        message = msg.payload.decode()
        self.get_logger().info(f"MQTT message: {message}")

        if message in self.valid_tables:
            table_msg = String()
            table_msg.data = message
            self.table_num_pub.publish(table_msg)
            self.get_logger().info(f"{message} topic 발행됨")
        else:
            self.get_logger().warn(f"Unknown table command received: {message}")


def main(args=None):
    rclpy.init(args=args)
    node = MQTTReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()