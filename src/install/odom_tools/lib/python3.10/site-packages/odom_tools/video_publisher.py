import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # 압축 이미지 퍼블리셔 설정
        self.publisher_ = self.create_publisher(
            CompressedImage, '/camera/image/compressed', 10)

        # 카메라 열기 (0번 장치, 필요시 /dev/videoX 로 변경)
        self.cap = cv2.VideoCapture(0)

        # 해상도 설정 (320x240)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
        else:
            self.get_logger().info('Camera opened successfully')

        # 타이머: 20 FPS
        self.timer = self.create_timer(0.1, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        # JPEG 압축
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            self.get_logger().warn('Failed to encode frame')
            return

        # CompressedImage 메시지 생성
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = buffer.tobytes()

        self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Camera Publisher stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
