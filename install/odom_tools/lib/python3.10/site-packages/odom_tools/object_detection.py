import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2

class YoloImageSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_image_subscriber')
        self.bridge = CvBridge()

        # 퍼블리셔 설정
        self.detected_pub = self.create_publisher(Image, '/camera/detected', 10)
        self.objects_pub = self.create_publisher(String, '/camera/detected_objects', 10)

        # YOLO 모델 로드
        self.model = YOLO('yolov5s.pt')
        self.get_logger().info('YOLOv5 Node initialized')

        # 압축 이미지 구독
        self.create_subscription(CompressedImage, '/camera/image/compressed', self.image_callback, 10)

        # 검출할 대상 클래스
        self.target_classes = {'cup', 'bottle', 'spoon', 'fork', 'knife', 'bowl'}

    def image_callback(self, msg):
        # 압축 이미지를 OpenCV 이미지로 디코딩
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warn('Failed to decode image')
            return

        detected_labels = set()

        # YOLOv5 추론
        results = self.model.predict(source=frame, device='cpu', stream=False)

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls.item())
                label = self.model.names[cls_id]

                if label not in self.target_classes:
                    continue

                detected_labels.add(label)
                x1, y1, x2, y2 = [int(i) for i in box.xyxy[0]]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 결과 이미지 퍼블리시
        detected_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.detected_pub.publish(detected_msg)

        # 검출된 객체 이름 퍼블리시
        labels_str = ', '.join(sorted(detected_labels)) if detected_labels else 'no objects'
        string_msg = String()
        string_msg.data = labels_str
        self.objects_pub.publish(string_msg)
        self.get_logger().info(f'Detected: {labels_str}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
