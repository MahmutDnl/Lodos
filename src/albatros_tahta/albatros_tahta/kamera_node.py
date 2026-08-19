import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('auto_search_index', True)

        cam_idx = int(self.get_parameter('camera_index').value)
        cam_id = int(self.get_parameter('camera_id').value)
        self.camera_index = cam_id if cam_id != 0 else cam_idx
        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.auto_search_index = bool(self.get_parameter('auto_search_index').value)

        self.publisher_ = self.create_publisher(Image, '/albatros/kamera/image_raw', 10)

        self.cap = None
        self.consecutive_failures = 0
        self.max_failures_before_reconnect = 5
        self.last_reconnect_time = 0.0

        # Kamerayı başlat
        self.open_camera(self.camera_index)

        timer_period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(timer_period, self.publish_frame)

    def open_camera(self, index: int) -> bool:
        """Belirtilen indeksteki kamerayı açar ve yapılandırır."""
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

        self.get_logger().info(f'Kamera açılıyor... (index: {index})')

        # 1. Öncelik: V4L2 Backend + MJPG
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not cap.isOpened():
            # 2. Öncelik: Varsayılan Backend
            cap = cv2.VideoCapture(index)

        if not cap.isOpened():
            self.get_logger().warn(f'Kamera (index: {index}) açılamadı.')
            return False

        # Çözünürlük ve FPS ayarla
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        # Test oku: Gerçekten kare alabiliyor mu?
        ret = False
        for _ in range(3):  # 3 deneme hakkı ver (bazı kameralar ilk karede boş döner)
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                break
            time.sleep(0.1)

        if not ret:
            # MJPG olmadan dene
            cap.set(cv2.CAP_PROP_FOURCC, 0)
            for _ in range(3):
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    break
                time.sleep(0.1)

        if ret and frame is not None and frame.size > 0:
            self.cap = cap
            self.camera_index = index
            self.consecutive_failures = 0
            self.get_logger().info(
                f'Kamera (index: {index}) başarıyla bağlandı! '
                f'Çözünürlük: {frame.shape[1]}x{frame.shape[0]}'
            )
            return True

        cap.release()
        self.get_logger().warn(f'Kamera (index: {index}) açıldı ancak kare okunamadı.')
        return False

    def find_and_open_working_camera(self):
        """Çalışan ilk kamera indeksini bulup açar (0, 1, 2, 4, 6...)."""
        candidate_indices = [self.camera_index, 0, 1, 2, 3, 4, 6]
        # Tekrarlayanları kaldır, sırayı koru
        seen = set()
        unique_candidates = [x for x in candidate_indices if not (x in seen or seen.add(x))]

        for idx in unique_candidates:
            if self.open_camera(idx):
                return True

        self.get_logger().error('Çalışan hiçbir kamera cihazı bulunamadı!')
        return False

    def publish_frame(self):
        if self.cap is None or not self.cap.isOpened():
            now = time.monotonic()
            if now - self.last_reconnect_time >= 3.0:
                self.last_reconnect_time = now
                if self.auto_search_index:
                    self.find_and_open_working_camera()
                else:
                    self.open_camera(self.camera_index)
            return

        ret, frame = self.cap.read()

        if ret and frame is not None and frame.size > 0:
            self.consecutive_failures = 0

            if len(frame.shape) == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif len(frame.shape) == 3 and frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            if frame.dtype != 'uint8':
                frame = frame.astype(np.uint8)

            image_msg = Image()
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_frame'
            image_msg.height = frame.shape[0]
            image_msg.width = frame.shape[1]
            image_msg.encoding = 'bgr8'
            image_msg.is_bigendian = False
            image_msg.step = frame.shape[1] * 3
            image_msg.data = frame.tobytes()

            self.publisher_.publish(image_msg)
        else:
            self.consecutive_failures += 1
            self.get_logger().warn(
                f'Kameradan görüntü alınamadı ({self.consecutive_failures}/{self.max_failures_before_reconnect}).'
            )

            if self.consecutive_failures >= self.max_failures_before_reconnect:
                self.get_logger().warn('Üst üste görüntü alma hatası! Kamera yeniden bağlanıyor...')
                if self.cap is not None:
                    self.cap.release()
                    self.cap = None
                self.consecutive_failures = 0

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()