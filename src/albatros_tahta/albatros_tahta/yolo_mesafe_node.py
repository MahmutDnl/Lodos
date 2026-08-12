#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class YoloMesafeNode(Node):

    def __init__(self):
        super().__init__('yolo_mesafe_node')

        # --- Parameters ---
        self.declare_parameter('input_topic', '/albatros/yolo/tespitler')
        self.declare_parameter('output_topic', '/albatros/yolo/mesafe_aci')
        
        # GEÇİCİ KALİBRASYON DEĞERLERİ:
        # fx, cx ve distance_width_constant değerleri geçici kalibrasyon değerleridir.
        # Bu değerler gerçek kamera ve duba testlerinden sonra güncellenecektir.
        self.declare_parameter('fx', 900.0)
        self.declare_parameter('cx', 640.0)
        self.declare_parameter('distance_width_constant', 360.0)
        
        self.declare_parameter('min_yolo_confidence', 0.60)
        self.declare_parameter('min_bbox_width_px', 20)
        self.declare_parameter('max_visual_distance_m', 20.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.fx = self.get_parameter('fx').value
        self.cx = self.get_parameter('cx').value
        self.distance_width_constant = self.get_parameter('distance_width_constant').value
        self.min_yolo_confidence = self.get_parameter('min_yolo_confidence').value
        self.min_bbox_width_px = self.get_parameter('min_bbox_width_px').value
        self.max_visual_distance_m = self.get_parameter('max_visual_distance_m').value

        # --- Subscriptions and Publishers ---
        self.subscription = self.create_subscription(
            String,
            self.input_topic,
            self.detections_callback,
            10
        )

        self.publisher = self.create_publisher(
            String,
            self.output_topic,
            10
        )

        self.get_logger().info(f"Yolo Mesafe Node başlatıldı.")
        self.get_logger().info(f"Abone olunan topic: {self.input_topic}")
        self.get_logger().info(f"Yayın yapılan topic: {self.output_topic}")

    def detections_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"JSON parse hatası: {e}")
            return

        stamp = data.get('stamp', 0.0)
        frame_id = data.get('frame_id', 'camera_frame')
        raw_detections = data.get('detections', [])

        processed_detections = []

        for det in raw_detections:
            confidence = det.get('confidence', 0.0)
            if confidence < self.min_yolo_confidence:
                continue

            bbox = det.get('bbox')
            if not bbox:
                continue

            # Bounding box genişliğini belirle
            bbox_width_px = bbox.get('width')
            if bbox_width_px is None:
                x_max = bbox.get('x_max')
                x_min = bbox.get('x_min')
                if x_max is not None and x_min is not None:
                    bbox_width_px = x_max - x_min
                else:
                    continue

            if bbox_width_px <= 0 or bbox_width_px < self.min_bbox_width_px:
                continue

            # Bounding box yatay merkezini belirle
            center_x = None
            center = det.get('center')
            if center:
                center_x = center.get('x')

            if center_x is None:
                x_max = bbox.get('x_max')
                x_min = bbox.get('x_min')
                if x_max is not None and x_min is not None:
                    center_x = (x_min + x_max) / 2.0
                else:
                    continue

            # Açı hesabı
            angle_rad, angle_deg = self.calculate_bearing(center_x)

            # Mesafe hesabı
            distance_m = self.calculate_distance(bbox_width_px)
            if distance_m <= 0 or distance_m > self.max_visual_distance_m:
                continue

            # Gövde koordinatları hesabı (ROS base_link standardına göre: x ileri, y sol)
            x_body_m = distance_m * math.cos(angle_rad)
            y_body_m = -distance_m * math.sin(angle_rad)

            processed_detections.append({
                "class_id": det.get('class_id', 0),
                "class_name": det.get('class_name', 'unknown'),
                "yolo_confidence": confidence,
                "bbox_width_px": int(bbox_width_px),
                "center_x_px": float(center_x),
                "bearing_deg": float(angle_deg),
                "visual_distance_m": float(distance_m),
                "x_body_m": float(x_body_m),
                "y_body_m": float(y_body_m),
                "valid": True
            })

        payload = {
            "stamp": stamp,
            "frame_id": frame_id,
            "source": "yolo_mesafe_node",
            "detection_count": len(processed_detections),
            "detections": processed_detections
        }

        self.publish_results(payload)

    def calculate_bearing(self, center_x):
        # Açı hesabı
        angle_rad = math.atan2(center_x - self.cx, self.fx)
        angle_deg = math.degrees(angle_rad)
        return angle_rad, angle_deg

    def calculate_distance(self, bbox_width_px):
        # Mesafe hesabı
        return self.distance_width_constant / bbox_width_px

    def publish_results(self, payload):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloMesafeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
