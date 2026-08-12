#!/usr/bin/env python3
# =============================================================================
# LODOS Albatros — YOLO Node Path Resolution Unit Tests
# =============================================================================
# ROS2 Jazzy / Ubuntu 24.04
# =============================================================================

import os
import unittest
from pathlib import Path

from albatros_tahta.yolo_node import YoloNode


class TestYoloNodePathResolution(unittest.TestCase):

    def test_resolve_model_path_existing_parkur12(self):
        """parkur12.hef dosyasının package share veya workspace altında başarıyla çözüldüğünü doğrular."""
        # Method test without fully instantiating Node (mocking get_logger if needed)
        resolved_path, attempted = YoloNode.resolve_model_path(None, "models/parkur12.hef", "parkur12")
        self.assertIsNotNone(resolved_path)
        self.assertTrue(resolved_path.exists())
        self.assertTrue(resolved_path.name.endswith(".hef"))

    def test_resolve_model_path_fallback_to_yolov11s(self):
        """Olmayan hef arandığında yolov11s.hef fallback yapıldığını doğrular."""
        resolved_path, attempted = YoloNode.resolve_model_path(None, "models/nonexistent_model.hef", "custom")
        self.assertIsNotNone(resolved_path)
        self.assertTrue(resolved_path.exists())
        self.assertEqual(resolved_path.name, "yolov11s.hef")

    def test_resolve_model_path_absolute(self):
        """Absolute path verilirse doğrudan döndürüldüğünü doğrular."""
        real_file = Path("/home/lodos/Lodos/models/yolov11s.hef")
        resolved_path, attempted = YoloNode.resolve_model_path(None, str(real_file), "absolute")
        self.assertEqual(resolved_path, real_file.resolve())


if __name__ == '__main__':
    unittest.main()
