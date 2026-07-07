这是 **Ctrl+C 信号与 ROS2 shutdown 的竞态条件** + **`cam.close_cam()` 与 `cam-service` 冲突** 导致的。

---

### 原因

1. **信号处理重复**：你的 `signal_handler` 调了 `rclpy.shutdown()`，`finally` 块又调了一次，导致 `RuntimeError: Context must be initialized before it can be shutdown`
2. **`cam.close_cam()` 卡死**：`cam-service` 在后台占着 ISP，你的 `destroy_node()` 里调用 `close_cam()` 试图停掉 VPF pipeline，但 `cam-service` 不让，所以 `hbn_vflow_stop` 报 `ret(-11)`，线程挂住等待
3. **ROS2 executor 等 timer 结束**：`rclpy.shutdown()` 要等待所有 callback 完成，但 `capture()` 卡在 `cv2_to_imgmsg` 或 `close_cam` 里，所以终端迟迟不返回

---

### 修复：不要 `close_cam()`，让 `cam-service` 维持状态

既然 `cam-service` 必须常驻后台来维持 ISP，你的 ROS2 节点退出时**不需要关闭摄像头硬件**，只释放 ROS2 资源即可。下次启动节点时，`open_cam` 会复用已有的 ISP 状态。

把 `mipi_camera_node.py` 改成这样：

```python
#!/usr/bin/env python3
"""MIPI camera driver node for RDK X5."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class MipiCameraNode(Node):
    def __init__(self):
        super().__init__('mipi_camera_node')

        # Parameters
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'camera')

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value

        # Import hobot_vio
        try:
            from hobot_vio import libsrcampy as srcampy
            self.srcampy = srcampy
        except ImportError as e:
            self.get_logger().error(f'Failed to import hobot_vio: {e}')
            rclpy.shutdown()
            sys.exit(1)

        self.cam = self.srcampy.Camera()
        self.get_logger().info(
            f'Opening MIPI camera: {self.width}x{self.height}')

        # 官方参数顺序：小分辨率在前，大分辨率在后
        ret = self.cam.open_cam(
            0, -1, -1,
            [512, self.width],
            [512, self.height],
            self.height, self.width
        )
        if ret != 0:
            self.get_logger().error(f'open_cam failed: {ret}')
            raise RuntimeError('Camera init failed')

        self.get_logger().info('Camera opened successfully')

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)

        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.capture)
        self.frame_count = 0

    def _nv12_to_bgr(self, nv12_data, width, height):
        expected_size = int(width * height * 1.5)
        if len(nv12_data) != expected_size:
            self.get_logger().warn(
                f'Buffer size mismatch: {len(nv12_data)} vs {expected_size}')
            return None

        nv12 = np.frombuffer(nv12_data, dtype=np.uint8).reshape(
            (int(height * 1.5), width))
        return cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)

    def capture(self):
        try:
            img_buf = self.cam.get_img(2, 512, 512)
            if img_buf is None:
                return

            frame_512 = self._nv12_to_bgr(img_buf, 512, 512)
            if frame_512 is None:
                return

            # 放大到目标分辨率
            frame = cv2.resize(frame_512, (self.width, self.height))

            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.pub.publish(msg)

            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')

        except Exception as e:
            self.get_logger().error(f'Capture error: {e}')

    def destroy_node(self):
        # 关键：不要 close_cam()！cam-service 维持 ISP 状态
        self.get_logger().info('Stopping camera node (camera kept alive)')
        self.timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MipiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received')
    finally:
        node.destroy_node()
        # 只 shutdown 一次，不要重复
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### 关键修改点

| 原代码                                        | 修复后                                                |
| --------------------------------------------- | ----------------------------------------------------- |
| 自定义 `signal_handler` 调 `rclpy.shutdown()` | 删除自定义 handler，用 `try/except KeyboardInterrupt` |
| `finally` 里调 `rclpy.shutdown()`             | `if rclpy.ok(): rclpy.shutdown()` 防止重复            |
| `destroy_node()` 里 `cam.close_cam()`         | **删除 `close_cam()`**，避免和 `cam-service` 冲突     |
| `destroy_node()` 只调 `super()`               | 加 `self.timer.cancel()` 停止定时器                   |

---

### 验证

替换代码后重新编译运行：

```bash
cd ~/dev_ws
colcon build --packages-select sentry_bringup
source install/setup.bash
ros2 run sentry_bringup mipi_camera_node
```

按 **Ctrl+C** 应该立刻退出，终端秒回。如果还有问题，把日志贴给我。