#!/usr/bin/env python3
import os, json, threading, time
import numpy as np
import cv2

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge


class YoloLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('yolo_node')

        self.declare_parameter('pt_path',     '/root/.cache/ultralytics/yolo11n.pt')
        self.declare_parameter('engine_path', '/root/.cache/ultralytics/yolo11n.engine')
        self.declare_parameter('camera_topic','front_camera/color/image_raw')

        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf',  0.25)
        self.declare_parameter('device', 0)  

        self.declare_parameter('use_tracking', True)
        self.declare_parameter('tracker_cfg',  'botsort.yaml')

        self.declare_parameter('draw_labels',  True)
        self.declare_parameter('label_every_n', 1)
        self.declare_parameter('line_thickness', 2)
        self.declare_parameter('font_scale', 0.5)
        self.declare_parameter('font_thickness', 1)

        self.bridge: CvBridge | None = None
        self._colors = None
        self._frame_idx = 0
        self._lock = threading.Lock()
        self._latest_frame = None
        self._latest_header = None
        self._running = False
        self._worker: threading.Thread | None = None
        self._active = False

        self.model = None
        self.class_names = {}

        self.image_pub = None
        self.compressed_pub = None
        self.json_pub = None
        self.sub = None


    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring YOLO node…')
        try:
            self.pt_path     = self.get_parameter('pt_path').value
            self.engine_path = self.get_parameter('engine_path').value
            self.camera_topic= self.get_parameter('camera_topic').value

            self.imgsz = int(self.get_parameter('imgsz').value)
            self.conf  = float(self.get_parameter('conf').value)
            self.device= int(self.get_parameter('device').value)

            self.use_tracking = bool(self.get_parameter('use_tracking').value)
            self.tracker_cfg  = self.get_parameter('tracker_cfg').value

            self.draw_labels    = bool(self.get_parameter('draw_labels').value)
            self.label_every_n  = int(self.get_parameter('label_every_n').value)
            self.line_thickness = int(self.get_parameter('line_thickness').value)
            self.font_scale     = float(self.get_parameter('font_scale').value)
            self.font_thickness = int(self.get_parameter('font_thickness').value)
            self.font = cv2.FONT_HERSHEY_SIMPLEX

            self.bridge = CvBridge()

            self.image_pub = self.create_lifecycle_publisher(Image, 'yolo/image', 10)
            self.compressed_pub = self.create_lifecycle_publisher(CompressedImage, 'yolo/image_compressed', 10)
            self.json_pub  = self.create_lifecycle_publisher(String, 'yolo/detections', 10)

            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            )
            self.sub = self.create_subscription(Image, self.camera_topic, self.image_callback, qos)

            from ultralytics import YOLO

            if not os.path.exists(self.engine_path):
                self.get_logger().info(f"Exporting TensorRT engine: imgsz={self.imgsz}, FP16, static shapes")
                model_tmp = YOLO(self.pt_path)
                model_tmp.export(format="engine", device=self.device, half=True, imgsz=self.imgsz, dynamic=False, workspace=2048)
            else:
                self.get_logger().info("Found existing engine, skipping export.")

            self.model = YOLO(self.engine_path)
            self.model.overrides['imgsz']   = self.imgsz
            self.model.overrides['conf']    = self.conf
            self.model.overrides['device']  = self.device
            self.model.overrides['verbose'] = False
            self.class_names = self.model.names if hasattr(self.model, "names") else {}

            dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
            _ = self.model.predict(dummy, imgsz=self.imgsz, conf=self.conf, device=self.device, verbose=False)

            self._colors = self._make_colors(256)

            self.get_logger().info("YOLO configured.")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'on_configure() failed: {e}')
            self._cleanup_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating YOLO node…')
        try:
            ret = super().on_activate(state)
            if ret != TransitionCallbackReturn.SUCCESS:
                return ret

            self._active = True
            self._running = True
            self._worker = threading.Thread(target=self.inference_loop, daemon=True)
            self._worker.start()
            self.get_logger().info("YOLO active (tracking ON).")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'on_activate() failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating YOLO node…')
        self._active = False
        self._running = False
        try:
            if self._worker and self._worker.is_alive():
                self._worker.join(timeout=1.0)
        except Exception:
            pass
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up YOLO node…')
        self._active = False
        self._running = False
        try:
            if self._worker and self._worker.is_alive():
                self._worker.join(timeout=1.0)
        except Exception:
            pass

        for h, destroy in [
            (self.sub, self.destroy_subscription),
            (self.image_pub, self.destroy_publisher),
            (self.compressed_pub, self.destroy_publisher),
            (self.json_pub, self.destroy_publisher),
        ]:
            if h is not None:
                try: destroy(h)
                except Exception: pass
        self.sub = self.image_pub = self.compressed_pub = self.json_pub = None

        self._cleanup_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down YOLO node…')
        self._active = False
        self._running = False
        try:
            if self._worker and self._worker.is_alive():
                self._worker.join(timeout=1.0)
        except Exception:
            pass
        self._cleanup_resources()
        return TransitionCallbackReturn.SUCCESS

    def _cleanup_resources(self):
        self.model = None
        self.bridge = None
        self._colors = None
        with self._lock:
            self._latest_frame = None
            self._latest_header = None

    def _make_colors(self, n):
        rng = np.random.default_rng(0)
        return [tuple(int(x) for x in rng.integers(0, 255, size=3)) for _ in range(n)]

    def _compress_frame(self, frame, size=(640, 360), quality=20):
        try:
            small = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
            ok, jpeg = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if not ok:
                return None
            return jpeg.tobytes()
        except Exception as e:
            self.get_logger().warn(f'Compression error: {e}')
            return None

    def image_callback(self, msg: Image):
        try:
            if self.bridge is None:
                return
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return
        with self._lock:
            self._latest_frame = cv_image
            self._latest_header = msg.header

    def inference_loop(self):
        from sensor_msgs.msg import CompressedImage
        while self._running and rclpy.ok():
            if not self._active or self.model is None:
                time.sleep(0.01)
                continue

            frame, header = None, None
            with self._lock:
                if self._latest_frame is not None:
                    frame = self._latest_frame
                    header = self._latest_header
                    self._latest_frame = None

            if frame is None:
                time.sleep(0.001)
                continue

            self._frame_idx += 1

            h0, w0 = frame.shape[:2]
            if (w0, h0) != (self.imgsz, self.imgsz):
                frame_in = cv2.resize(frame, (self.imgsz, self.imgsz), interpolation=cv2.INTER_LINEAR)
                scale_x = w0 / float(self.imgsz)
                scale_y = h0 / float(self.imgsz)
            else:
                frame_in = frame
                scale_x = scale_y = 1.0

            try:
                if self.use_tracking:
                    results = self.model.track(
                        frame_in, persist=True, tracker=self.tracker_cfg,
                        imgsz=self.imgsz, conf=self.conf, device=self.device, verbose=False
                    )[0]
                else:
                    results = self.model.predict(
                        frame_in, imgsz=self.imgsz, conf=self.conf, device=self.device, verbose=False
                    )[0]
            except Exception as e:
                self.get_logger().warn(f"Inference error: {e}")
                continue

            dets_list = []
            boxes = getattr(results, 'boxes', None)
            if boxes is not None and len(boxes) > 0:
                confs = boxes.conf.view(-1).detach().cpu().numpy() if getattr(boxes, "conf", None) is not None else []
                clses = boxes.cls.view(-1).detach().cpu().numpy()  if getattr(boxes, "cls", None)  is not None else []
                tids  = (boxes.id.view(-1).detach().cpu().numpy()  if self.use_tracking and getattr(boxes, "id", None) is not None else [])
                xyxy  = boxes.xyxy.detach().cpu().numpy()

                for i in range(len(xyxy)):
                    cls_id = int(clses[i]) if i < len(clses) else -1
                    label  = self.class_names.get(cls_id, str(cls_id)) if cls_id >= 0 else "unknown"
                    conf   = float(confs[i]) if i < len(confs) else 0.0
                    tid    = int(tids[i]) if (self.use_tracking and i < len(tids) and tids[i] is not None and int(tids[i]) >= 0) else None
                    dets_list.append({
                        "object_id": str(cls_id if cls_id >= 0 else -1),
                        "object": label,
                        "confidence": f"{conf:.3f}",
                        "track_id": tid
                    })

                annotated = self.fast_draw(frame, xyxy, clses, confs, tids, scale_x, scale_y)

                out_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
                if header: out_msg.header = header
                self.image_pub.publish(out_msg)

                jpeg_bytes = self._compress_frame(annotated, size=(640, 360), quality=20)
                if jpeg_bytes is not None:
                    comp = CompressedImage()
                    if header: comp.header = header
                    comp.format = "jpeg"
                    comp.data = jpeg_bytes
                    self.compressed_pub.publish(comp)
            else:
                out_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                if header: out_msg.header = header
                self.image_pub.publish(out_msg)

                jpeg_bytes = self._compress_frame(frame, size=(640, 360), quality=20)
                if jpeg_bytes is not None:
                    comp = CompressedImage()
                    if header: comp.header = header
                    comp.format = "jpeg"
                    comp.data = jpeg_bytes
                    self.compressed_pub.publish(comp)

            payload = {"detections_num": str(len(dets_list)), "detected_objects": dets_list}
            msg_json = String()
            msg_json.data = json.dumps(payload, separators=(',', ':'))
            self.json_pub.publish(msg_json)

    def fast_draw(self, frame_bgr, xyxy, clses, confs, tids, sx, sy):
        img = frame_bgr
        H, W = img.shape[:2]
        lt = self.line_thickness
        draw_text = self.draw_labels and (self._frame_idx % self.label_every_n == 0)
        color = (183, 29, 130)  # fixed BGR

        for i in range(len(xyxy)):
            x1, y1, x2, y2 = xyxy[i]
            x1 = int(x1 * sx); y1 = int(y1 * sy); x2 = int(x2 * sx); y2 = int(y2 * sy)
            x1 = 0 if x1 < 0 else (W-1 if x1 >= W else x1)
            y1 = 0 if y1 < 0 else (H-1 if y1 >= H else y1)
            x2 = 0 if x2 < 0 else (W-1 if x2 >= W else x2)
            y2 = 0 if y2 < 0 else (H-1 if y2 >= H else y2)

            cv2.rectangle(img, (x1, y1), (x2, y2), color, lt)

            if draw_text:
                conf = confs[i] if i < len(confs) else 0.0
                tid  = int(tids[i]) if (self.use_tracking and i < len(tids) and tids[i] is not None and int(tids[i]) >= 0) else -1
                cls_id = int(clses[i]) if i < len(clses) else 0
                label = self.class_names.get(cls_id, str(cls_id))
                s = f"{label} {conf:.2f}" if tid < 0 else f"{label} {conf:.2f} id:{tid}"

                (tw, th), baseline = cv2.getTextSize(s, self.font, self.font_scale, self.font_thickness)
                x2bg = min(x1 + tw + 4, W - 1)
                y2bg = min(y1 + th + baseline + 4, H - 1)
                cv2.rectangle(img, (x1, y1), (x2bg, y2bg), color, thickness=-1)
                cv2.putText(img, s, (x1 + 2, y1 + th + 1), self.font, self.font_scale, (255,255,255), self.font_thickness, cv2.LINE_AA)
        return img


def main(args=None):
    rclpy.init(args=args)
    node = YoloLifecycleNode()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()