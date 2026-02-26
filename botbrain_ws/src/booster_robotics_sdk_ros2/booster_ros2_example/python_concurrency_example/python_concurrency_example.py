import time
import numpy as np
from booster_robotics_sdk_python import ChannelFactory, B1LowStateSubscriber
from booster_interface.msg import LowCmd, MotorCmd
import threading
import queue
import rclpy
import math
import multiprocessing as mp
from multiprocessing.shared_memory import SharedMemory

# ===== Config =====
JOINT_DIM = 14  # Joint dimension

def joint_state_thread(joint_queue, exit_event):
    """Thread: Collect joint states and push to queue"""
    ChannelFactory.Instance().Init(0)

    class JointStateHandler:
        def __init__(self, queue_):
            self.queue = queue_
            self.joint_q = np.zeros(JOINT_DIM, dtype=np.float32)
            self.frame_count = 0
            self.last_time = time.time()

        def __call__(self, low_state_msg):
            # Copy q of motors 2~15 into the array
            for i, motor in enumerate(low_state_msg.motor_state_serial[2:2+JOINT_DIM]):
                self.joint_q[i] = motor.q

            # Push to queue (drop if overflow)
            try:
                self.queue.put(self.joint_q.copy(), timeout=0.01)
            except queue.Full:
                pass

            # FPS statistics (every 2 seconds)
            self.frame_count += 1
            now = time.time()
            elapsed = now - self.last_time
            if elapsed >= 2.0:
                fps = self.frame_count / elapsed if elapsed > 0 else 0
                print(f"[JointStateHandler] 2s average FPS: {fps:.2f} Hz")
                self.frame_count = 0
                self.last_time = now

    handler = JointStateHandler(joint_queue)
    channel_subscriber = B1LowStateSubscriber(handler)
    channel_subscriber.InitChannel()
    try:
        while not exit_event.is_set():
            time.sleep(0.001)
    finally:
        try:
            channel_subscriber.CloseChannel()
        except Exception:
            pass

def ros2_pub_thread(exit_event):
    """Thread: ROS2 LowCmd publisher"""
    rclpy.init()
    from rclpy.node import Node

    class SimplePubNode(Node):
        def __init__(self):
            super().__init__('simple_pub_node')
            self.publisher = self.create_publisher(LowCmd, 'test_control', 10)
            self.frame_count = 0
            self.last_time = time.time()

        def pub_loop(self):
            rate = 1.0 / 500  # 500Hz
            while not exit_event.is_set():
                now = time.time()
                # Publish motor control command
                value = math.sin(now) + 0.5 * math.cos(2 * now) + np.random.normal(0, 0.01)
                msg = LowCmd()
                mc = MotorCmd()
                mc.q = value
                msg.motor_cmd.append(mc)
                self.publisher.publish(msg)
                self.frame_count += 1

                elapsed = now - self.last_time
                if elapsed >= 2.0:
                    fps = self.frame_count / elapsed if elapsed > 0 else 0
                    print(f"[SimplePubNode] 2s average publish FPS: {fps:.2f} Hz, last value: {value:.4f}")
                    self.frame_count = 0
                    self.last_time = now

                time.sleep(rate)

    node = SimplePubNode()
    try:
        node.pub_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

# ===== Mock computation process reads latest joint feedback =====
def cpu_intensive_process(exit_event, shm_name):
    """Process: High CPU load computation, read latest joint q from shared memory"""
    print("[CPUProc] High CPU load process started...")
    # Connect to existing shared memory
    shm = SharedMemory(name=shm_name)
    try:
        joint_view = np.ndarray((JOINT_DIM,), dtype=np.float32, buffer=shm.buf)
        count = 0
        total = 0.0
        last_time = time.time()

        while not exit_event.is_set():
            # Read latest joint (lock-free snapshot; for stricter needs use double buffer or seqlock)
            q = joint_view.copy()
            total += float(np.sum(np.sin(q) * np.cos(q)))
            count += 1

            # Statistics output
            now = time.time()
            if now - last_time >= 2.0:
                print(f"[CPUProc] 2s total float ops: {count} times, current total={total:.2f}")
                count = 0
                last_time = now

            # Yield time slice appropriately (prevent full core usage affecting other threads)
            # Note: Remove this line if you want maximum throughput
            # time.sleep(0.0005)
    finally:
        shm.close()
        print("[CPUProc] Exiting")

if __name__ == "__main__":
    # Process event: for thread and process exit
    exit_event = mp.Event()

    # Inter-thread queue: from joint subscription thread to main thread
    joint_queue = queue.Queue(maxsize=10)

    # Create shared memory, for "main thread writes latest joint q", CPU process reads
    shm = SharedMemory(create=True, size=JOINT_DIM * 4)
    joint_buf = np.ndarray((JOINT_DIM,), dtype=np.float32, buffer=shm.buf)
    joint_buf[:] = 0.0

    # Start subscription and publishing threads (using mp.Event as normal Event in threads is OK)
    t_sub = threading.Thread(target=joint_state_thread, args=(joint_queue, exit_event), daemon=True)
    t_pub = threading.Thread(target=ros2_pub_thread, args=(exit_event,), daemon=True)
    t_sub.start()
    t_pub.start()

    # Start CPU process
    cpu_proc = mp.Process(target=cpu_intensive_process, args=(exit_event, shm.name), daemon=True)
    cpu_proc.start()

    try:
        while True:
            try:
                qv = joint_queue.get(timeout=1.0)
                # Write latest joint to shared memory (one-shot slice copy)
                joint_buf[:] = qv
            except Exception:
                print("No joint data received")
    except KeyboardInterrupt:
        print("Received interrupt signal, exiting...")
        exit_event.set()
        # Wait for threads and process to exit
        t_sub.join(timeout=2.0)
        t_pub.join(timeout=2.0)
        cpu_proc.join(timeout=2.0)
    finally:
        # Release shared memory
        try:
            shm.close()
            shm.unlink()
        except Exception:
            pass
        print("Cleanup complete, exited.")
